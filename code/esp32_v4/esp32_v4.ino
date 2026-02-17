// ===================== ESP32 + DHT22 + MQ2 + MQ4 + microSD (Supervisor-Style) =====================
// ADC(ESP) -> Vadc -> Vsensor(pre-divider) -> Rs -> (Rs/R0) -> PPM   |   CSV logger + Serial diagnostics
// NOTE: Use ADC1 pins (GPIO32-39) to avoid WiFi conflicts; add a voltage divider from MQ AO to ESP32 ADC.

// ------------------- Includes -------------------
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <math.h>

// ------------------- Pins / Peripherals (ESP32) -------------------
// DHT22 can be anywhere (GPIO 0..39); use a free GPIO and add a 10k pull-up if needed.
#define DHTPIN             4
#define DHTTYPE            DHT22

// Use ADC1 pins for analog (safe with WiFi). GPIO34/35 are input-only (perfect for ADC).
#define MQ2_ADC_PIN       34          // AO -> voltage divider -> GPIO34
#define MQ4_ADC_PIN       35          // AO -> voltage divider -> GPIO35

// VSPI default pins on ESP32
#define SD_CS_PIN          5          // VSPI CS
#define SD_SCK_PIN        18          // VSPI SCK
#define SD_MISO_PIN       19          // VSPI MISO
#define SD_MOSI_PIN       23          // VSPI MOSI

// ------------------- Timings -------------------
#define SAMPLE_INTERVAL_MS  5000UL     // one row / 5 s (your current setting)
#define WARMUP_MS           2000UL     // short warmup chatter; extend in real use

// ------------------- ADC / HW constants -------------------
// ESP32 ADC is ~12-bit (0..4095). We'll use a simple scale with 11dB attenuation (~0..3.6V).
// For accuracy, keep wiring short and stable; ESP32 ADC has some non-linearity (OK for this use).
static const float ADC_VREF          = 3.3f;     // ESP32 ADC reference (approx, for scaling)
static const int   ADC_BITS          = 12;       // 12-bit ADC
static const int   ADC_MAX           = (1 << ADC_BITS) - 1;  // 4095
static const float VMIN              = 0.01f;    // guard
static const float PPM_MAX_CLAMP     = 65000.0f;

// ---- Sensor supply voltage (what powers the MQ divider RESISTOR network) ----
// Most MQ boards’ AO sits on a divider referenced to their supply (often 5.0V).
// If your MQ board’s divider runs from 5V, keep this at 5.0. If you run the divider from 3.3V, set 3.3.
static const float SENSOR_VCC        = 5.0f;     // **IMPORTANT**: the voltage used in Rs = RL*(Vcc - V)/V

// ---- Voltage divider from sensor AO to ESP32 ADC ----
// If you use a divider Rtop (to sensor AO) and Rbot (to GND), the ADC sees: Vadc = Vsensor * (Rbot / (Rtop + Rbot))
// Define the **gain** to reconstruct Vsensor:  Vsensor = Vadc * ((Rtop + Rbot) / Rbot)
static const float MQ2_DIV_GAIN      = 2.00f;    // e.g., 100k(top) / 100k(bot) => gain 2.0  (5V -> 2.5V max)
static const float MQ4_DIV_GAIN      = 2.00f;    // set to your actual divider ratio

// ---- MQ hardware: measured load resistors (EDIT if different) ----
static const float MQ2_RL_OHMS       = 13710.0f; // your measured MQ-2 RL (Ω)
static const float MQ4_RL_OHMS       =   998.0f; // your measured MQ-4 RL (Ω)

// ---- MQ calibration: clean-air baselines (SET after your calibration) ----
static const float MQ2_R0_OHMS       = 10000.0f; // TODO: replace with your R0
static const float MQ4_R0_OHMS       =  1000.0f; // TODO: replace with your R0

// ------------------- PPM model (power-law) -------------------
// ppm = A * (Rs/R0)^B    |   tune A,B from your calibration (B usually < 0)
static const float MQ2_A =   50.0f;    // hydrocarbons proxy for MQ-2
static const float MQ2_B =   -2.30f;
static const float MQ4_A = 1000.0f;    // methane proxy for MQ-4
static const float MQ4_B =   -2.95f;

// ------------------- Globals -------------------
DHT dht(DHTPIN, DHTTYPE);
static const char* CSV_FILE = "datalog.csv";
static bool sd_ok = false;

// ------------------- Helpers -------------------
static inline float adcToVadc(int adc) {
  return (float)adc * (ADC_VREF / (float)ADC_MAX);   // ESP ADC pin voltage (post-divider)
}

static inline float reconstructVsensor(float vadc, float gain) {
  return vadc * gain;                                 // back-calc pre-divider node voltage
}

static inline float mqRsFromVsensor(float v_sensor, float RL_ohms) {
  if (v_sensor < VMIN) return 1e9f;
  // Rs = RL * (Vcc - Vout) / Vout, where Vcc is the sensor divider supply (SENSOR_VCC)
  return (SENSOR_VCC - v_sensor) * RL_ohms / v_sensor;
}

static inline float ppm_from_ratio(float r, float A, float B) {
  if (r <= 0.0f) return 0.0f;
  float ppm = A * powf(r, B);
  if (!(ppm >= 0.0f)) ppm = 0.0f;
  if (ppm > PPM_MAX_CLAMP) ppm = PPM_MAX_CLAMP;
  return ppm;
}

static uint16_t readADCavg(int pin, int samples = 8) {
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) { acc += analogRead(pin); delay(2); }
  return (uint16_t)(acc / samples);
}

static File openAppend(const char* name) {
  File f = SD.open(name, FILE_WRITE);
  if (!f) { Serial.print(F("[SD] open FAIL: ")); Serial.println(name); return File(); }
  f.seek(f.size());
  return f;
}

static void ensureCsvHeader() {
  File f = SD.open(CSV_FILE, FILE_READ);
  bool needHeader = true;
  if (f) { if (f.size() > 0) needHeader = false; f.close(); }
  if (needHeader) {
    File w = SD.open(CSV_FILE, FILE_WRITE);
    if (w) {
      w.println(F("time_ms,"
                  "mq2_adc,mq2_vadc,mq2_vsns,mq2_Rs,mq2_Rs_R0,mq2_ppm,"
                  "mq4_adc,mq4_vadc,mq4_vsns,mq4_Rs,mq4_Rs_R0,mq4_ppm,"
                  "tempC,hum"));
      w.flush(); w.close();
      Serial.println(F("[CSV] header written"));
    } else {
      Serial.println(F("[CSV] header write FAIL"));
    }
  }
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("[BOOT] ESP32 DHT22+MQ2+MQ4+SD (PPM pipeline)"));

  // ADC config: 12-bit; set attenuation so full-scale ~3.6V (helps if divider lets up to ~2.5–3.0V in)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(MQ2_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(MQ4_ADC_PIN, ADC_11db);

  dht.begin();

  // SD on VSPI
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  Serial.print(F("[SD] init..."));
  sd_ok = SD.begin(SD_CS_PIN, SPI);
  Serial.println(sd_ok ? F("OK") : F("FAILED"));
  if (sd_ok) ensureCsvHeader();

  // short warm-up chatter (extend in real deployments)
  unsigned long t0 = millis();
  while (millis() - t0 <= WARMUP_MS) {
    delay(250);
    Serial.println(F("warming MQ sensors..."));
  }
}

// ------------------- LOOP -------------------
void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    // ---- MQ2: ADC -> Vadc -> Vsensor -> Rs -> (Rs/R0) -> PPM ----
    uint16_t adc2 = readADCavg(MQ2_ADC_PIN);
    float vadc2 = adcToVadc((int)adc2);
    float vsns2 = reconstructVsensor(vadc2, MQ2_DIV_GAIN);
    float rs2   = mqRsFromVsensor(vsns2, MQ2_RL_OHMS);
    float r02   = (MQ2_R0_OHMS > 0.0f ? MQ2_R0_OHMS : 1.0f);
    float ratio2= rs2 / r02;
    float ppm2  = ppm_from_ratio(ratio2, MQ2_A, MQ2_B);

    // ---- MQ4: ADC -> Vadc -> Vsensor -> Rs -> (Rs/R0) -> PPM ----
    uint16_t adc4 = readADCavg(MQ4_ADC_PIN);
    float vadc4 = adcToVadc((int)adc4);
    float vsns4 = reconstructVsensor(vadc4, MQ4_DIV_GAIN);
    float rs4   = mqRsFromVsensor(vsns4, MQ4_RL_OHMS);
    float r04   = (MQ4_R0_OHMS > 0.0f ? MQ4_R0_OHMS : 1.0f);
    float ratio4= rs4 / r04;
    float ppm4  = ppm_from_ratio(ratio4, MQ4_A, MQ4_B);

    // ---- DHT22 ----
    float hum = dht.readHumidity();
    float tC  = dht.readTemperature();
    if (isnan(hum)) hum = -1.0f;
    if (isnan(tC))  tC  = -1000.0f;

    // --------- human-readable printout ---------
    Serial.print(F("[SAMPLE] t_ms="));  Serial.print(now);

    Serial.print(F(" | MQ2 adc="));     Serial.print(adc2);
    Serial.print(F(" vadc="));          Serial.print(vadc2,3);
    Serial.print(F(" vsns="));          Serial.print(vsns2,3);
    Serial.print(F(" Rs="));            Serial.print(rs2,0);
    Serial.print(F(" Rs/R0="));         Serial.print(ratio2,3);
    Serial.print(F(" mq2_ppm="));       Serial.print(ppm2,0);

    Serial.print(F(" || MQ4 adc="));    Serial.print(adc4);
    Serial.print(F(" vadc="));          Serial.print(vadc4,3);
    Serial.print(F(" vsns="));          Serial.print(vsns4,3);
    Serial.print(F(" Rs="));            Serial.print(rs4,0);
    Serial.print(F(" Rs/R0="));         Serial.print(ratio4,3);
    Serial.print(F(" mq4_ppm="));       Serial.print(ppm4,0);

    Serial.print(F(" || tempC="));      Serial.print(tC,1);
    Serial.print(F(" hum%="));          Serial.println(hum,1);

    // ------------------- CSV append -------------------
    if (sd_ok) {
      File f = openAppend(CSV_FILE);
      if (f) {
        f.print(now);  f.print(',');
        f.print(adc2); f.print(','); f.print(vadc2,3); f.print(','); f.print(vsns2,3); f.print(',');
        f.print(rs2,0); f.print(','); f.print(ratio2,4); f.print(','); f.print(ppm2,0); f.print(',');
        f.print(adc4); f.print(','); f.print(vadc4,3); f.print(','); f.print(vsns4,3); f.print(',');
        f.print(rs4,0); f.print(','); f.print(ratio4,4); f.print(','); f.print(ppm4,0); f.print(',');
        f.print(tC,2);  f.print(','); f.println(hum,1);
        f.flush(); f.close();
        Serial.println(F("[CSV] appended"));
      } else {
        Serial.println(F("[SD] open FAIL"));
      }
    } else {
      Serial.println(F("[SD] not mounted"));
    }
  }
}
