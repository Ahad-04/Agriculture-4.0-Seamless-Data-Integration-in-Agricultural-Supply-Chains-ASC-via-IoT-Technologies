// ===================== UNO + DHT22 + MQ2 + MQ4 + microSD (Supervisor-Style) =====================
// ADC -> V -> Rs -> (Rs/R0) -> PPM   |   CSV logger + Serial diagnostics

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <math.h>

// ------------------- Pins / Peripherals -------------------
#define DHTPIN            7
#define DHTTYPE           DHT22
#define CS_PIN            10            // SD CS (UNO: 10)
#define MQ2_A_Pin         A0
#define MQ4_A_Pin         A1

// ------------------- Timings -------------------
#define SAMPLE_INTERVAL_MS   30000UL    // one row / 30 s  (matches supervisor cadence)
#define WARMUP_MS            2000UL     // short warmup chatter (extend to minutes in real use)

// ------------------- ADC / HW constants -------------------
static const float VREF            = 5.0f;     // UNO analog reference
static const float VMIN            = 0.01f;    // <10 mV -> guard
static const float PPM_MAX_CLAMP   = 65000.0f; // fits uint16_t if needed

// ---- MQ hardware: measured load resistors (EDIT if different) ----
static const float MQ2_RL_OHMS     = 14310.0f; // measured MQ-2 RL (Ω)
static const float MQ4_RL_OHMS     =   993.0f; //  measured MQ-4 RL (Ω)

// // // ---- MQ calibration: clean-air baselines (SET after your calibration) ----
// static const float MQ2_R0_OHMS     = 10000.0f; // TODO: replace with your R0
// static const float MQ4_R0_OHMS     =  1000.0f; // TODO: replace with your R0
static const float MQ2_R0_OHMS  = 5600.0f;
static const float MQ4_R0_OHMS  =  500.0f;

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
static inline float adcToVolt(int adc) {
  return (float)adc * (VREF / 1023.0f);
}

static inline float mqRsFromVolt(float v, float RL_ohms) {
  if (v < VMIN) return 1e9f;                 // guard
  return (VREF - v) * RL_ohms / v;           // Rs = RL * (Vcc - Vout) / Vout
}

static inline float ppm_from_ratio(float r, float A, float B) {
  if (r <= 0.0f) return 0.0f;
  float ppm = A * powf(r, B);
  if (!(ppm >= 0.0f)) ppm = 0.0f;            // NaN/inf guard
  if (ppm > PPM_MAX_CLAMP) ppm = PPM_MAX_CLAMP;
  return ppm;
}

static uint16_t readADCavg(uint8_t pin, int samples = 8) {
  // one dummy read to settle mux
  (void)analogRead(pin);
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) { acc += analogRead(pin); delay(2); }
  return (uint16_t)(acc / samples);
}

static File openAppend(const char* name) {
  File f = SD.open(name, FILE_WRITE);
  if (!f) { Serial.print(F("[SD] open FAIL: ")); Serial.println(name); return File(); }
  f.seek(f.size());                          // explicit append
  return f;
}

static void ensureCsvHeader() {
  File f = SD.open(CSV_FILE, FILE_READ);
  bool needHeader = true;
  if (f) { if (f.size() > 0) needHeader = false; f.close(); }
  if (needHeader) {
    File w = SD.open(CSV_FILE, FILE_WRITE);
    if (w) {
      w.println(F("time_ms,mq2_adc,mq2_V,mq2_Rs,mq2_Rs_R0,mq2_ppm,"
                  "mq4_adc,mq4_V,mq4_Rs,mq4_Rs_R0,mq4_ppm,tempC,hum"));
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
  Serial.println(F("[BOOT] UNO DHT22+MQ2+MQ4+SD (PPM pipeline)"));

  // keep SPI master mode & CS lines sane
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(CS_PIN, OUTPUT); digitalWrite(CS_PIN, HIGH);

  analogReference(DEFAULT);

  dht.begin();
  // Prime DHT once (first read often NaN)
  (void)dht.readHumidity(); (void)dht.readTemperature();

  Serial.print(F("[SD] init..."));
  sd_ok = SD.begin(CS_PIN);
  Serial.println(sd_ok ? F("OK") : F("FAILED"));
  if (sd_ok) ensureCsvHeader();

  unsigned long t0 = millis();
  while (millis() - t0 <= WARMUP_MS) {
    delay(250);
    Serial.println(F("[WARMUP] MQ sensors stabilizing..."));
  }
}

// ------------------- LOOP -------------------
void loop() {
  static uint32_t lastSample = 0;
  uint32_t now = millis();

  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    // ---- MQ2: ADC -> V -> Rs -> (Rs/R0) -> PPM ----
    uint16_t adc2 = readADCavg(MQ2_A_Pin);
    float v2 = adcToVolt((int)adc2);
    float rs2 = mqRsFromVolt(v2, MQ2_RL_OHMS);
    float r02 = (MQ2_R0_OHMS > 0.0f ? MQ2_R0_OHMS : 1.0f);
    float ratio2 = rs2 / r02;
    float ppm2 = ppm_from_ratio(ratio2, MQ2_A, MQ2_B);

    // ---- MQ4: ADC -> V -> Rs -> (Rs/R0) -> PPM ----
    uint16_t adc4 = readADCavg(MQ4_A_Pin);
    float v4 = adcToVolt((int)adc4);
    float rs4 = mqRsFromVolt(v4, MQ4_RL_OHMS);
    float r04 = (MQ4_R0_OHMS > 0.0f ? MQ4_R0_OHMS : 1.0f);
    float ratio4 = rs4 / r04;
    float ppm4 = ppm_from_ratio(ratio4, MQ4_A, MQ4_B);

    // ---- DHT22 ----
    float hum = dht.readHumidity();
    float tC  = dht.readTemperature();
    if (isnan(hum)) hum = -1.0f;
    if (isnan(tC))  tC  = -1000.0f;

    // --------- human-readable printout ---------
    Serial.print(F("[SAMPLE] t_ms="));  Serial.print(now);

    Serial.print(F(" | MQ2 adc="));     Serial.print(adc2);
    Serial.print(F(" v="));             Serial.print(v2,3);
    Serial.print(F(" Rs="));            Serial.print(rs2,0);
    Serial.print(F(" Rs/R0="));         Serial.print(ratio2,3);
    Serial.print(F(" mq2_ppm="));       Serial.print(ppm2,0);

    Serial.print(F(" || MQ4 adc="));    Serial.print(adc4);
    Serial.print(F(" v="));             Serial.print(v4,3);
    Serial.print(F(" Rs="));            Serial.print(rs4,0);
    Serial.print(F(" Rs/R0="));         Serial.print(ratio4,3);
    Serial.print(F(" mq4_ppm="));       Serial.print(ppm4,0);

    Serial.print(F(" || tempC="));      Serial.print(tC,1);
    Serial.print(F(" hum%="));          Serial.println(hum,1);

    // ------------------- CSV append -------------------
    if (sd_ok) {
      File f = openAppend(CSV_FILE);
      if (f) {
        f.print(now);            f.print(',');
        f.print(adc2);           f.print(',');
        f.print(v2,3);           f.print(',');
        f.print(rs2,0);          f.print(',');
        f.print(ratio2,4);       f.print(',');
        f.print(ppm2,0);         f.print(',');
        f.print(adc4);           f.print(',');
        f.print(v4,3);           f.print(',');
        f.print(rs4,0);          f.print(',');
        f.print(ratio4,4);       f.print(',');
        f.print(ppm4,0);         f.print(',');
        f.print(tC,2);           f.print(',');
        f.println(hum,1);
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
