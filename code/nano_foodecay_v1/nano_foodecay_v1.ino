// ===================== Arduino Nano + MQ2(A0) + MQ4(A1) + DHT22(D7) + microSD(CS=D10) =====================
// Logs once per SAMPLE_INTERVAL_MS to Serial and /datalog.csv:
// elapsed_hms,tempC,hum,
// mq2_adc,mq2_V,mq2_RS_ohm,mq2_RS_over_RO,mq2_GasIndex,
// mq4_adc,mq4_V,mq4_RS_ohm,mq4_RS_over_RO,mq4_GasIndex
//
// FIRST BOOT: warms up 10 minutes in clean air, then calibrates RO for 60s (averages RS) and saves in EEPROM.
// ANYTIME: type "CAL" (newline) in Serial to recalibrate in clean air (after warm-up).
//
// GAS INDEX: RS/RO → 0..100 scale (human-friendly)
//   1.0→0, 0.6→40, 0.3→80, 0.1→100, clamped [0,100]

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <EEPROM.h>

// ------------------- Pins -------------------
#define PIN_DHT        7
#define DHTTYPE        DHT22   // white sensor → DHT22

#define PIN_MQ2_A      A0
#define PIN_MQ4_A      A1

// microSD on Nano (ATmega328P) HW SPI: SCK=13, MOSI=11, MISO=12
#define PIN_SD_CS      10

// ------------------- Timing -------------------
// Choose one interval:
#define SAMPLE_INTERVAL_MS   1000UL            // log every 1 s
//#define SAMPLE_INTERVAL_MS   60000UL          // log every 1 min

#define WARMUP_MS            (10UL*60UL*1000UL) // 10 minutes warm-up for MQ sensors
#define CAL_SECONDS          60                 // average RS for 60s to get RO

// ------------------- ADC & Electrical -------------------
#define VREF_VOLT            5.0f     // Nano analog reference (AVcc ≈ 5.0 V)
#define ADC_COUNTS           1023.0f  // 10-bit ADC
#define RL_MQ2_OHMS          13710.0f  // adjust if your module uses another RL
#define RL_MQ4_OHMS          998.0f

// ------------------- EEPROM layout for RO -------------------
#define EE_MAGIC_ADDR        0
#define EE_MAGIC_VALUE       0x45A1B2C3UL
#define EE_RO_MQ2_ADDR       (EE_MAGIC_ADDR + sizeof(uint32_t))       // 4
#define EE_RO_MQ4_ADDR       (EE_RO_MQ2_ADDR + sizeof(float))         // 8

// ------------------- Globals -------------------
DHT dht(PIN_DHT, DHTTYPE);
float g_ro_mq2 = NAN;
float g_ro_mq4 = NAN;

const char* CSV_FILE = "datalog.csv";

// Optional cached DHT (helps if your DHT is flaky)
float lastTemp = NAN, lastHum = NAN;
uint32_t lastDhtRead = 0;
const uint32_t DHT_PERIOD_MS = 2000; // read DHT every 2s

// ------------------- Helpers -------------------
static inline float adcToVolt(uint16_t adc) {
  return (VREF_VOLT * (float)adc) / ADC_COUNTS;
}

// RS = RL * (Vref/V - 1)   (here Vref is the sensor/ADC 5V)
static inline float adcToRS(uint16_t adc, float RL_ohms) {
  float v = adcToVolt(adc);
  if (v < 0.001f) v = 0.001f; // avoid div by zero
  return RL_ohms * (VREF_VOLT / v - 1.0f);
}

static uint16_t readADCavg(uint8_t pin, int samples = 8) {
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) {
    acc += analogRead(pin);
    delay(2);
  }
  return (uint16_t)(acc / samples);
}

// elapsed "HH:MM:SS" from millis()
static void msToHMS(uint32_t ms, char* out, size_t outlen) {
  uint32_t s = ms / 1000UL;
  uint32_t h = s / 3600UL;
  uint8_t  m = (s % 3600UL) / 60UL;
  uint8_t  sec = s % 60UL;
  snprintf(out, outlen, "%lu:%02u:%02u", (unsigned long)h, m, sec);
}

// Piecewise-linear GasIndex mapping from RS/RO → 0..100
static float gasIndexFromRsRo(float r) {
  if (!isfinite(r) || r <= 0) return NAN;
  if (r >= 1.0f) return 0.0f;
  if (r <= 0.1f) return 100.0f;

  if (r > 0.6f) {
    // map 1.0→0, 0.6→40
    float t = (1.0f - r) / (1.0f - 0.6f);
    return 0.0f + t * 40.0f;
  } else if (r > 0.3f) {
    // map 0.6→40, 0.3→80
    float t = (0.6f - r) / (0.6f - 0.3f);
    return 40.0f + t * 40.0f;
  } else {
    // map 0.3→80, 0.1→100
    float t = (0.3f - r) / (0.3f - 0.1f);
    return 80.0f + t * 20.0f;
  }
}

// ------------------- EEPROM RO -------------------
static void saveROtoEEPROM(float ro2, float ro4) {
  uint32_t magic = EE_MAGIC_VALUE;
  EEPROM.put(EE_MAGIC_ADDR, magic);
  EEPROM.put(EE_RO_MQ2_ADDR, ro2);
  EEPROM.put(EE_RO_MQ4_ADDR, ro4);
}

static bool loadROfromEEPROM(float &ro2, float &ro4) {
  uint32_t magic = 0;
  EEPROM.get(EE_MAGIC_ADDR, magic);
  if (magic != EE_MAGIC_VALUE) return false;
  EEPROM.get(EE_RO_MQ2_ADDR, ro2);
  EEPROM.get(EE_RO_MQ4_ADDR, ro4);
  if (!isfinite(ro2) || ro2 <= 0 || !isfinite(ro4) || ro4 <= 0) return false;
  return true;
}

// ------------------- CSV -------------------
static bool ensureCSV() {
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println(F("[SD] init FAIL"));
    return false;
  }
  if (!SD.exists(CSV_FILE)) {
    File f = SD.open(CSV_FILE, FILE_WRITE);
    if (!f) {
      Serial.println(F("[SD] create FAIL"));
      return false;
    }
    f.println(F("elapsed_hms,tempC,hum,"
                "mq2_adc,mq2_V,mq2_RS_ohm,mq2_RS_over_RO,mq2_GasIndex,"
                "mq4_adc,mq4_V,mq4_RS_ohm,mq4_RS_over_RO,mq4_GasIndex"));
    f.close();
  }
  return true;
}

static void appendCSV(uint32_t t_ms,
                      float tempC, float hum,
                      uint16_t adc2, float v2, float rs2, float r2_over_ro, float gi2,
                      uint16_t adc4, float v4, float rs4, float r4_over_ro, float gi4) {
  File f = SD.open(CSV_FILE, FILE_WRITE);
  if (!f) { Serial.println(F("[SD] open FAIL")); return; }

  char hms[16];
  msToHMS(t_ms, hms, sizeof(hms));

  f.print(hms);               f.print(',');
  if (isnan(tempC)) f.print("NA"); else f.print(tempC, 1); f.print(',');
  if (isnan(hum))   f.print("NA"); else f.print(hum, 0);   f.print(',');

  f.print(adc2);   f.print(',');
  f.print(v2, 3);  f.print(',');
  f.print(rs2, 1); f.print(',');
  if (!isfinite(r2_over_ro)) f.print("NA"); else f.print(r2_over_ro, 3); f.print(',');
  if (!isfinite(gi2)) f.print("NA"); else f.print(gi2, 0); f.print(',');

  f.print(adc4);   f.print(',');
  f.print(v4, 3);  f.print(',');
  f.print(rs4, 1); f.print(',');
  if (!isfinite(r4_over_ro)) f.print("NA"); else f.print(r4_over_ro, 3); f.print(',');
  if (!isfinite(gi4)) f.print("NA"); else f.print(gi4, 0);

  f.println();
  f.close();
}

// ------------------- Calibration -------------------
static void calibrateRO_cleanAir(uint16_t seconds) {
  Serial.println(F("\n[CAL] Calibrating RO in CLEAN AIR... keep sensors in clean air (box open)."));
  const unsigned long t_start = millis();
  const unsigned long t_end   = t_start + (unsigned long)seconds * 1000UL;

  double acc_rs2 = 0.0, acc_rs4 = 0.0;
  uint32_t n = 0;
  uint32_t lastPrint = 0;

  while (millis() < t_end) {
    uint16_t adc2 = readADCavg(PIN_MQ2_A);
    uint16_t adc4 = readADCavg(PIN_MQ4_A);

    float rs2 = adcToRS(adc2, RL_MQ2_OHMS);
    float rs4 = adcToRS(adc4, RL_MQ4_OHMS);

    acc_rs2 += (double)rs2;
    acc_rs4 += (double)rs4;
    n++;

    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      uint16_t elapsed = (uint16_t)((millis() - t_start) / 1000UL);
      Serial.print(F("[CAL] ")); Serial.print(elapsed);
      Serial.print(F("/")); Serial.print(seconds); Serial.println(F(" s"));
    }
    delay(10);
  }

  if (n == 0) n = 1;
  g_ro_mq2 = (float)(acc_rs2 / (double)n);
  g_ro_mq4 = (float)(acc_rs4 / (double)n);

  saveROtoEEPROM(g_ro_mq2, g_ro_mq4);

  Serial.print(F("[CAL] RO_MQ2 = ")); Serial.print(g_ro_mq2, 1); Serial.println(F(" ohms"));
  Serial.print(F("[CAL] RO_MQ4 = ")); Serial.print(g_ro_mq4, 1); Serial.println(F(" ohms"));
  Serial.println(F("[CAL] Saved to EEPROM. Calibration done.\n"));
}

// ------------------- Setup & Loop -------------------
void setup() {
  Serial.begin(115200);
  // Do NOT wait for Serial on Nano (no native USB), it can hang:
  // while (!Serial) { ; }

  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);

  dht.begin();

  if (!ensureCSV()) {
    Serial.println(F("[BOOT] SD not ready. Logging to Serial only."));
  } else {
    Serial.println(F("[BOOT] SD ready."));
  }

  // Try load RO from EEPROM; if absent → warm-up 10 min then 60s calibration
  if (!loadROfromEEPROM(g_ro_mq2, g_ro_mq4)) {
    Serial.println(F("[BOOT] No RO in EEPROM → Warm-up 10 min in CLEAN AIR, then auto-calibrate 60s."));
    uint32_t t0 = millis();
    while (millis() - t0 < WARMUP_MS) {
      uint32_t rem = (WARMUP_MS - (millis() - t0)) / 1000UL;
      if ((millis() - t0) % 10000UL < 50UL) { // about every 10s
        Serial.print(F("[WARMUP] ")); Serial.print(rem); Serial.println(F(" s remaining"));
      }
      delay(50);
    }
    Serial.println(F("[WARMUP] Done. Starting calibration..."));
    calibrateRO_cleanAir(CAL_SECONDS);
  } else {
    Serial.print(F("[BOOT] Loaded RO_MQ2=")); Serial.print(g_ro_mq2, 1);
    Serial.print(F(" ohms, RO_MQ4=")); Serial.print(g_ro_mq4, 1); Serial.println(F(" ohms"));
    Serial.println(F("[BOOT] Type 'CAL' (newline) in Serial to recalibrate in clean air (after warm-up)."));
  }
}

void loop() {
  // Listen for "CAL" to recalibrate in clean air
  static String cmd;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmd.equalsIgnoreCase("CAL")) {
        Serial.println(F("[CMD] CAL received. Please ensure CLEAN AIR; warming 2 minutes..."));
        uint32_t t0 = millis();
        while (millis() - t0 < 2UL*60UL*1000UL) { delay(10); } // quick warm-up before averaging
        calibrateRO_cleanAir(CAL_SECONDS);
      }
      cmd = "";
    } else {
      cmd += c;
      if (cmd.length() > 32) cmd.remove(0, cmd.length() - 32);
    }
  }

  // enforce the SAMPLE_INTERVAL_MS for all logging
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < SAMPLE_INTERVAL_MS) return;
  last = now;

  // --- DHT22 (read every 2s; keep last good) ---
  if (now - lastDhtRead >= DHT_PERIOD_MS) {
    lastDhtRead = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature(); // °C
    if (!isnan(h) && !isnan(t)) { lastHum = h; lastTemp = t; }
  }
  float hum  = lastHum;
  float temp = lastTemp;

  // --- MQ2/MQ4 raw ---
  uint16_t adc2 = readADCavg(PIN_MQ2_A);
  uint16_t adc4 = readADCavg(PIN_MQ4_A);

  float v2 = adcToVolt(adc2);
  float v4 = adcToVolt(adc4);

  float rs2 = adcToRS(adc2, RL_MQ2_OHMS);
  float rs4 = adcToRS(adc4, RL_MQ4_OHMS);

  float r2_over_ro = (isfinite(g_ro_mq2) && g_ro_mq2 > 0) ? (rs2 / g_ro_mq2) : NAN;
  float r4_over_ro = (isfinite(g_ro_mq4) && g_ro_mq4 > 0) ? (rs4 / g_ro_mq4) : NAN;

  float gi2 = gasIndexFromRsRo(r2_over_ro);
  float gi4 = gasIndexFromRsRo(r4_over_ro);

  // --- Serial status line (human-readable) ---
  Serial.print(F("[T+ ")); Serial.print(now / 1000.0f, 3); Serial.print(F(" s] "));
  Serial.print(F("TempC=")); if (isnan(temp)) Serial.print("NA"); else Serial.print(temp,1);
  Serial.print(F(" Hum%=")); if (isnan(hum))  Serial.print("NA"); else Serial.print(hum,0);

  Serial.print(F(" | MQ2: adc=")); Serial.print(adc2);
  Serial.print(F(" V="));         Serial.print(v2,3);
  Serial.print(F(" RS="));        Serial.print(rs2,1);
  Serial.print(F(" RS/RO="));     if (!isfinite(r2_over_ro)) Serial.print("NA"); else Serial.print(r2_over_ro,3);
  Serial.print(F(" GI="));        if (!isfinite(gi2)) Serial.print("NA"); else Serial.print((int)gi2);

  Serial.print(F(" | MQ4: adc=")); Serial.print(adc4);
  Serial.print(F(" V="));         Serial.print(v4,3);
  Serial.print(F(" RS="));        Serial.print(rs4,1);
  Serial.print(F(" RS/RO="));     if (!isfinite(r4_over_ro)) Serial.print("NA"); else Serial.print(r4_over_ro,3);
  Serial.print(F(" GI="));        if (!isfinite(gi4)) Serial.print("NA"); else Serial.print((int)gi4);
  Serial.println();

  // --- CSV append (exactly once per SAMPLE_INTERVAL_MS) ---
  if (SD.begin(PIN_SD_CS)) {
    appendCSV(now, temp, hum, adc2, v2, rs2, r2_over_ro, gi2,
                      adc4, v4, rs4, r4_over_ro, gi4);
    Serial.println(F("[CSV] appended"));
  } else {
    Serial.println(F("[SD] init FAIL (this interval not logged)"));
  }
}
