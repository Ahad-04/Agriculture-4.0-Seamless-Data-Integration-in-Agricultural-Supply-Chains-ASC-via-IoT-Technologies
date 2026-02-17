// ===================== ESP32 + MQ2(GPIO34) + MQ4(GPIO35) + DHT22(GPIO4) + microSD(CS=5) =====================
// Logs once per SAMPLE_INTERVAL_MS to Serial and /datalog.csv, with warm-up, RO auto-cal, and "CAL" command.
//
// CSV columns:
// elapsed_hms,tempC,hum,
// mq2_adc,mq2_Vout,mq2_RS_ohm,mq2_RS_over_RO,mq2_GasIndex,
// mq4_adc,mq4_Vout,mq4_RS_ohm,mq4_RS_over_RO,mq4_GasIndex
//
// IMPORTANT (ESP32 specifics):
// - ADC is 12-bit (0..4095).
// - ADC full-scale is ~3.3V (we set 11dB attenuation for ~0..3.6V).
// - If your MQ modules are powered at 5V, you MUST scale their analog output down to <= 3.3V before ESP32.
//   Use a simple divider (e.g., 10k:10k). Then set INPUT_DIV_GAIN to (Rtop+Rbottom)/Rbottom (e.g., 2.0).
//
// FIRST BOOT: warms up 10 min in clean air, then calibrates RO for 60s (averages RS) and saves in EEPROM emulation.
// ANYTIME: type "CAL" (newline) in Serial to recalibrate in clean air (after warm-up).
//
// GAS INDEX (same as Mega code): RS/RO → 0..100 scale (human-friendly)
//   1.0→0, 0.6→40, 0.3→80, 0.1→100, clamped [0,100]

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <EEPROM.h>    // ESP32 uses flash-backed EEPROM emulation: call EEPROM.begin(size)

// ------------------- Pins (edit as needed) -------------------
#define PIN_DHT        4
#define DHTTYPE        DHT22

// Use ADC1 channels on ESP32 (ADC2 is blocked by WiFi)
#define PIN_MQ2_A      34   // ADC1_CH6 (input-only)
#define PIN_MQ4_A      35   // ADC1_CH7 (input-only)

// SPI microSD (VSPI default pins: SCK=18, MISO=19, MOSI=23)
#define PIN_SD_CS       5   // choose any free GPIO, 5 is VSPI SS by default

// ------------------- Timing -------------------
#define SAMPLE_INTERVAL_MS   30000UL           // 1 s (change if you like)
#define WARMUP_MS            (10UL*60UL*1000UL)
#define CAL_SECONDS          60

// ------------------- Electrical / ADC Model -------------------
// ADC full-scale voltage on ESP32 (approx; with 11dB attenuation we treat it as ~3.3V)
static const float ADC_FS_VOLT      = 3.3f;

// Your MQ module supply voltage (commonly 5.0 V). Used in RS calculation: Rs = RL * (Vcc/Vout - 1)
static const float MQ_SUPPLY_V      = 5.0f;

// Divider gain to reconstruct the module's actual Vout from the ADC pin voltage.
// If you use 10k:10k (Vadc = Vout/2), set this to 2.0. If no divider and Vout<=3.3V, set to 1.0.
static const float INPUT_DIV_GAIN   = 2.0f;

// ADC resolution
static const float ADC_COUNTS       = 4095.0f; // 12-bit

// Load resistors on your MQ modules (check your board)
static const float RL_MQ2_OHMS      = 5000.0f;
static const float RL_MQ4_OHMS      = 5000.0f;

// ------------------- EEPROM layout for RO -------------------
// On ESP32 you MUST call EEPROM.begin(N) once in setup; we use 64 bytes.
#define EE_MAGIC_ADDR        0
#define EE_MAGIC_VALUE       0x45A1B2C3UL
#define EE_RO_MQ2_ADDR       (EE_MAGIC_ADDR + sizeof(uint32_t))       // 4
#define EE_RO_MQ4_ADDR       (EE_RO_MQ2_ADDR + sizeof(float))         // 8
#define EEPROM_BYTES         64

// ------------------- Globals -------------------
DHT dht(PIN_DHT, DHTTYPE);
float g_ro_mq2 = NAN;
float g_ro_mq4 = NAN;

const char* CSV_FILE = "/datalog.csv";

// Optional cached DHT (helps if your DHT is flaky)
float lastTemp = NAN, lastHum = NAN;
uint32_t lastDhtRead = 0;
const uint32_t DHT_PERIOD_MS = 2000; // read DHT every 2s

// ------------------- ADC setup helpers (ESP32) -------------------
#include "driver/adc.h"
static void setupADC() {
  // Use 12-bit resolution and 11dB attenuation (~0..3.6V) on the pins we sample
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_MQ2_A, ADC_11db);
  analogSetPinAttenuation(PIN_MQ4_A, ADC_11db);
  // NOTE: ESP32 ADCs are not perfectly linear; for better accuracy consider calibration w/ known voltages.
}

// ------------------- Helpers -------------------
static inline float adcCountsToVadc(uint16_t adc) {
  return (ADC_FS_VOLT * (float)adc) / ADC_COUNTS; // voltage AT THE ESP32 PIN
}

// Returns the module’s actual analog output Vout after undoing your input divider
static inline float adcToModuleVout(uint16_t adc) {
  float vadc = adcCountsToVadc(adc);
  return vadc * INPUT_DIV_GAIN; // reconstruct the original module pin voltage
}

// RS = RL * (Vcc/Vout - 1)  [Vcc = MQ_SUPPLY_V, Vout = module’s analog pin voltage]
static inline float voutToRS(float vout, float RL_ohms) {
  if (vout < 0.001f) vout = 0.001f; // avoid div by zero
  return RL_ohms * (MQ_SUPPLY_V / vout - 1.0f);
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
    float t = (1.0f - r) / (1.0f - 0.6f);  // 1.0→0, 0.6→40
    return 0.0f + t * 40.0f;
  } else if (r > 0.3f) {
    float t = (0.6f - r) / (0.6f - 0.3f);  // 0.6→40, 0.3→80
    return 40.0f + t * 40.0f;
  } else {
    float t = (0.3f - r) / (0.3f - 0.1f);  // 0.3→80, 0.1→100
    return 80.0f + t * 20.0f;
  }
}

// ------------------- EEPROM RO (ESP32) -------------------
static void saveROtoEEPROM(float ro2, float ro4) {
  uint32_t magic = EE_MAGIC_VALUE;
  EEPROM.put(EE_MAGIC_ADDR, magic);
  EEPROM.put(EE_RO_MQ2_ADDR, ro2);
  EEPROM.put(EE_RO_MQ4_ADDR, ro4);
  EEPROM.commit();  // REQUIRED on ESP32
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
  if (!SD.begin(PIN_SD_CS, SPI, 25000000)) { // 25 MHz is typical; lower if flaky
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
                "mq2_adc,mq2_Vout,mq2_RS_ohm,mq2_RS_over_RO,mq2_GasIndex,"
                "mq4_adc,mq4_Vout,mq4_RS_ohm,mq4_RS_over_RO,mq4_GasIndex"));
    f.close();
  }
  return true;
}

static void appendCSV(uint32_t t_ms,
                      float tempC, float hum,
                      uint16_t adc2, float vout2, float rs2, float r2_over_ro, float gi2,
                      uint16_t adc4, float vout4, float rs4, float r4_over_ro, float gi4) {
  File f = SD.open(CSV_FILE, FILE_WRITE);
  if (!f) { Serial.println(F("[SD] open FAIL")); return; }

  char hms[16];
  msToHMS(t_ms, hms, sizeof(hms));

  f.print(hms);               f.print(',');
  if (isnan(tempC)) f.print("NA"); else f.print(tempC, 1); f.print(',');
  if (isnan(hum))   f.print("NA"); else f.print(hum, 0);   f.print(',');

  f.print(adc2);   f.print(',');
  f.print(vout2, 3);  f.print(',');
  f.print(rs2, 1); f.print(',');
  if (!isfinite(r2_over_ro)) f.print("NA"); else f.print(r2_over_ro, 3); f.print(',');
  if (!isfinite(gi2)) f.print("NA"); else f.print(gi2, 0); f.print(',');

  f.print(adc4);   f.print(',');
  f.print(vout4, 3);  f.print(',');
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

    float vout2 = adcToModuleVout(adc2);
    float vout4 = adcToModuleVout(adc4);

    float rs2 = voutToRS(vout2, RL_MQ2_OHMS);
    float rs4 = voutToRS(vout4, RL_MQ4_OHMS);

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
  Serial.println(F("[CAL] Saved. Calibration done.\n"));
}

// ------------------- Setup & Loop -------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Prepare EEPROM emulation
  if (!EEPROM.begin(EEPROM_BYTES)) {
    Serial.println(F("[BOOT] EEPROM.begin FAILED"));
  }

  // ADC config
  setupADC();

  // SD
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
      if ((millis() - t0) % 10000UL < 50UL) { // ~every 10s
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
        while (millis() - t0 < 2UL*60UL*1000UL) { delay(10); }
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

  // reconstruct module Vout from ADC reading (undo divider)
  float vout2 = adcToModuleVout(adc2);
  float vout4 = adcToModuleVout(adc4);

  float rs2 = voutToRS(vout2, RL_MQ2_OHMS);
  float rs4 = voutToRS(vout4, RL_MQ4_OHMS);

  float r2_over_ro = (isfinite(g_ro_mq2) && g_ro_mq2 > 0) ? (rs2 / g_ro_mq2) : NAN;
  float r4_over_ro = (isfinite(g_ro_mq4) && g_ro_mq4 > 0) ? (rs4 / g_ro_mq4) : NAN;

  float gi2 = gasIndexFromRsRo(r2_over_ro);
  float gi4 = gasIndexFromRsRo(r4_over_ro);

  // --- Serial status line ---
  Serial.print(F("[T+ ")); Serial.print(now / 1000.0f, 3); Serial.print(F(" s] "));
  Serial.print(F("TempC=")); if (isnan(temp)) Serial.print("NA"); else Serial.print(temp,1);
  Serial.print(F(" Hum%=")); if (isnan(hum))  Serial.print("NA"); else Serial.print(hum,0);

  Serial.print(F(" | MQ2: adc=")); Serial.print(adc2);
  Serial.print(F(" Vout="));      Serial.print(vout2,3);
  Serial.print(F(" RS="));        Serial.print(rs2,1);
  Serial.print(F(" RS/RO="));     if (!isfinite(r2_over_ro)) Serial.print("NA"); else Serial.print(r2_over_ro,3);
  Serial.print(F(" GI="));        if (!isfinite(gi2)) Serial.print("NA"); else Serial.print((int)gi2);

  Serial.print(F(" | MQ4: adc=")); Serial.print(adc4);
  Serial.print(F(" Vout="));      Serial.print(vout4,3);
  Serial.print(F(" RS="));        Serial.print(rs4,1);
  Serial.print(F(" RS/RO="));     if (!isfinite(r4_over_ro)) Serial.print("NA"); else Serial.print(r4_over_ro,3);
  Serial.print(F(" GI="));        if (!isfinite(gi4)) Serial.print("NA"); else Serial.print((int)gi4);
  Serial.println();

  // --- CSV append ---
  if (SD.begin(PIN_SD_CS, SPI, 25000000)) {
    appendCSV(now, temp, hum, adc2, vout2, rs2, r2_over_ro, gi2,
                        adc4, vout4, rs4, r4_over_ro, gi4);
    Serial.println(F("[CSV] appended"));
  } else {
    Serial.println(F("[SD] init FAIL (this interval not logged)"));
  }
}
