#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <math.h>
#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

/* ======================= Configuration ======================= */
#define DHTPIN   7
#define DHTTYPE  DHT22
#define CS_PIN   10
#define MQ2_A_Pin A0
#define MQ4_A_Pin A1

#define SAMPLE_INTERVAL_MS  30000UL
#define WARMUP_MS           2000UL
#define DHT_RETRIES         4

// --- Sensor Constants ---
static const float VREF = 5.0f;
static const float VMIN = 0.01f;
static const float PPM_MAX_CLAMP = 65000.0f;

static const float MQ2_RL_OHMS = 12300.0f;
static const float MQ4_RL_OHMS = 23200.0f;

// Fixed R0 values
static const float MQ2_R0_OHMS = 10000.0f;
static const float MQ4_R0_OHMS = 3000.0f;

// Gas curve parameters
static const float MQ2_A = 50.0f;
static const float MQ2_B = -2.30f;
static const float MQ4_A = 1000.0f;
static const float MQ4_B = -2.95f;

// --- Time variables ---
time_t   startEpoch  = 0;
time_t   bootEpoch   = 0;
uint32_t bootMillis  = 0;

DHT dht(DHTPIN, DHTTYPE);
static bool sd_ok = false;
static String rxbuf;

/* ======================= NFC (PN532 over I2C) ======================= */
Adafruit_PN532 nfc(-1, -1);

static String g_latestNfcText = "";
static String g_nowDate = "";
static String g_nowTime = "";
static String g_startDate = "";
static String g_startTime = "";
static String g_startGas  = "";  // NEW: "S2=... S4=..."
static uint32_t g_lastWriteMs = 0;
static const uint32_t NFC_COOLDOWN_MS = 2000;
static bool nfc_available = false;

/* ======================= Utility functions ======================= */
static inline float adcToVolt(int adc) { return adc * (VREF / 1023.0f); }
static inline float mqRsFromVolt(float v, float RL) {
  if (v < VMIN) return 1e9f;
  return (VREF - v) * RL / v;
}
static inline float ppm_from_ratio(float r, float A, float B) {
  if (r <= 0.0f) return 0.0f;
  float ppm = A * pow(r, B);
  if (!(ppm >= 0.0f)) ppm = 0.0f;
  if (ppm > PPM_MAX_CLAMP) ppm = PPM_MAX_CLAMP;
  return ppm;
}
static uint16_t readADCavg(uint8_t pin, int samples = 8) {
  (void)analogRead(pin);
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) { acc += analogRead(pin); delay(2); }
  return (uint16_t)(acc / samples);
}

static void ensureCsvHeader() {
  if (!sd_ok) return;
  File f = SD.open(F("datalog.csv"), FILE_READ);
  bool needHeader = true;
  if (f) { if (f.size() > 0) needHeader = false; f.close(); }
  if (needHeader) {
    File w = SD.open(F("datalog.csv"), FILE_WRITE);
    if (w) {
      w.println(F("time_now,time_start,t_ms,mq2_adc,mq2_V,mq2_Rs,mq2_ratio,mq2_ppm,"
                  "mq4_adc,mq4_V,mq4_Rs,mq4_ratio,mq4_ppm,tempC,hum"));
      w.flush(); w.close();
      Serial.println(F("[CSV] header written"));
    } else {
      Serial.println(F("[CSV] failed to create file"));
      sd_ok = false;
    }
  }
}

void dhtRead(float &hum, float &tC) {
  hum = NAN; tC = NAN;
  for (int i=0; i<DHT_RETRIES; ++i) {
    hum = dht.readHumidity();
    tC  = dht.readTemperature();
    if (!isnan(hum) && !isnan(tC)) return;
    delay(200);
  }
  if (isnan(hum)) hum = -1.0f;
  if (isnan(tC))  tC  = -1000.0f;
}

String fmtEpoch(time_t t) {
  if (t == 0) return F("UNSET");
  char buf[20];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
  return String(buf);
}
String fmtDate(time_t t) {
  if (t == 0) return F("UNSET");
  char buf[16];
  sprintf(buf, "%04d-%02d-%02d", year(t), month(t), day(t));
  return String(buf);
}
String fmtTime(time_t t) {
  if (t == 0) return F("UNSET");
  char buf[16];
  sprintf(buf, "%02d:%02d:%02d", hour(t), minute(t), second(t));
  return String(buf);
}

/* ---------- Time Sync Helpers ---------- */
bool parseTimeString(const char* s, tmElements_t &tm) {
  // Expect "YYYY-MM-DD HH:MM:SS"
  int Y,M,D,h,m,sec;
  if (sscanf(s, "%d-%d-%d %d:%d:%d", &Y,&M,&D,&h,&m,&sec) != 6) return false;
  tm.Year   = CalendarYrToTm(Y);
  tm.Month  = M;
  tm.Day    = D;
  tm.Hour   = h;
  tm.Minute = m;
  tm.Second = sec;
  return true;
}

bool setNowFromSD() {
  if (!sd_ok) return false;
  File f = SD.open(F("TIME.TXT"), FILE_READ);
  if (!f) return false;
  String line = f.readStringUntil('\n');
  f.close();
  line.trim();
  tmElements_t tm;
  if (!parseTimeString(line.c_str(), tm)) {
    Serial.println(F("[TIME] TIME.TXT invalid format. Use: YYYY-MM-DD HH:MM:SS"));
    return false;
  }
  bootEpoch  = makeTime(tm);
  bootMillis = millis();
  Serial.print(F("[TIME] Set from SD: ")); Serial.println(fmtEpoch(bootEpoch));
  // optional: delete file after use so it’s one-shot
  SD.remove(F("TIME.TXT"));
  return true;
}

bool setNowFromSerialString(const String &s) {
  tmElements_t tm;
  if (!parseTimeString(s.c_str(), tm)) return false;
  bootEpoch  = makeTime(tm);
  bootMillis = millis();
  Serial.print(F("[TIME] Set from Serial: ")); Serial.println(fmtEpoch(bootEpoch));
  return true;
}

bool setNowFromCompile() {
  char monStr[4]; monStr[3] = 0;
  int d, y, H, M, S;
  if (sscanf(__DATE__, "%3s %d %d", monStr, &d, &y) != 3) return false;
  if (sscanf(__TIME__, "%d:%d:%d", &H, &M, &S) != 3) return false;
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* p = strstr(months, monStr);
  int m = p ? ((p - months) / 3 + 1) : 1;
  tmElements_t tm;
  tm.Year = CalendarYrToTm(y);
  tm.Month = m;
  tm.Day = d;
  tm.Hour = H;
  tm.Minute = M;
  tm.Second = S;
  bootEpoch  = makeTime(tm);
  bootMillis = millis();
  return true;
}

time_t nowTime() {
  uint32_t elapsed = (millis() - bootMillis) / 1000;
  return bootEpoch + elapsed;
}

/* ---------- CSV “first row” start gas fetch ---------- */
bool getCsvFirstPPMs(float &mq2_ppm, float &mq4_ppm) {
  mq2_ppm = NAN; mq4_ppm = NAN;
  if (!sd_ok) return false;
  File f = SD.open(F("datalog.csv"), FILE_READ);
  if (!f) return false;

  // skip header
  String line = f.readStringUntil('\n');
  // read first non-empty data row
  while (f.available()) {
    line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    // split by commas (we only need columns 7 and 12; 0-based)
    // indices: 0 time_now,1 time_start,2 t_ms,3 mq2_adc,4 mq2_V,5 mq2_Rs,6 mq2_ratio,7 mq2_ppm,8 mq4_adc,9 mq4_V,10 mq4_Rs,11 mq4_ratio,12 mq4_ppm,13 tempC,14 hum
    int idx = 0;
    int start = 0;
    String fields[15];
    for (int i=0; i<15; ++i) fields[i] = "";
    for (int i=0; i<=line.length(); ++i) {
      if (i == (int)line.length() || line[i] == ',') {
        if (idx < 15) fields[idx] = line.substring(start, i);
        idx++;
        start = i+1;
      }
    }
    if (idx >= 13) {
      mq2_ppm = fields[7].toFloat();
      mq4_ppm = fields[12].toFloat();
      f.close();
      return true;
    }
  }
  f.close();
  return false;
}

/* ======================= NFC helpers for MIFARE Classic ======================= */
bool mifare_write_multiblock(const String& line1, const String& line2, const String& line3,
                             const String& line4, const String& line5, const String& line6) {
  uint8_t uid[7];
  uint8_t uidLength = 0;

  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
    return false; // no tag
  }
  Serial.println(F("[NFC] Tag detected!"));

  if (uidLength == 4) {
    // MIFARE Classic 1K
    uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    // Prepare data blocks
    uint8_t data1[16], data2[16], data3[16], data4[16], data5[16], data6[16];
    memset(data1, 0x20, 16);
    memset(data2, 0x20, 16);
    memset(data3, 0x20, 16);
    memset(data4, 0x20, 16);
    memset(data5, 0x20, 16);
    memset(data6, 0x20, 16);

    for (int i=0;i<min((int)line1.length(),16);i++) data1[i]=(uint8_t)line1[i];
    for (int i=0;i<min((int)line2.length(),16);i++) data2[i]=(uint8_t)line2[i];
    for (int i=0;i<min((int)line3.length(),16);i++) data3[i]=(uint8_t)line3[i];
    for (int i=0;i<min((int)line4.length(),16);i++) data4[i]=(uint8_t)line4[i];
    for (int i=0;i<min((int)line5.length(),16);i++) data5[i]=(uint8_t)line5[i];
    for (int i=0;i<min((int)line6.length(),16);i++) data6[i]=(uint8_t)line6[i];

    // Sector 1 (blocks 4,5,6,7[trailer])
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya)) {
      Serial.println(F("[NFC] Auth failed S1!"));
      return false;
    }
    if (!nfc.mifareclassic_WriteDataBlock(4, data1)) { Serial.println(F("[NFC] Blk 4 fail!")); return false; }
    if (!nfc.mifareclassic_WriteDataBlock(5, data2)) { Serial.println(F("[NFC] Blk 5 fail!")); return false; }
    if (!nfc.mifareclassic_WriteDataBlock(6, data3)) { Serial.println(F("[NFC] Blk 6 fail!")); return false; }

    // Sector 2 (blocks 8,9,10,11[trailer])
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 8, 0, keya)) {
      Serial.println(F("[NFC] Auth failed S2!"));
      return false;
    }
    if (!nfc.mifareclassic_WriteDataBlock(8,  data4)) { Serial.println(F("[NFC] Blk 8 fail!"));  return false; }
    if (!nfc.mifareclassic_WriteDataBlock(9,  data5)) { Serial.println(F("[NFC] Blk 9 fail!"));  return false; }
    if (!nfc.mifareclassic_WriteDataBlock(10, data6)) { Serial.println(F("[NFC] Blk 10 fail!")); return false; }

    Serial.println(F("[NFC] OK!"));
    return true;
  } else if (uidLength == 7) {
    Serial.println(F("[NFC] NTAG - not supported"));
    return false;
  }
  return false;
}

/* ======================= Command handler ======================= */
void handleCommand(String line) {
  line.trim();
  String u = line; u.toUpperCase();

  if (u == "HELP") {
    Serial.println(F("Commands:\n  SET (save start=now)\n  TIME?\n  TIME=YYYY-MM-DD HH:MM:SS  (set wall clock)"));
    return;
  }

  if (u == "SET") {
    startEpoch = nowTime();
    Serial.print(F("[SET] New start time saved -> "));
    Serial.println(fmtEpoch(startEpoch));
    return;
  }

  if (u == "TIME?") {
    Serial.print(F("[TIME] Now="));   Serial.print(fmtEpoch(nowTime()));
    Serial.print(F(" | Start="));     Serial.println(fmtEpoch(startEpoch));
    return;
  }

  // TIME=YYYY-MM-DD HH:MM:SS
  if (u.startsWith("TIME=")) {
    String ts = line.substring(5); ts.trim();
    if (setNowFromSerialString(ts)) {
      Serial.println(F("[TIME] OK"));
    } else {
      Serial.println(F("[TIME] Bad format. Use TIME=YYYY-MM-DD HH:MM:SS"));
    }
    return;
  }
}

/* ======================= Setup ======================= */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("\n[BOOT] Multi-Sensor Logger + NFC"));
  Serial.println(F("[HINT] To set Now Time, send TIME=YYYY-MM-DD HH:MM:SS over Serial."));
  Serial.println(F("[HINT] To set Start TIME = Now Time, SET over Serial."));

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  analogReference(DEFAULT);

  // DHT init
  Serial.println(F("[DHT] Init..."));
  dht.begin();
  delay(1000);
  (void)dht.readHumidity(); (void)dht.readTemperature();
  delay(500);

  // SD init
  Serial.println(F("[SD] Init..."));
  if (SD.begin(CS_PIN)) {
    sd_ok = true;
    Serial.println(F("[SD] OK"));
    ensureCsvHeader();
  } else {
    sd_ok = false;
    Serial.println(F("[SD] Failed - no SD logging"));
  }

  // Time setup: prefer SD TIME.TXT, else wait briefly for Serial TIME=..., else fallback to compile time
  bool timeSet = false;
  if (setNowFromSD()) timeSet = true;

  unsigned long waitUntil = millis() + 2000; // small window to allow TIME=... immediately after boot
  while (!timeSet && millis() < waitUntil) {
    if (Serial.available()) {
      char ch = (char)Serial.read();
      if (ch == '\n' || ch == '\r') { if (rxbuf.length()) { handleCommand(rxbuf); rxbuf = ""; } }
      else { if (rxbuf.length() < 96) rxbuf += ch; }
    }
    delay(5);
  }
  if (!timeSet && bootEpoch == 0) {
    setNowFromCompile();
    Serial.print(F("[TIME] Start time: "));
    Serial.println(fmtEpoch(bootEpoch));
  }
  startEpoch = nowTime(); // default start at boot; user can SET later

  // MQ warmup
  Serial.println(F("[WARMUP] Stabilizing..."));
  unsigned long t0 = millis();
  while ((millis() - t0) <= WARMUP_MS) { delay(250); }

  // NFC init
  Serial.println(F("[NFC] Init..."));
  Wire.begin();
  nfc.begin();
  uint32_t ver = nfc.getFirmwareVersion();
  if (!ver) {
    Serial.println(F("[NFC] Not found - disabled"));
    nfc_available = false;
  } else {
    Serial.print(F("[NFC] v"));
    Serial.println((ver >> 16) & 0xFF, HEX);
    nfc.SAMConfig();
    nfc_available = true;
  }

  Serial.println(F("\n[READY] Type HELP, SET or TIME?"));
}

/* ======================= Loop ======================= */
void loop() {
  // Command shell
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rxbuf.length()) { handleCommand(rxbuf); rxbuf = ""; }
    } else {
      if (rxbuf.length() < 96) rxbuf += ch;
    }
  }

  // Periodic sample
  static uint32_t lastSample = 0;
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - lastSample) >= SAMPLE_INTERVAL_MS) {
    lastSample = nowMs;

    // Read MQ2
    uint16_t adc2 = readADCavg(MQ2_A_Pin);
    float v2 = adcToVolt(adc2);
    float rs2 = mqRsFromVolt(v2, MQ2_RL_OHMS);
    float ratio2 = rs2 / MQ2_R0_OHMS;
    float ppm2 = ppm_from_ratio(ratio2, MQ2_A, MQ2_B);

    // Read MQ4
    uint16_t adc4 = readADCavg(MQ4_A_Pin);
    float v4 = adcToVolt(adc4);
    float rs4 = mqRsFromVolt(v4, MQ4_RL_OHMS);
    float ratio4 = rs4 / MQ4_R0_OHMS;
    float ppm4 = ppm_from_ratio(ratio4, MQ4_A, MQ4_B);

    // Read DHT
    float hum, tC;
    dhtRead(hum, tC);

    // SD logging
    const char* sdStatus = "NOT_MOUNTED";
    if (sd_ok) {
      File f = SD.open(F("datalog.csv"), FILE_WRITE);
      if (f) {
        f.seek(f.size());
        f.print(fmtEpoch(nowTime()));    f.print(',');
        f.print(fmtEpoch(startEpoch));   f.print(',');
        f.print(nowMs);                  f.print(',');
        f.print(adc2);                   f.print(',');
        f.print(v2,3);                   f.print(',');
        f.print(rs2,0);                  f.print(',');
        f.print(ratio2,4);               f.print(',');
        f.print(ppm2,0);                 f.print(',');
        f.print(adc4);                   f.print(',');
        f.print(v4,3);                   f.print(',');
        f.print(rs4,0);                  f.print(',');
        f.print(ratio4,4);               f.print(',');
        f.print(ppm4,0);                 f.print(',');
        f.print(tC,2);                   f.print(',');
        f.println(hum,1);
        f.flush(); f.close();
        sdStatus = "APPENDED";
      } else {
        sdStatus = "FAIL_OPEN";
        sd_ok = false;
      }
    }

    // Build NFC text (all strings <=16 chars)
    time_t now = nowTime();
    g_nowDate   = "ND:" + fmtDate(now);             // e.g. "ND:2025-10-25"
    g_nowTime   = "NT:" + fmtTime(now);             // e.g. "NT:17:07:13"
    g_startDate = "SD:" + fmtDate(startEpoch);      // e.g. "SD:2025-10-25"
    g_startTime = "ST:" + fmtTime(startEpoch);      // e.g. "ST:17:06:14"
    g_latestNfcText = "M2=" + String((int)ppm2) + " M4=" + String((int)ppm4);  // latest

    // Start gas values from first CSV row (or fallback to current)
    static bool startGasKnown = false;
    static int  startM2 = 0, startM4 = 0;
    if (!startGasKnown) {
      float s2, s4;
      if (getCsvFirstPPMs(s2, s4)) {
        startM2 = (int)round(s2); startM4 = (int)round(s4);
        startGasKnown = true;
        Serial.print(F("[START-GAS] From CSV first row S2=")); Serial.print(startM2);
        Serial.print(F(" S4=")); Serial.println(startM4);
      } else {
        startM2 = (int)ppm2; startM4 = (int)ppm4; // fallback until CSV exists
        // do not mark known; we’ll try again next sample to catch CSV after first write
      }
    }
    g_startGas = "S2=" + String(startM2) + " S4=" + String(startM4);

    // Serial output
    Serial.print(F("[SAMPLE] now="));   Serial.print(fmtEpoch(nowTime()));
    Serial.print(F(" | start="));       Serial.print(fmtEpoch(startEpoch));
    Serial.print(F(" | MQ2="));         Serial.print((int)ppm2);
    Serial.print(F(" | MQ4="));         Serial.print((int)ppm4);
    Serial.print(F(" | temp="));        Serial.print(tC,1);
    Serial.print(F(" | hum="));         Serial.println(hum,1);
    Serial.print(F("[SD] ")); Serial.println(sdStatus);
    Serial.print(F("[NFC] Data ready: ")); Serial.print(g_latestNfcText);
    Serial.print(F(" | StartGas: ")); Serial.println(g_startGas);
  }

  // NFC: write when tag is presented (cached data from last sample)
  if (nfc_available && g_latestNfcText.length() > 0) {
    if (millis() - g_lastWriteMs > NFC_COOLDOWN_MS) {
      if (mifare_write_multiblock(
            g_nowDate,   // block 4
            g_nowTime,   // block 5
            g_startDate, // block 6
            g_startTime, // block 8
            g_latestNfcText, // block 9
            g_startGas       // block 10 (NEW)
          )) {
        g_lastWriteMs = millis();
        delay(300);
      }
    }
  }
}
