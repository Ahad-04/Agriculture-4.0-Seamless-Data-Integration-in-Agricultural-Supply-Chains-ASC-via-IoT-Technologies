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
  if (f) { 
    if (f.size() > 0) needHeader = false; 
    f.close(); 
  }
  
  if (needHeader) {
    File w = SD.open(F("datalog.csv"), FILE_WRITE);
    if (w) {
      w.println(F("time_now,time_start,t_ms,mq2_adc,mq2_V,mq2_Rs,mq2_ratio,mq2_ppm,"
                  "mq4_adc,mq4_V,mq4_Rs,mq4_ratio,mq4_ppm,tempC,hum"));
      w.flush();
      w.close();
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

// Separate date and time strings for NFC
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

/* ======================= NFC helpers for MIFARE Classic ======================= */
bool mifare_write_multiblock(const String& line1, const String& line2, const String& line3, const String& line4, const String& line5) {
  uint8_t uid[7];
  uint8_t uidLength = 0;

  // Try to read the card
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
    return false; // no tag present
  }

  Serial.println(F("[NFC] Tag detected!"));

  if (uidLength == 4) {
    // MIFARE Classic 1K
    Serial.println(F("[NFC] Writing blocks..."));
    
    uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    
    // Authenticate block 4 (this authenticates entire sector 1: blocks 4,5,6,7)
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya)) {
      Serial.println(F("[NFC] Auth failed S1!"));
      return false;
    }

    // Prepare 5 blocks (16 bytes each)
    uint8_t data1[16], data2[16], data3[16], data4[16], data5[16];
    memset(data1, 0x20, 16); // fill with spaces
    memset(data2, 0x20, 16);
    memset(data3, 0x20, 16);
    memset(data4, 0x20, 16);
    memset(data5, 0x20, 16);
    
    // Copy strings to blocks
    for (int i = 0; i < min((int)line1.length(), 16); i++) data1[i] = (uint8_t)line1[i];
    for (int i = 0; i < min((int)line2.length(), 16); i++) data2[i] = (uint8_t)line2[i];
    for (int i = 0; i < min((int)line3.length(), 16); i++) data3[i] = (uint8_t)line3[i];
    for (int i = 0; i < min((int)line4.length(), 16); i++) data4[i] = (uint8_t)line4[i];
    for (int i = 0; i < min((int)line5.length(), 16); i++) data5[i] = (uint8_t)line5[i];

    // Write to blocks 4, 5, 6 (sector 1)
    if (!nfc.mifareclassic_WriteDataBlock(4, data1)) {
      Serial.println(F("[NFC] Blk 4 fail!"));
      return false;
    }
    if (!nfc.mifareclassic_WriteDataBlock(5, data2)) {
      Serial.println(F("[NFC] Blk 5 fail!"));
      return false;
    }
    if (!nfc.mifareclassic_WriteDataBlock(6, data3)) {
      Serial.println(F("[NFC] Blk 6 fail!"));
      return false;
    }

    // Authenticate block 8 (sector 2: blocks 8,9,10,11)
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 8, 0, keya)) {
      Serial.println(F("[NFC] Auth failed S2!"));
      return false;
    }

    // Write to blocks 8, 9 (sector 2)
    if (!nfc.mifareclassic_WriteDataBlock(8, data4)) {
      Serial.println(F("[NFC] Blk 8 fail!"));
      return false;
    }
    if (!nfc.mifareclassic_WriteDataBlock(9, data5)) {
      Serial.println(F("[NFC] Blk 9 fail!"));
      return false;
    }

    Serial.println(F("[NFC] OK!"));
    return true;
  } 
  else if (uidLength == 7) {
    // NTAG - different handling
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
    Serial.println(F("Commands: SET (save start=now) | TIME?"));
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
}

/* ======================= Setup ======================= */
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println(F("\n[BOOT] Multi-Sensor Logger + NFC"));

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  analogReference(DEFAULT);

  // DHT init
  Serial.println(F("[DHT] Init..."));
  dht.begin();
  delay(1000);
  (void)dht.readHumidity();
  (void)dht.readTemperature();
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

  // Time setup
  setNowFromCompile();
  startEpoch = nowTime();

  // MQ warmup
  Serial.println(F("[WARMUP] Stabilizing..."));
  unsigned long t0 = millis();
  while ((millis() - t0) <= WARMUP_MS) {
    delay(250);
  }

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

  Serial.println(F("\n[READY] Type SET or TIME?"));
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
        f.flush();
        f.close();
        sdStatus = "APPENDED";
      } else {
        sdStatus = "FAIL_OPEN";
        sd_ok = false;
      }
    }

    // Build NFC text FIRST - this ensures Serial, SD, and NFC all use SAME data
    // Separate date and time into different blocks
    time_t now = nowTime();
    g_nowDate = "ND:" + fmtDate(now);           // "ND:2025-10-25" = 13 bytes
    g_nowTime = "NT:" + fmtTime(now);           // "NT:14:53:22" = 12 bytes
    g_startDate = "SD:" + fmtDate(startEpoch);  // "SD:2025-10-25" = 13 bytes
    g_startTime = "ST:" + fmtTime(startEpoch);  // "ST:14:35:39" = 12 bytes
    g_latestNfcText = "M2=" + String((int)ppm2) + " M4=" + String((int)ppm4);  // "M2=3 M4=0"

    // Serial output (always show, even if SD fails)
    Serial.print(F("[SAMPLE] now="));   Serial.print(fmtEpoch(nowTime()));
    Serial.print(F(" | start="));       Serial.print(fmtEpoch(startEpoch));
    Serial.print(F(" | MQ2="));         Serial.print((int)ppm2);
    Serial.print(F(" | MQ4="));         Serial.print((int)ppm4);
    Serial.print(F(" | temp="));        Serial.print(tC,1);
    Serial.print(F(" | hum="));         Serial.println(hum,1);
    Serial.print(F("[SD] ")); Serial.println(sdStatus);
    Serial.print(F("[NFC] Data ready: ")); Serial.println(g_latestNfcText);
  }

  // NFC: Try to write when tag is presented (uses cached data from last sample)
  if (nfc_available && g_latestNfcText.length() > 0) {
    if (millis() - g_lastWriteMs > NFC_COOLDOWN_MS) {
      if (mifare_write_multiblock(g_nowDate, g_nowTime, g_startDate, g_startTime, g_latestNfcText)) {
        g_lastWriteMs = millis();
        delay(300);
      }
    }
  }
}