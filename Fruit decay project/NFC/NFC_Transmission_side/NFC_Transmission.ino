/*
======================== Arduino Mega 2560 ========================

1) DHT22 (Temp/Humidity)
   - DHT22 VCC(+) -> 5V
   - DHT22 GND (-) -> GND
   - DHT22 DATA(out) -> Digital pin 7 

2) MQ-2 Gas Sensor (analog out)
   - MQ2 VCC    -> 5V
   - MQ2 GND    -> GND
   - MQ2 AO     -> A0      

3) MQ-4 Gas Sensor (analog out)
   - MQ4 VCC    -> 5V
   - MQ4 GND    -> GND
   - MQ4 AO     -> A1       

4) SD Card Module (SPI)  ***Arduino MEGA SPI pins***
   - SD VCC     -> 5V
   - SD GND     -> GND
   - SD CS      -> D10            
   - SD MOSI    -> D51
   - SD MISO    -> D50
   - SD SCK     -> D52

5) PN532 NFC (I2C mode)  ***Arduino MEGA I2C pins***
   - PN532 VCC  -> 5V  
   - PN532 GND  -> GND
   - PN532 SDA  -> D20 (SDA)
   - PN532 SCL  -> D21 (SCL)

===========================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <math.h>
#include <TimeLib.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <EEPROM.h>

/* ======================= Configuration ======================= */
#define DHTPIN   7
#define DHTTYPE  DHT22
#define CS_PIN   10
#define MQ2_A_Pin A0
#define MQ4_A_Pin A1

#define SAMPLE_INTERVAL_MS  30000UL       //  set this time(ms) after which you want fetch sensor values



#define WARMUP_MS           2000UL
#define DHT_RETRIES         4

// --- Sensor Constants ---
static const float VREF = 5.0f;
static const float VMIN = 0.01f;
static const float PPM_MAX_CLAMP = 65000.0f;

static const float MQ2_RL_OHMS = 12300.0f;
static const float MQ4_RL_OHMS = 23200.0f;

// Fixed R0 values (tune)
static const float MQ2_R0_OHMS = 10000.0f;
static const float MQ4_R0_OHMS = 3000.0f;

// Gas curve parameters (tune)
static const float MQ2_A = 50.0f;
static const float MQ2_B = -2.30f;
static const float MQ4_A = 1000.0f;
static const float MQ4_B = -2.95f;

// --- Time variables ---
time_t   startEpoch  = 0;
time_t   bootEpoch   = 0;
uint32_t bootMillis  = 0;

// DHT + SD
DHT dht(DHTPIN, DHTTYPE);
static bool sd_ok = false;
static String rxbuf;

/* ======================= NFC (PN532 over I2C, no IRQ/RST) ======================= */
Adafruit_PN532 nfc(-1, -1, &Wire);

static String g_SD = "", g_ST = "", g_ND = "", g_NT = ""; // 16-char lines
static String g_SGas = "", g_NGas = "";
static uint32_t g_lastWriteMs = 0;
static const uint32_t NFC_COOLDOWN_MS = 2000;
static bool nfc_available = false;

// latest instantaneous ppm (for NGas)
static int g_last_ppm2 = 0;
static int g_last_ppm4 = 0;

/* -------- Daily snapshot state -------- */
static const uint16_t MAX_DAYS = 39;     // Classic 1K capacity with our layout
static int day_m2[MAX_DAYS + 1];         // 1..MAX_DAYS; -1 = not set
static int day_m4[MAX_DAYS + 1];
static uint16_t prev_days = 0;           // days at previous sample

/* ======================= EEPROM persistence ======================= */
/*
Layout (offset 0):
  uint16_t magic = 0xC0DE
  uint8_t  version = 1
  uint8_t  reserved = 0
  uint32_t startEpoch
  uint16_t daysCount
  uint16_t m2[39]   // day 1..39 -> index 0..38; 0xFFFF = unset
  uint16_t m4[39]
Total ~170 bytes
*/
static const uint16_t EE_MAGIC   = 0xC0DE;
static const uint8_t  EE_VERSION = 1;

struct EEHeader {
  uint16_t magic;
  uint8_t  version;
  uint8_t  reserved;
  uint32_t startEpoch;
  uint16_t daysCount;
};

static const int EE_OFF_HDR   = 0;
static const int EE_OFF_M2    = EE_OFF_HDR + sizeof(EEHeader);
static const int EE_OFF_M4    = EE_OFF_M2 + (MAX_DAYS * sizeof(uint16_t));

EEHeader eeHdr;

void eeWriteHeader() { EEPROM.put(EE_OFF_HDR, eeHdr); }
void eeReadHeader()  { EEPROM.get(EE_OFF_HDR, eeHdr); }

void eeReset(uint32_t startEpoch_) {
  eeHdr.magic      = EE_MAGIC;
  eeHdr.version    = EE_VERSION;
  eeHdr.reserved   = 0;
  eeHdr.startEpoch = startEpoch_;
  eeHdr.daysCount  = 0;
  eeWriteHeader();
  // clear arrays in EEPROM to 0xFFFF
  for (int i=0; i<MAX_DAYS; ++i) {
    uint16_t v = 0xFFFF;
    EEPROM.put(EE_OFF_M2 + i*sizeof(uint16_t), v);
    EEPROM.put(EE_OFF_M4 + i*sizeof(uint16_t), v);
  }
}

void eeLoadOrReset(uint32_t startEpoch_) {
  eeReadHeader();
  if (eeHdr.magic != EE_MAGIC || eeHdr.version != EE_VERSION || eeHdr.startEpoch != startEpoch_) {
    eeReset(startEpoch_);
    return;
  }
  uint16_t dc = eeHdr.daysCount;
  if (dc > MAX_DAYS) dc = MAX_DAYS;
  prev_days = dc;
  for (int i=0; i<MAX_DAYS; ++i) {
    uint16_t v2, v4;
    EEPROM.get(EE_OFF_M2 + i*sizeof(uint16_t), v2);
    EEPROM.get(EE_OFF_M4 + i*sizeof(uint16_t), v4);
    day_m2[i+1] = (v2 == 0xFFFF) ? -1 : (int)v2;
    day_m4[i+1] = (v4 == 0xFFFF) ? -1 : (int)v4;
  }
}

void eeSaveDay(uint16_t dayIdx, int m2, int m4) {
  if (dayIdx < 1 || dayIdx > MAX_DAYS) return;
  uint16_t u2 = (m2 < 0) ? 0 : (m2 > 65535 ? 65535 : (uint16_t)m2);
  uint16_t u4 = (m4 < 0) ? 0 : (m4 > 65535 ? 65535 : (uint16_t)m4);
  EEPROM.put(EE_OFF_M2 + (dayIdx-1)*sizeof(uint16_t), u2);
  EEPROM.put(EE_OFF_M4 + (dayIdx-1)*sizeof(uint16_t), u4);
  if (dayIdx > eeHdr.daysCount) {
    eeHdr.daysCount = dayIdx;
    eeWriteHeader();
  }
}

/* ======================= Utils ======================= */
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

/* ---------- Time Helpers ---------- */
bool parseTimeString(const char* s, tmElements_t &tm) {
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

/* ---------- CSV first-row start gas ---------- */
bool getCsvFirstPPMs(float &mq2_ppm, float &mq4_ppm) {
  mq2_ppm = NAN; mq4_ppm = NAN;
  if (!sd_ok) return false;
  File f = SD.open(F("datalog.csv"), FILE_READ);
  if (!f) return false;

  String line = f.readStringUntil('\n'); // skip header
  while (f.available()) {
    line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int idx = 0, start = 0;
    String fields[15];
    for (int i=0; i<15; ++i) fields[i] = "";
    for (int i=0; i<=line.length(); ++i) {
      if (i == (int)line.length() || line[i] == ',') {
        if (idx < 15) fields[idx] = line.substring(start, i);
        idx++; start = i+1;
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

/* ======================= MIFARE CLASSIC HELPERS ======================= */
// Slots start at block 1 (skip manufacturer block 0), skip all sector trailers.
static const uint8_t MIFARE_KEYA[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// slot -> block mapping
static int slotToBlock(uint16_t slotIdx) {
  if (slotIdx == 0) return -1;
  if (slotIdx == 1) return 1;
  if (slotIdx == 2) return 2;
  uint32_t rem = slotIdx - 3;         // from block4 onward
  uint32_t sectorOffset = rem / 3;
  uint32_t within = rem % 3;          // 0..2
  uint32_t sector = 1 + sectorOffset; // sectors 1..15
  if (sector > 15) return -1;
  uint32_t base = sector * 4;
  return (int)(base + within);        // base, base+1, base+2
}

static bool writeBlockWithRetry(uint8_t block, const uint8_t *data, int tries=3) {
  for (int i=0;i<tries;i++) {
    if (nfc.mifareclassic_WriteDataBlock(block, (uint8_t*)data)) return true;
    delay(20);
  }
  return false;
}

static bool authForBlock(uint8_t *uid, uint8_t uidLength, uint8_t block) {
  return nfc.mifareclassic_AuthenticateBlock(uid, uidLength, block, 0, (uint8_t*)MIFARE_KEYA);
}

static bool readBlock(uint8_t block, uint8_t out[16]) {
  return nfc.mifareclassic_ReadDataBlock(block, out);
}

static bool isEmptyOrSpaces(const uint8_t b[16]) {
  bool all0 = true, all20 = true;
  for (int i=0;i<16;i++) {
    if (b[i] != 0x00) all0  = false;
    if (b[i] != 0x20) all20 = false;
  }
  return all0 || all20;
}

/* >>> CHANGED: compact 16-byte day line: "<d>D=<m2>,<m4>" <<< */
static void makeDayLine(uint16_t dayIdx, int m2, int m4, uint8_t out16[16]) {
  // Clamp to 0..65535 to keep <=5 digits
  if (m2 < 0) m2 = 0; if (m2 > 65535) m2 = 65535;
  if (m4 < 0) m4 = 0; if (m4 > 65535) m4 = 65535;
  char line[17];
  // examples: "1D=16327,2461", "10D=85,0", "39D=65535,65535"
  snprintf(line, sizeof(line), "%uD=%d,%d", dayIdx, m2, m4);
  memset(out16, 0x20, 16); // space pad
  for (int i=0; i<16 && line[i]; ++i) out16[i] = (uint8_t)line[i];
}

/* >>> CHANGED: overwrite check prefix "<d>D=" <<< */
static bool shouldOverwriteDayBlock(uint8_t *uid, uint8_t uidLen, uint8_t block, uint16_t d) {
  uint8_t buf[16];
  if (!authForBlock(uid, uidLen, block)) return true;
  if (!readBlock(block, buf)) return true;
  if (isEmptyOrSpaces(buf)) return true;
  // check prefix "<d>D="
  char expect[6];
  snprintf(expect, sizeof(expect), "%uD=", d);
  for (int i=0; expect[i] && i<5; ++i) {
    if (buf[i] != (uint8_t)expect[i]) return true; // not our format -> overwrite
  }
  return false; // looks like our day d line already
}

static void prepLine16(const String& s, uint8_t out[16]) {
  memset(out, 0x20, 16);
  int n = min((int)s.length(), 16);
  for (int i=0;i<n;i++) out[i] = (uint8_t)s[i];
}

// Clear (fill with spaces) all slots from startSlot to end of card
static void clearRemainingSlots(uint8_t *uid, uint8_t uidLen, uint16_t startSlot) {
  uint8_t blank[16]; memset(blank, 0x20, 16);
  for (uint16_t slot = startSlot; ; ++slot) {
    int blk = slotToBlock(slot);
    if (blk < 0) break;
    if (!authForBlock(uid, uidLen, (uint8_t)blk)) continue;
    (void)writeBlockWithRetry((uint8_t)blk, blank);
    delay(5);
  }
}

/* ======================= NFC writer ======================= */
/*
  Slots (start at block 1):
  1 -> SD:YYYY-MM-DD
  2 -> ST:HH:MM:SS
  3 -> ND:YYYY-MM-DD
  4 -> NT:HH:MM:SS
  5 -> Sm2=xxx Sm4=yyy
  6 -> Mq2=xxx Mq4=yyy
  7+ -> "<d>D=<m2>,<m4>"  (e.g., "1D=16327,2461")
*/
bool write_layout_and_days(const String& SD_line, const String& ST_line,
                           const String& ND_line, const String& NT_line,
                           const String& SGas_line, const String& NGas_line,
                           time_t startEpoch_, time_t nowEpoch_,
                           /*out*/ uint16_t &days_written) {
  days_written = 0;

  uint8_t uid[7]; uint8_t uidLength = 0;
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 300)) {
    return false; // no tag
  }
  Serial.println(F("[NFC] Tag detected!"));

  if (uidLength != 4) {
    Serial.println(F("[NFC] Only MIFARE Classic (4-byte UID) supported."));
    return false;
  }

  // Prepare base lines
  uint8_t b1[16], b2[16], b3[16], b4[16], b5[16], b6[16];
  prepLine16(SD_line,   b1);
  prepLine16(ST_line,   b2);
  prepLine16(ND_line,   b3);
  prepLine16(NT_line,   b4);
  prepLine16(SGas_line, b5);
  prepLine16(NGas_line, b6);

  // Write base slots (1..6) -> blocks 1,2,4,5,6,8
  for (uint16_t slot = 1; slot <= 6; ++slot) {
    int blk = slotToBlock(slot);
    if (blk < 0) { Serial.println(F("[NFC] Mapping error")); return false; }
    if (!authForBlock(uid, uidLength, (uint8_t)blk)) { Serial.print(F("[NFC] Auth fail slot ")); Serial.println(slot); return false; }
    uint8_t* pdata = (slot==1)?b1:(slot==2)?b2:(slot==3)?b3:(slot==4)?b4:(slot==5)?b5:b6;
    if (!writeBlockWithRetry((uint8_t)blk, pdata)) { Serial.print(F("[NFC] Write fail slot ")); Serial.println(slot); return false; }
  }

  // Compute days (full 24h periods)
  uint16_t days = 0;
  if (startEpoch_ != 0 && nowEpoch_ >= startEpoch_) {
    uint32_t elapsed = (uint32_t)(nowEpoch_ - startEpoch_);
    days = (uint16_t)(elapsed / 86400UL);
    if (days > MAX_DAYS) days = MAX_DAYS;
  }

  // Write snapshots for 1..days from EEPROM-backed arrays (only if set)
  if (days >= 1) {
    for (uint16_t d = 1; d <= days; ++d) {
      const uint16_t slot = 6 + d;          // day-1 -> slot 7
      const int blk = slotToBlock(slot);
      if (blk < 0) break;

      if (!authForBlock(uid, uidLength, (uint8_t)blk)) {
        Serial.print(F("[NFC] Auth fail day ")); Serial.println(d);
        break;
      }

      if (day_m2[d] >= 0 && day_m4[d] >= 0) {
        // overwrite rule: only if block empty or not matching "<d>D="
        if (shouldOverwriteDayBlock(uid, uidLength, (uint8_t)blk, d)) {
          uint8_t line[16]; makeDayLine(d, day_m2[d], day_m4[d], line);
          if (writeBlockWithRetry((uint8_t)blk, line)) {
            Serial.print(F("[NFC] Wrote day ")); Serial.print(d);
            Serial.print(F(" -> block ")); Serial.println(blk);
            days_written = d;
          } else {
            Serial.print(F("[NFC] Day ")); Serial.print(d); Serial.println(F(" write failed"));
          }
        } else {
          days_written = d; // already there in correct format
        }
      }
    }
    // ALWAYS clear everything after the last current day slot
    const uint16_t clearFrom = 6 + days + 1;
    clearRemainingSlots(uid, uidLength, clearFrom);
  } else {
    // <24h: ALWAYS clear all day slots
    clearRemainingSlots(uid, uidLength, 7);
  }

  Serial.println(F("[NFC] data written to passive tag"));
  if (days_written >= 1) {
    Serial.print(F("[NFC] Wrote day "));
    Serial.println(days_written);
  }
  return true;
}

/* ======================= Command handler ======================= */
void resetDayArrays() {
  for (uint16_t i=0;i<=MAX_DAYS;i++) { day_m2[i] = -1; day_m4[i] = -1; }
  prev_days = 0;
}

void handleCommand(String line) {
  line.trim();
  String u = line; u.toUpperCase();

  if (u == "HELP") {
    Serial.println(F("Commands:\n  SET (save start=now)\n  TIME?\n  TIME=YYYY-MM-DD HH:MM:SS"));
    return;
  }

  if (u == "SET") {
    startEpoch = nowTime();
    resetDayArrays();                 // RAM reset
    eeReset((uint32_t)startEpoch);    // EEPROM reset to new experiment
    Serial.print(F("[SET] New start time saved -> "));
    Serial.println(fmtEpoch(startEpoch));
    return;
  }

  if (u == "TIME?") {
    Serial.print(F("[TIME] Now="));   Serial.print(fmtEpoch(nowTime()));
    Serial.print(F(" | Start="));     Serial.println(fmtEpoch(startEpoch));
    return;
  }

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

  Serial.println(F("\n[BOOT] Multi-Sensor Logger + NFC + EEPROM daily snapshots (compact 16-byte day lines)"));
  Serial.println(F("[HINT] TIME=YYYY-MM-DD HH:MM:SS  then  SET"));

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  analogReference(DEFAULT);

  resetDayArrays();

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

  // Time init
  bool timeSet = false;
  if (setNowFromSD()) timeSet = true;

  unsigned long waitUntil = millis() + 2000;
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

  startEpoch = nowTime(); // default start; user can SET after giving TIME=...
  eeLoadOrReset((uint32_t)startEpoch);

  // MQ warmup
  Serial.println(F("[WARMUP] Stabilizing..."));
  unsigned long t0 = millis();
  while ((millis() - t0) <= WARMUP_MS) { delay(250); }

  // NFC init
  Serial.println(F("[NFC] Init (I2C)..."));
  Wire.begin();
  nfc.begin();
  uint32_t ver = nfc.getFirmwareVersion();
  if (!ver) {
    Serial.println(F("[NFC] Not found - disabled"));
    nfc_available = false;
  } else {
    Serial.print(F("[NFC] Firmware v"));
    Serial.println((ver >> 16) & 0xFF, HEX);
    nfc.SAMConfig();
    nfc_available = true;
  }

  Serial.println(F("\n[READY] TIME=..., SET; tap tag to write."));
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

    // MQ2
    uint16_t adc2 = readADCavg(MQ2_A_Pin);
    float v2 = adcToVolt(adc2);
    float rs2 = mqRsFromVolt(v2, MQ2_RL_OHMS);
    float ratio2 = rs2 / MQ2_R0_OHMS;
    float ppm2 = ppm_from_ratio(ratio2, MQ2_A, MQ2_B);

    // MQ4
    uint16_t adc4 = readADCavg(MQ4_A_Pin);
    float v4 = adcToVolt(adc4);
    float rs4 = mqRsFromVolt(v4, MQ4_RL_OHMS);
    float ratio4 = rs4 / MQ4_R0_OHMS;
    float ppm4 = ppm_from_ratio(ratio4, MQ4_A, MQ4_B);

    g_last_ppm2 = (int)ppm2;
    g_last_ppm4 = (int)ppm4;

    // DHT
    float hum, tC;
    dhtRead(hum, tC);

    // SD logging (unchanged)
    const char* sdStatus = "NOT_MOUNTED";
    if (sd_ok) {
      File f = SD.open(F("datalog.csv"), FILE_WRITE);
      if (f) {
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

    // Compose NFC strings (<=16 chars each)
    time_t now = nowTime();
    g_SD   = "SD:" + fmtDate(startEpoch);   // Slot 1 -> Block 1
    g_ST   = "ST:" + fmtTime(startEpoch);   // Slot 2 -> Block 2
    g_ND   = "ND:" + fmtDate(now);          // Slot 3 -> Block 4
    g_NT   = "NT:" + fmtTime(now);          // Slot 4 -> Block 5

    // Start gas from first CSV row (fallback to current)
    static bool startGasKnown = false;
    static int  startM2 = 0, startM4 = 0;
    if (!startGasKnown) {
      float s2, s4;
      if (getCsvFirstPPMs(s2, s4)) {
        startM2 = (int)round(s2); startM4 = (int)round(s4);
        startGasKnown = true;
        Serial.print(F("[START-GAS] CSV Sm2=")); Serial.print(startM2);
        Serial.print(F(" Sm4=")); Serial.println(startM4);
      } else {
        startM2 = (int)ppm2; startM4 = (int)ppm4;
      }
    }
    g_SGas = "Sm2=" + String(startM2) + " Sm4=" + String(startM4);        // Slot 5 -> Block 6
    g_NGas = "Mq2=" + String((int)ppm2) + " Mq4=" + String((int)ppm4);    // Slot 6 -> Block 8

    // ---- Update daily snapshots: capture once when NEW day appears; save to EEPROM ----
    uint16_t days = 0;
    if (now >= startEpoch) {
      uint32_t elapsed = (uint32_t)(now - startEpoch);
      days = (uint16_t)(elapsed / 86400UL);
      if (days > MAX_DAYS) days = MAX_DAYS;
    }

    if (days > prev_days) {
      for (uint16_t d = prev_days + 1; d <= days; ++d) {
        day_m2[d] = g_last_ppm2;
        day_m4[d] = g_last_ppm4;
        eeSaveDay(d, day_m2[d], day_m4[d]);   // persist immediately
      }
      prev_days = days;
    }

    // ---- Serial output ----
    Serial.print(F("[SAMPLE] start=")); Serial.print(fmtEpoch(startEpoch));
    Serial.print(F(" | now="));         Serial.print(fmtEpoch(now));
    Serial.print(F(" | Smq2="));        Serial.print(startM2);
    Serial.print(F(" Smq4="));          Serial.print(startM4);
    Serial.print(F(" | MQ2="));         Serial.print((int)ppm2);
    Serial.print(F(" | MQ4="));         Serial.print((int)ppm4);

    if (days >= 1) {
      Serial.print(F(" | "));
      for (uint16_t d=1; d<=days; ++d) {
        if (d>1) Serial.print(F(" | "));
        Serial.print(d); Serial.print(F("Dm2=")); Serial.print(day_m2[d] >= 0 ? day_m2[d] : -1);
        Serial.print(F("  "));
        Serial.print(d); Serial.print(F("Dm4=")); Serial.print(day_m4[d] >= 0 ? day_m4[d] : -1);
      }
    }

    Serial.print(F(" | temp="));        Serial.print(tC,1);
    Serial.print(F(" | hum="));         Serial.println(hum,1);

    Serial.print(F("[SD] ")); Serial.println(sdStatus);

    Serial.print(F("[NFC] Ready: SD=")); Serial.print(g_SD);
    Serial.print(F(" ST="));            Serial.print(g_ST);
    Serial.print(F(" ND="));            Serial.print(g_ND);
    Serial.print(F(" NT="));            Serial.print(g_NT);
    Serial.print(F(" SGas:"));          Serial.print(g_SGas);
    Serial.print(F(" NGas:"));          Serial.print(g_NGas);

    if (days >= 1) {
      Serial.print(F(" | "));
      for (uint16_t d=1; d<=days; ++d) {
        if (d>1) Serial.print(F(" | "));
        Serial.print(d); Serial.print(F("Dm2=")); Serial.print(day_m2[d] >= 0 ? day_m2[d] : -1);
        Serial.print(F("  "));
        Serial.print(d); Serial.print(F("Dm4=")); Serial.print(day_m4[d] >= 0 ? day_m4[d] : -1);
      }
    }
    Serial.println();
  }

  // NFC write when tag is presented
  if (nfc_available && g_NGas.length() > 0) {
    if (millis() - g_lastWriteMs > NFC_COOLDOWN_MS) {
      uint16_t days_written = 0;
      if (write_layout_and_days(
            g_SD, g_ST, g_ND, g_NT, g_SGas, g_NGas,
            startEpoch, nowTime(),
            days_written
          )) {
        g_lastWriteMs = millis();
        delay(300);
      }
    }
  }
}
