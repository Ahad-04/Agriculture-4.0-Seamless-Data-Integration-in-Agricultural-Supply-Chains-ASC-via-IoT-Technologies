// ===================== Mega 2560 + HM-10 Reliable Sender (1 rec / 30s per file, SEQ+ACK+Retry) =====================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "DHT.h"
#include <ErriezDS3231.h>
#include <EEPROM.h>
#include <stdint.h>
#include <ctype.h>   // tolower
#include <string.h>  // strstr

// ------------------- HM-10 CONFIG -------------------
#define HM10_SERIAL        Serial1       // HM-10 UART on Mega: TX1=D18, RX1=D19
#define HM10_BAUD          9600
#define BLE_MTU_SIZE       20            // 20-byte BLE UART chunk
#define WirelessSerial     HM10_SERIAL   // <— alias so old code compiles

// OPTIONAL: HM-10 STATE pin (HIGH=connected on many modules)
#define BLE_STATE_PIN      8             // change if your module uses a different pin

// ------------------- Pins / Peripherals -------------------
#define DHTPIN           7
#define DHTTYPE          DHT11
#define LocationSerial   Serial2         // GPS on Serial2
#define CS_PIN           10
#define IRSensor         2
#define MQ4_A_Pin        A2
#define MQ2_A_Pin        A1
#define MQ4_D_Pin        31

// ------------------- Timings -------------------
//#define SAMPLE_INTERVAL_MS       30000UL   // one reading every 30 s -> one file
#define SAMPLE_INTERVAL_MS       900000UL  // one reading every 15 MINT -> one file
#define MQ2_WARM_UP_DELAY_MS     500UL
#define REQ_POLL_TIMEOUT_MS      0         // non-blocking

// ------------------- Reliability (ACK) -------------------
#define ACK_TIMEOUT_MS           1000
#define ACK_MAX_RETRIES          5
static uint16_t g_seq = 0;                // increases per frame

// ------------------- EEPROM filename slots -------------------
#define EEPROM_ADDR_FILENAME             0
#define EEPROM_ADDR_FILENAME_COMPLEMENT  4

// ------------------- RTC (DS3231) set-once-at-flash config -------------------
#define DS3231_ADDR 0x68
#define EEPROM_RTC_MAGIC_ADDR  16
#define RTC_MAGIC_VALUE        0x5A7CB2E1UL

// ------------------- Globals -------------------



// CSV file name for the rolling log
static const char* CSV_FILE = "datalog.csv";

// Forward declarations so we can call them anywhere
static void rtcReadYMDHMS(uint16_t &yr, uint8_t &mo, uint8_t &dy,
                          uint8_t &hh, uint8_t &mi, uint8_t &ss);



// Latest raw sensor readings for printing
static float g_lastDhtC = NAN;
static float g_lastBmpC = NAN;
static float g_lastHum  = NAN;
static float g_lastPres = NAN;     // <— new

static bool  bmpPresent = false;   // <— new

ErriezDS3231 ds3231;
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmpTemp = nullptr;
Adafruit_Sensor *bmpPressure = nullptr;
DHT dht(DHTPIN, DHTTYPE);
TinyGPSPlus gps;

volatile bool isWirelessRequestPending = false;
volatile bool isTransmissionInterrupted = false;
bool isWirelessSlaveConnected = false;
bool isFileSystemMountSuccess = false;

double lat = 0, lon = 0;

File dataFile;
char dataFilename[16]; // "XXXXXXXX.bin" (8 hex chars + ".bin" + NUL)

// ------------------- DataRecord (28 bytes) -------------------
typedef struct {
  uint32_t timestamp;   // epoch seconds
  float    latitude;
  float    longitude;
  uint16_t mq2;
  uint16_t mq4;
  float    temperature;
  float    pressure;
  uint8_t  humidity;
  uint8_t  status;      // IR (0/1)
  uint16_t crc;         // big-endian
} __attribute__((packed)) DataRecord;

union {
  DataRecord rec;
  uint8_t    bytes[sizeof(DataRecord)];
  // place this right after the union DU { ... } DU; block
  static void appendCSV(const DataRecord &rec);

} DU;

// ------------------- CRC (big-endian like receiver) -------------------
static uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF, poly = 0x1021;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? ((crc << 1) ^ poly) : (crc << 1);
  }
  return (crc << 8) | (crc >> 8); // return big-endian
}

// ------------------- Reliable SEQ+ACK framing -------------------
static bool waitAck(uint16_t wantSeq, uint32_t timeoutMs) {
  unsigned long t0 = millis();
  bool haveFirst = false;
  uint8_t first = 0;

  while (millis() - t0 < timeoutMs) {
    while (WirelessSerial.available()) {
      uint8_t b = (uint8_t)WirelessSerial.read();
      if (!haveFirst) {
        first = b;
        haveFirst = true;
      } else {
        // 2-byte sliding window (little-endian)
        uint16_t val = (uint16_t)first | ((uint16_t)b << 8);
        if (val == wantSeq) return true;
        // slide window forward by one byte
        first = b;
      }
    }
    delay(1);
  }
  return false;
}

static bool sendFramedBLE(const uint8_t *payload, size_t len) {
  uint8_t frame[2 + 2 + 2 + 28 + 2];  // header + len + seq + payload + crc
  frame[0] = 0xAA; frame[1] = 0x55;

  uint16_t innerLen = 2 + (uint16_t)len;
  frame[2] = (uint8_t)(innerLen & 0xFF);
  frame[3] = (uint8_t)(innerLen >> 8);

  frame[4] = (uint8_t)(g_seq & 0xFF);
  frame[5] = (uint8_t)(g_seq >> 8);

  if (len && payload) memcpy(&frame[6], payload, len);

  uint16_t c = crc16_ccitt(&frame[2], 4 + len);
  frame[6 + len] = (uint8_t)(c >> 8);
  frame[7 + len] = (uint8_t)(c & 0xFF);

  const size_t FL = 2 + 2 + 2 + len + 2;

  for (int attempt = 0; attempt < ACK_MAX_RETRIES; ++attempt) {
    for (size_t i = 0; i < FL; i += BLE_MTU_SIZE) {
      size_t n = min((size_t)BLE_MTU_SIZE, FL - i);
      WirelessSerial.write(&frame[i], n);
      delay(8);
    }
    if (waitAck(g_seq, ACK_TIMEOUT_MS)) {
      ++g_seq;
      return true;
    }
    delay(40);
  }
  return false;
}

// ----- DS3231 write helpers (UTC) -----
static uint8_t toBCD(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

// Clear Oscillator Stop Flag
static void rtcClearOSF() {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0F);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 1);
  uint8_t status = Wire.available() ? Wire.read() : 0;
  status &= ~(1 << 7); // clear OSF
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0F);
  Wire.write(status);
  Wire.endTransmission();
}

static bool rtcOscillatorStopped() {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x0F);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 1);
  if (!Wire.available()) return true;
  uint8_t status = Wire.read();
  return (status & (1 << 7)) != 0; // OSF bit
}

static void rtcSetYMDHMS(uint16_t yr, uint8_t mo, uint8_t dy,
                         uint8_t hh, uint8_t mm, uint8_t ss) {
  uint8_t y2k = (yr >= 2000) ? (yr - 2000) : (yr % 100);

  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  Wire.write(toBCD(ss));
  Wire.write(toBCD(mm));
  Wire.write(toBCD(hh));   // 24h mode
  Wire.write(0x01);        // DOW (unused)
  Wire.write(toBCD(dy));
  Wire.write(toBCD(mo));
  Wire.write(toBCD(y2k));
  Wire.endTransmission();

  rtcClearOSF();

  Serial.print(F("[RTC] set UTC "));
  Serial.print(yr); Serial.print('-');
  if (mo < 10) Serial.print('0'); Serial.print(mo); Serial.print('-');
  if (dy < 10) Serial.print('0'); Serial.print(dy); Serial.print(' ');
  if (hh < 10) Serial.print('0'); Serial.print(hh); Serial.print(':');
  if (mm < 10) Serial.print('0'); Serial.print(mm); Serial.print(':');
  if (ss < 10) Serial.print('0'); Serial.println(ss);
}

// One-shot: set RTC from GPS UTC when both date & time are valid.
static bool syncRTCFromGPSOnce(TinyGPSPlus& gps) {
  if (!gps.date.isValid() || !gps.time.isValid()) return false;
  uint16_t yr = gps.date.year();
  uint8_t  mo = gps.date.month();
  uint8_t  dy = gps.date.day();
  uint8_t  hh = gps.time.hour();
  uint8_t  mm = gps.time.minute();
  uint8_t  ss = gps.time.second();
  if (yr < 2020 || mo < 1 || mo > 12 || dy < 1 || dy > 31) return false;
  rtcSetYMDHMS(yr, mo, dy, hh, mm, ss);
  return true;
}



// Read DS3231 to Y-M-D H:M:S (UTC)
static void rtcReadYMDHMS(uint16_t &yr, uint8_t &mo, uint8_t &dy,
                          uint8_t &hh, uint8_t &mi, uint8_t &ss) {
  auto fromBCD = [](uint8_t b){ return (uint8_t)(10 * (b >> 4) + (b & 0x0F)); };

  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 7);

  if (Wire.available() >= 7) {
    ss = fromBCD(Wire.read() & 0x7F);
    mi = fromBCD(Wire.read() & 0x7F);
    uint8_t hr = Wire.read();
    if (hr & 0x40) { // 12h mode
      hh = fromBCD(hr & 0x1F);
      if (hr & 0x20) hh = (hh % 12) + 12;
    } else {         // 24h mode
      hh = fromBCD(hr & 0x3F);
    }
    (void)Wire.read();                 // DOW (unused)
    dy = fromBCD(Wire.read() & 0x3F);
    mo = fromBCD(Wire.read() & 0x1F);
    uint8_t y2k = fromBCD(Wire.read());
    yr = 2000 + y2k;
  } else {
    yr=1970; mo=1; dy=1; hh=0; mi=0; ss=0;
  }
}

// Append one CSV row (creates header on first write)
// Requires openForWriteRetry() and your globals.
static void appendCSV(const DataRecord &rec) {
  bool existed = SD.exists(CSV_FILE);
  File f = openForWriteRetry(CSV_FILE);
  if (!f) { Serial.println(F("[CSV] open FAIL")); return; }

  if (!existed || f.size() == 0) {
    f.println(F("epoch,datetime_utc,lat,lon,mq2,mq4,tempC,pressure_hPa,hum,ir,mq4_d,crc_hex"));
  }

  // Format current RTC as ISO (UTC)
  uint16_t y; uint8_t mo, d, h, m, s;
  rtcReadYMDHMS(y, mo, d, h, m, s);
  char iso[24];
  snprintf(iso, sizeof(iso), "%04u-%02u-%02u %02u:%02u:%02u",
           (unsigned)y, (unsigned)mo, (unsigned)d, (unsigned)h, (unsigned)m, (unsigned)s);

  uint8_t mq4_d = digitalRead(MQ4_D_Pin);

  // Write CSV row
  f.print(rec.timestamp);      f.print(',');
  f.print(iso);                f.print(',');
  f.print(rec.latitude, 6);    f.print(',');
  f.print(rec.longitude, 6);   f.print(',');
  f.print(rec.mq2);            f.print(',');
  f.print(rec.mq4);            f.print(',');
  f.print(rec.temperature, 2); f.print(',');
  f.print(rec.pressure, 1);    f.print(',');
  f.print(rec.humidity);       f.print(',');
  f.print(rec.status);         f.print(',');
  f.print(mq4_d);              f.print(',');
  f.println(rec.crc, HEX);

  f.close();
  Serial.println(F("[CSV] appended"));
}




// Build-time helpers
static uint8_t monthFromAbbrev(const char* m) {
  const char* names = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char* p = strstr(names, m);
  return p ? (uint8_t)((p - names) / 3 + 1) : 1;
}
static void syncRTCFromBuild() {
  char mmm[4]; int dd, yyyy, hh, mm, ss;
  sscanf(__DATE__, "%3s %d %d", mmm, &dd, &yyyy);
  sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
  rtcSetYMDHMS(yyyy, monthFromAbbrev(mmm), (uint8_t)dd, (uint8_t)hh, (uint8_t)mm, (uint8_t)ss);
}

// Set-once-at-flash: only set from build time if needed
static void initRTC_AtFlashIfNeeded() {
  uint32_t magic = 0;
  EEPROM.get(EEPROM_RTC_MAGIC_ADDR, magic);
  bool needInit = (magic != RTC_MAGIC_VALUE) || rtcOscillatorStopped();

  if (needInit) {
    syncRTCFromBuild();
    EEPROM.put(EEPROM_RTC_MAGIC_ADDR, (uint32_t)RTC_MAGIC_VALUE);
  } else {
    Serial.println(F("[RTC] already initialized; skipping build-time set"));
  }
}

// ------------------- Helpers -------------------
static void initialize_BMP() {
  bmpPresent = false;
  if (bmp.begin(0x76) || bmp.begin(0x77)) {
    bmpPresent = true;
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    bmpTemp     = bmp.getTemperatureSensor();
    bmpPressure = bmp.getPressureSensor();
    Serial.println(F("[BMP280] ready"));
  } else {
    bmpTemp = nullptr;
    bmpPressure = nullptr;
    Serial.println(F("[BMP280] not found (0x76/0x77)"));
  }
}

static void retrieveDataFileName() {
  uint32_t stored, comp;
  EEPROM.get(EEPROM_ADDR_FILENAME, stored);
  EEPROM.get(EEPROM_ADDR_FILENAME_COMPLEMENT, comp);
  if (stored != ~comp) {
    uint32_t epoch = ds3231.getEpoch();
    uint32_t complement = ~epoch;
    EEPROM.put(EEPROM_ADDR_FILENAME, epoch);
    EEPROM.put(EEPROM_ADDR_FILENAME_COMPLEMENT, complement);
    snprintf(dataFilename, sizeof(dataFilename), "%08lX.bin", epoch);
    Serial.print(F("[EEPROM] set filename: ")); Serial.println(dataFilename);
  } else {
    snprintf(dataFilename, sizeof(dataFilename), "%08lX.bin", stored);
    Serial.print(F("[EEPROM] restored filename: ")); Serial.println(dataFilename);
  }
}

static void assignNewFilenameFromEpoch() {
  uint32_t epoch = ds3231.getEpoch();
  uint32_t complement = ~epoch;
  EEPROM.put(EEPROM_ADDR_FILENAME, epoch);
  EEPROM.put(EEPROM_ADDR_FILENAME_COMPLEMENT, complement);
  snprintf(dataFilename, sizeof(dataFilename), "%08lX.bin", epoch);
  Serial.print(F("[FS] new file: ")); Serial.println(dataFilename);
}

static uint16_t readADCavg(uint8_t pin) {
  uint32_t acc = 0;
  for (int i=0;i<8;i++){ acc += analogRead(pin); delay(2); }
  return (uint16_t)(acc/8);
}

static inline bool bleConnected() {
  return digitalRead(BLE_STATE_PIN) == HIGH;
}

static bool endsWithBinCI(const char* name) {
  size_t L = strlen(name);
  if (L < 4) return false;
  char b0 = tolower(name[L-3]);
  char b1 = tolower(name[L-2]);
  char b2 = tolower(name[L-1]);
  return (name[L-4]=='.' && b0=='b' && b1=='i' && b2=='n');
}

// Only parse ASCII commands here; do NOT swallow binary ACKs
static void checkWirelessRequest(uint32_t /*timeout_ms*/) {
  // Non-blocking single sweep: only look for 'R','E','Q' and ignore other printable bytes.
  // Leave non-printable (binary) in the buffer for ACK parser.
  static uint8_t state = 0; // 0: none, 1:'R', 2:'RE'
  while (WirelessSerial.available()) {
    int c = WirelessSerial.peek();
    if (c < 0) break;

    if (c < 32) return;           // stop at binary → let ACK/frames consume it
    (void)WirelessSerial.read();  // consume printable

    char ch = (char)c;
    switch (state) {
      case 0: state = (ch=='R') ? 1 : 0; break;
      case 1: state = (ch=='E') ? 2 : (ch=='R'?1:0); break;
      case 2:
        if (ch=='Q') { isWirelessRequestPending = true; state = 0; }
        else state = (ch=='R') ? 1 : 0;
        break;
    }
    // ignore everything else (e.g., 'E','R','R','O','R')
  }
}

// ---------- SD helper: open with one retry (re-begin if needed) ----------
static File openForWriteRetry(const char* name) {
  File f = SD.open(name, FILE_WRITE);
  if (f) return f;
  // re-begin and try again
  if (!SD.begin(CS_PIN)) {
    Serial.println(F("[SD] re-begin FAIL"));
    return File();
  }
  Serial.println(F("[SD] re-begin OK, retry open..."));
  return SD.open(name, FILE_WRITE);
}

static void fillDataRecordOnce(DataRecord &rec) {
  rec.timestamp = ds3231.getEpoch();
  rec.latitude  = gps.location.isValid() ? gps.location.lat() : 0.0;
  rec.longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
  rec.mq2       = readADCavg(MQ2_A_Pin);
  rec.mq4       = readADCavg(MQ4_A_Pin);
  rec.status    = digitalRead(IRSensor) ? 1 : 0;

  // DHT
  float dhtHum  = dht.readHumidity();
  float dhtC    = dht.readTemperature();

  // BMP guarded
  float bmpC = NAN, pres = NAN;
  if (bmpPresent) {
    sensors_event_t tEv = {}, pEv = {};
    if (bmpTemp)     bmpTemp->getEvent(&tEv);
    if (bmpPressure) bmpPressure->getEvent(&pEv);
    bmpC = tEv.temperature;
    pres = pEv.pressure;
  }

  // Keep for printing
  g_lastDhtC = dhtC;
  g_lastBmpC = bmpC;
  g_lastHum  = dhtHum;
  g_lastPres = pres;

  // Use DHT if valid; else BMP; else 0
  rec.temperature = (!isnan(dhtC)) ? dhtC :
                    (!isnan(bmpC) ? bmpC : 0.0f);
  // Store pressure (0 if not available)
  rec.pressure    = (!isnan(pres)) ? pres : 0.0f;

  rec.humidity    = (uint8_t)constrain((int)(!isnan(dhtHum) ? lround(dhtHum) : 0), 0, 100);

  rec.crc = crc16_ccitt((const uint8_t*)&rec, sizeof(DataRecord) - 2);
}

// --------- human-readable sensor printout ---------
static void printDataRecord(const DataRecord &rec) {
  uint8_t mq4_d = digitalRead(MQ4_D_Pin);

  Serial.print(F("[DATA] ts="));  Serial.print(rec.timestamp);
  Serial.print(F(" lat="));       Serial.print(rec.latitude, 6);
  Serial.print(F(" lon="));       Serial.print(rec.longitude, 6);
  Serial.print(F(" mq2="));       Serial.print(rec.mq2);
  Serial.print(F(" mq4="));       Serial.print(rec.mq4);

  Serial.print(F(" tempC="));     Serial.print(rec.temperature, 1);

  Serial.print(F(" tdhtC="));
  if (isnan(g_lastDhtC)) Serial.print(F("NA")); else Serial.print(g_lastDhtC, 1);

  Serial.print(F(" tbmpC="));
  if (!bmpPresent || isnan(g_lastBmpC)) Serial.print(F("NA")); else Serial.print(g_lastBmpC, 1);

  Serial.print(F(" pres_hPa="));
  if (!bmpPresent || isnan(g_lastPres)) Serial.print(F("NA")); else Serial.print(g_lastPres, 1);

  Serial.print(F(" hum%="));
  if (isnan(g_lastHum)) Serial.print(F("NA")); else Serial.print((int)rec.humidity);

  Serial.print(F(" ir="));        Serial.print(rec.status);
  Serial.print(F(" mq4_d="));     Serial.print(mq4_d);
  Serial.print(F(" crc=0x"));     Serial.println(rec.crc, HEX);
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  Serial.print(F("sizeof(DataRecord)=")); Serial.println(sizeof(DataRecord));

  // Keep Mega as SPI master & deselect common SS lines
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);
  pinMode(53, OUTPUT); digitalWrite(53, HIGH);

  Wire.begin();
  Wire.setClock(100000);
  ds3231.begin();

  // Set RTC once (from build time) after flashing or if oscillator stopped
  initRTC_AtFlashIfNeeded();

  pinMode(BLE_STATE_PIN, INPUT);
  pinMode(IRSensor, INPUT_PULLUP);
  pinMode(MQ4_D_Pin, INPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  WirelessSerial.begin(HM10_BAUD);
  WirelessSerial.setTimeout(0);

  LocationSerial.begin(9600);
  dht.begin();
  initialize_BMP();

  Serial.print(F("[SD] init..."));
  if (SD.begin(CS_PIN)) {
    Serial.println(F("OK"));
    isFileSystemMountSuccess = true;
    retrieveDataFileName();
    File f = SD.open(dataFilename, FILE_WRITE);
    if (f) f.close();
  } else {
    Serial.println(F("FAILED"));
  }

  unsigned long t0 = millis();
  while (millis() - t0 <= MQ2_WARM_UP_DELAY_MS) {
    delay(250);
    Serial.println(F("waiting for MQ2 to warm up..."));
  }
}

// ------------------- LOOP -------------------
void loop() {
  while (LocationSerial.available() > 0) {
    gps.encode(LocationSerial.read());
  }

  // One-shot: sync RTC from GPS when date+time become valid
  static bool rtcSyncedFromGPS = false;
  if (!rtcSyncedFromGPS && syncRTCFromGPSOnce(gps)) {
    rtcSyncedFromGPS = true;
    Serial.println(F("[RTC] synced from GPS (UTC)."));
  }

  static uint32_t lastSample = 0;
  uint32_t now = millis();

  // 1) Every 30s: take ONE snapshot → ONE new file with ONE record
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    if (!isFileSystemMountSuccess) {
      Serial.println(F("[SD] not mounted"));
    } else {
      assignNewFilenameFromEpoch();
      fillDataRecordOnce(DU.rec);

      // print sensor values for this snapshot
      printDataRecord(DU.rec);

      appendCSV(DU.rec);           // <— add this line

      
      File f = openForWriteRetry(dataFilename);   // <— retry-aware open
      if (f) {
        f.write(DU.bytes, sizeof(DU.bytes));
        f.close();
        Serial.print(F("[SD] wrote 1 record to ")); Serial.println(dataFilename);
      } else {
        Serial.print(F("[SD] open FAIL: ")); Serial.println(dataFilename);
      }

      if (!sendFramedBLE(DU.bytes, sizeof(DU.bytes))) {
        Serial.println(F("[BLE] live send failed (no ACK)"));
      }
    }
  }

  

  // 2) Handle REQ from ESP32 → stream ALL .bin files and send END after each file
  checkWirelessRequest(REQ_POLL_TIMEOUT_MS);
  if (isWirelessRequestPending) {
    isWirelessRequestPending = false;
    Serial.println(F("[BLE] REQ → stream all .bin files"));

    if (!isFileSystemMountSuccess) {
      Serial.println(F("[BLE] WARN: SD not mounted"));
    } else {
      File root = SD.open("/");
      if (root) {
        for (;;) {
          File entry = root.openNextFile();
          if (!entry) break; // done
          if (!entry.isDirectory()) {
            const char *name = entry.name();
            if (endsWithBinCI(name)) {
              Serial.print(F("[BLE] sending ")); Serial.println(name);
              isTransmissionInterrupted = false;

              while (entry.available()) {
                uint8_t recBuf[sizeof(DataRecord)];
                int n = entry.read(recBuf, sizeof(DataRecord));
                if (n <= 0) break;
                if (n != (int)sizeof(DataRecord)) {
                  Serial.println(F("[BLE] partial record, skip tail"));
                  break;
                }

                if (!bleConnected()) {
                  Serial.println(F("[BLE] Disconnected → abort stream"));
                  isTransmissionInterrupted = true;
                  break;
                }
                if (!sendFramedBLE(recBuf, sizeof(DataRecord))) {
                  Serial.println(F("[BLE] send failed after retries → abort file"));
                  isTransmissionInterrupted = true;
                  break;
                }
                delay(6); // gentle pacing
              }

              if (!isTransmissionInterrupted && bleConnected()) {
                if (!sendFramedBLE(nullptr, 0)) {
                  Serial.println(F("[BLE] END not ACKed"));
                }
              }
            }
          }
          entry.close();
        }
        root.close();
        Serial.println(F("[BLE] stream done"));
      } else {
        Serial.println(F("[SD] root open FAIL"));
      }
    }
  }
}
