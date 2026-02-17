#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "DHT.h"
#include <math.h>
#include <TimeLib.h>

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

// Fixed R0 values (no EEPROM)
static const float MQ2_R0_OHMS = 10000.0f;
static const float MQ4_R0_OHMS = 3000.0f;

// Gas curve parameters
static const float MQ2_A = 50.0f;
static const float MQ2_B = -2.30f;
static const float MQ4_A = 1000.0f;
static const float MQ4_B = -2.95f;

// --- Time variables ---
time_t   startEpoch  = 0;   // your saved "start" time (set at boot, or when you type SET)
time_t   bootEpoch   = 0;   // wall-clock corresponding to boot (compile time)
uint32_t bootMillis  = 0;   // millis() at boot sync

DHT dht(DHTPIN, DHTTYPE);
static const char* CSV_FILE = "datalog.csv";
static bool sd_ok = false;
static String rxbuf;

// ---------------- Utility functions ----------------
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

static File openAppend(const char* name) {
  File f = SD.open(name, FILE_WRITE);
  if (!f) {
    Serial.print(F("[CSV] open FAIL: ")); Serial.println(name);
    return File();
  }
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
      w.println(F("time_now,time_start,t_ms,mq2_adc,mq2_V,mq2_Rs,mq2_ratio,mq2_ppm,"
                  "mq4_adc,mq4_V,mq4_Rs,mq4_ratio,mq4_ppm,tempC,hum"));
      w.flush(); w.close();
      Serial.println(F("[CSV] header written"));
    }
  }
}

void dhtRead(float &hum, float &tC) {
  hum = NAN; tC = NAN;
  for (int i=0; i<DHT_RETRIES; ++i) {
    hum = dht.readHumidity(); 
    tC  = dht.readTemperature();
    if (!isnan(hum) && !isnan(tC)) return;
    delay(150);
  }
  if (isnan(hum)) hum = -1.0f;
  if (isnan(tC))  tC  = -1000.0f;
}

String fmtEpoch(time_t t) {
  if (t == 0) return "UNSET";
  char buf[20];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d", year(t), month(t), day(t), hour(t), minute(t), second(t));
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

// ---------------- Command handler ----------------
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

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Serial.println(F("[BOOT] DHT22 + MQ2 + MQ4 + SD Logger (Auto Time)"));

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  analogReference(DEFAULT);

  dht.begin();
  (void)dht.readHumidity();
  (void)dht.readTemperature();

  sd_ok = SD.begin(CS_PIN);
  if (sd_ok) ensureCsvHeader();
  else Serial.println(F("[SD] not mounted"));

  setNowFromCompile();               // use compile date/time as base clock
  startEpoch = nowTime();            // default: Start = boot time (you can change with SET)
  Serial.print(F("[TIME] Boot time -> "));  Serial.println(fmtEpoch(bootEpoch));
  Serial.print(F("[AUTO] Start time -> ")); Serial.println(fmtEpoch(startEpoch));

  unsigned long t0 = millis();
  while ((millis() - t0) <= WARMUP_MS) {
    delay(250);
    Serial.println(F("[WARMUP] MQ sensors stabilizing..."));
  }
  Serial.println(F("[READY] Logging started...  Type SET to overwrite Start, or TIME? to view."));
}

// ---------------- Loop ----------------
void loop() {
  // command shell
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (rxbuf.length()) { handleCommand(rxbuf); rxbuf = ""; }
    } else {
      if (rxbuf.length() < 96) rxbuf += ch;
    }
  }

  // periodic sample
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

    // DHT
    float hum, tC;
    dhtRead(hum, tC);

    // SD status
    const char* sdStatus = "NOT_MOUNTED";

    if (sd_ok) {
      File f = openAppend(CSV_FILE);
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
      }
    }

    // --- Serial output in two lines ---
    Serial.print(F("[SAMPLE] now="));   Serial.print(fmtEpoch(nowTime()));
    Serial.print(F(" | start="));       Serial.print(fmtEpoch(startEpoch));
    Serial.print(F(" | MQ2="));         Serial.print(ppm2,0);
    Serial.print(F(" | MQ4="));         Serial.print(ppm4,0);
    Serial.print(F(" | temp="));        Serial.print(tC,1);
    Serial.print(F(" | hum="));         Serial.println(hum,1);

    Serial.print(F("[SD] ")); Serial.println(sdStatus);
  }
}

