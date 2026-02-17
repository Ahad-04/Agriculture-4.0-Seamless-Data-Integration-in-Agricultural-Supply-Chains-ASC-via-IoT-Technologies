// ===== ESP32 BLE Central (HM-10 friendly; notify via CCCD write, ACK queue) =====
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "BLEDevice.h"  // Works with both Classic BLE & NimBLE in ESP32 core

// ---------- WiFi / Server ----------
const char* ssid     = "ChipVista 2.4G";
const char* password = "ChipVistaTrans";
const char* post_url = "http://192.168.18.20:5000/pong";

// HM-10 UART/FFE0-FFE1 (BLE 4.0, 20-byte payload)
static BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
static BLEUUID charUUID   ("0000FFE1-0000-1000-8000-00805F9B34FB");

static bool doConnect = false;
static bool connected = false;

static BLEClient *gClient = nullptr;
static BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice *myDevice = nullptr;

// --------- record ----------
typedef struct {
  uint32_t timestamp;
  float    latitude;
  float    longitude;
  uint16_t mq2;
  uint16_t mq4;
  float    temperature;
  float    pressure;
  uint8_t  humidity;
  uint8_t  status;
  uint16_t crc; // big-endian
} __attribute__((packed)) DataRecord;

static uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF, poly = 0x1021;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? ((crc << 1) ^ poly) : (crc << 1);
  }
  return (crc << 8) | (crc >> 8);
}

// ---------- frame reassembly ----------
static uint8_t rxBuf[512];
static size_t  rxLen = 0;

// ---------- ACK queue ----------
struct AckItem { uint16_t seq; };
static AckItem ackQ[32];
static volatile uint8_t ackHead = 0, ackTail = 0;

static inline bool ackEnq(uint16_t s) {
  uint8_t nHead = (uint8_t)((ackHead + 1) % 32);
  if (nHead == ackTail) return false; // full
  ackQ[ackHead].seq = s;
  ackHead = nHead;
  return true;
}
static inline bool ackDeq(uint16_t &s) {
  if (ackTail == ackHead) return false;
  s = ackQ[ackTail].seq;
  ackTail = (uint8_t)((ackTail + 1) % 32);
  return true;
}

static void handleRecordAndPost(const uint8_t* payload) {
  const DataRecord* r = (const DataRecord*)payload;
  uint16_t calc = crc16_ccitt((const uint8_t*)payload, sizeof(DataRecord)-2);
  if (r->crc != calc) { Serial.printf("REC_CRC_BAD calc=%04X found=%04X\n", calc, r->crc); return; }

  if (WiFi.isConnected()) {
    String json = "{";
    json += "\"timestamp\":"   + String(r->timestamp)           + ",";
    json += "\"latitude\":"    + String(r->latitude, 6)         + ",";
    json += "\"longitude\":"   + String(r->longitude, 6)        + ",";
    json += "\"mq2\":"         + String(r->mq2)                 + ",";
    json += "\"mq4\":"         + String(r->mq4)                 + ",";
    json += "\"temperature\":" + String(r->temperature, 1)      + ",";
    json += "\"pressure\":"    + String(r->pressure, 1)         + ",";
    json += "\"humidity\":"    + String(r->humidity)            + ",";
    json += "\"status\":"      + String(r->status);
    json += "}";
    HTTPClient http;
    http.begin(post_url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(json);
    Serial.printf("[HTTP] %d -> %s\n", code, json.c_str());
    http.end();
  } else {
    Serial.println("[HTTP] NO_NETWORK");
  }
}

static void parseFrames() {
  while (rxLen >= 8) {
    // seek header 0xAA 0x55
    size_t start = 0;
    while (start + 1 < rxLen && !(rxBuf[start] == 0xAA && rxBuf[start+1] == 0x55)) start++;
    if (start > 0) {
      memmove(rxBuf, rxBuf + start, rxLen - start);
      rxLen -= start;
      if (rxLen < 8) return;
    }
    if (!(rxLen >= 8 && rxBuf[0] == 0xAA && rxBuf[1] == 0x55)) return;

    uint16_t innerLen = (uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8);
    if (innerLen < 2 || innerLen > 512) {
      Serial.printf("LEN_BAD: inner=%u -> resync byte\n", innerLen);
      memmove(rxBuf, rxBuf + 1, rxLen - 1);
      rxLen -= 1;
      continue;
    }
    uint16_t payloadLen = innerLen - 2;
    size_t frameLen = 2 + 2 + 2 + payloadLen + 2;
    if (rxLen < frameLen) return;

    const uint8_t *frame = rxBuf;
    uint16_t seq = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

    uint16_t calc = crc16_ccitt(&frame[2], 4 + payloadLen);
    uint16_t got  = ((uint16_t)frame[6 + payloadLen] << 8) | (uint16_t)frame[7 + payloadLen];
    if (calc != got) {
      Serial.println("FRAME_CRC_BAD → resync");
      memmove(rxBuf, rxBuf + 1, rxLen - 1);
      rxLen -= 1;
      continue;
    }

    // queue ACK
    if (!ackEnq(seq)) {
      uint16_t tmp; ackDeq(tmp);
      ackEnq(seq);
      Serial.println("[ACK] queue full → dropped oldest");
    }

    // payload
    const uint8_t *payload = &frame[6];
    if (payloadLen == sizeof(DataRecord)) handleRecordAndPost(payload);

    // consume
    size_t remain = rxLen - frameLen;
    if (remain) memmove(rxBuf, rxBuf + frameLen, remain);
    rxLen = remain;
  }
}

static void notifyCallback(BLERemoteCharacteristic *, uint8_t *pData, size_t length, bool) {
  if (length) {
    size_t space = sizeof(rxBuf) - rxLen;
    if (length > space) {
      size_t drop = min(length - space, rxLen);
      memmove(rxBuf, rxBuf + drop, rxLen - drop);
      rxLen -= drop;
      Serial.printf("[RX] drop %u bytes to make room\n", (unsigned)drop);
    }
    size_t n = min(length, sizeof(rxBuf) - rxLen);
    memcpy(rxBuf + rxLen, pData, n);
    rxLen += n;
  }
  parseFrames();
}

// ----- BLE plumbing -----
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient*) override { Serial.println("[BLE] connected"); }
  void onDisconnect(BLEClient*) override {
    connected = false; pRemoteCharacteristic = nullptr;
    Serial.println("[BLE] disconnected");
  }
};

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    Serial.printf("[BLE] found: %s\n", d.toString().c_str());
    if (d.haveServiceUUID() && d.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      if (myDevice) delete myDevice;
      myDevice = new BLEAdvertisedDevice(d);
      doConnect = true;
    }
  }
};

// Enable notifications by writing CCCD 0x2902 to 0x0001
static bool enableNotifyCCCD(BLERemoteCharacteristic* ch) {
  BLEUUID cccdUUID((uint16_t)0x2902);
  BLERemoteDescriptor* cccd = ch->getDescriptor(cccdUUID);
  if (!cccd) { Serial.println("[BLE] CCCD not found"); return false; }
  uint8_t notifyOn[2] = {0x01, 0x00}; // notifications enabled
  bool ok = cccd->writeValue(notifyOn, 2, true);
  Serial.printf("[BLE] CCCD write %s\n", ok ? "OK" : "FAIL");
  return ok;
}

static bool connectToServer() {
  if (gClient) { if (gClient->isConnected()) gClient->disconnect(); delete gClient; gClient = nullptr; }
  gClient = BLEDevice::createClient();
  gClient->setClientCallbacks(new MyClientCallback());

  if (!gClient->connect(myDevice)) { Serial.println("[BLE] connect fail"); return false; }

  BLERemoteService *svc = gClient->getService(serviceUUID);
  if (!svc) { Serial.println("[BLE] service not found"); gClient->disconnect(); return false; }

  pRemoteCharacteristic = svc->getCharacteristic(charUUID);
  if (!pRemoteCharacteristic) { Serial.println("[BLE] char not found"); gClient->disconnect(); return false; }

  if (!(pRemoteCharacteristic->canNotify() || pRemoteCharacteristic->canIndicate())) {
    Serial.println("[BLE] char has no notify/indicate"); gClient->disconnect(); pRemoteCharacteristic = nullptr; return false;
  }

  // Set callback (returns void on NimBLE, bool on Classic; we ignore the return)
  pRemoteCharacteristic->registerForNotify(notifyCallback);

  // Manually enable notifications via CCCD (works on both stacks)
  if (!enableNotifyCCCD(pRemoteCharacteristic)) {
    Serial.println("[BLE] enable notify failed");
    gClient->disconnect(); pRemoteCharacteristic = nullptr; return false;
  }

  connected = true;
  Serial.println("[BLE] ready");
  return true;
}

// ----- ACK sender (safe context) -----
static void serviceAckQueue() {
  if (!connected || !pRemoteCharacteristic) return;

  uint16_t seq;
  uint8_t sent = 0;
  while (sent < 4 && ackDeq(seq)) {
    uint8_t ack[2] = { (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };

    // Try write-with-response if available; otherwise fallback (HM-10 often is WNR only)
    bool preferResp = pRemoteCharacteristic->canWrite();
    bool ok = false;
    for (int attempt = 1; attempt <= 3; ++attempt) {
      bool res = pRemoteCharacteristic->writeValue(ack, 2, /*response=*/preferResp);
      if (res) { Serial.printf("[ACK] seq=%u sent (%s)\n", seq, preferResp ? "resp" : "noresp"); ok = true; break; }
      Serial.printf("[ACK] seq=%u write fail (attempt %d)\n", seq, attempt);
      delay(8 * attempt);
    }
    if (!ok) { ackEnq(seq); break; } // requeue on persistent failure
    ++sent;
  }
}

// ----- Setup / Loop -----
void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  Serial.println("[WiFi] start");
  WiFi.begin(ssid, password);
  Serial.println("[WiFi] connecting...");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000) { delay(300); Serial.print("."); }
  if (WiFi.status() == WL_CONNECTED) Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
  else Serial.println("\n[WiFi] offline");

  BLEDevice::init(""); // Compatible with both implementations
  BLEScan *scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);
  scan->start(5, false);
}

void loop() {
  if (doConnect) { (void)connectToServer(); doConnect = false; }

  if (!connected) {
    static uint32_t lastScan = 0;
    if (millis() - lastScan > 5000) {
      lastScan = millis();
      BLEDevice::getScan()->stop();
      BLEDevice::getScan()->start(5, false);
    }
  } else {
    serviceAckQueue();
  }

  delay(5);
}
