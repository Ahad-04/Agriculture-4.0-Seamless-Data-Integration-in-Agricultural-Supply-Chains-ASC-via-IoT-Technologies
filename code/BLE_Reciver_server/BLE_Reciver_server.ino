// ===== ESP32 BLE Central for Mega CSV dump =====
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "BLEDevice.h"

// ---------------- WiFi ----------------
const char* ssid     = "ChipVista 5G";
const char* password = "ChipVistaTrans";

// ---------------- Flask URLs ----------------
// change IP if your PC is different
const char* URL_PONG      = "http://192.168.18.20:5000/pong";
const char* URL_CSV_CHUNK = "http://192.168.18.20:5000/csv_chunk";
const char* URL_CSV_DONE  = "http://192.168.18.20:5000/csv_done";

// tiny control server to trigger REQ on ESP
WebServer web(80);
static bool g_triggerDump = false;

// ---------------- BLE (HM-10 style) ----------------
static BLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
static BLEUUID charUUID   ("0000FFE1-0000-1000-8000-00805F9B34FB");

static bool doConnect = false;
static bool connected = false;

static BLEClient* gClient = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static BLEAdvertisedDevice* myDevice = nullptr;

// ---------------- DataRecord (live 28-byte) ----------------
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

// ---------------- frame reassembly buffer ----------------
static uint8_t rxBuf[512];
static size_t  rxLen = 0;

// ---------------- ACK queue (to Mega) ----------------
struct AckItem { uint16_t seq; };
static AckItem ackQ[32];
static volatile uint8_t ackHead = 0, ackTail = 0;

static inline bool ackEnq(uint16_t s) {
  uint8_t nHead = (uint8_t)((ackHead + 1) % 32);
  if (nHead == ackTail) return false;
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

// ---------------- HTTP helpers ----------------
static void postRecordJSON(const DataRecord* r) {
  uint16_t calc = crc16_ccitt((const uint8_t*)r, sizeof(DataRecord) - 2);
  if (r->crc != calc) {
    Serial.printf("[REC] CRC bad (got %04X want %04X)\n", r->crc, calc);
    return;
  }
  if (!WiFi.isConnected()) {
    Serial.println("[HTTP JSON] no wifi");
    return;
  }

  String json = "{";
  json += "\"timestamp\":"   + String(r->timestamp)          + ",";
  json += "\"latitude\":"    + String(r->latitude, 6)      + ",";
  json += "\"longitude\":"   + String(r->longitude, 6)     + ",";
  json += "\"mq2\":"         + String(r->mq2)                + ",";
  json += "\"mq4\":"         + String(r->mq4)                + ",";
  json += "\"temperature\":" + String(r->temperature, 1)     + ",";
  json += "\"pressure\":"    + String(r->pressure, 1)      + ",";
  json += "\"humidity\":"    + String(r->humidity)           + ",";
  json += "\"status\":"      + String(r->status);
  json += "}";

  HTTPClient http;
  http.begin(URL_PONG);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(json);
  Serial.printf("[HTTP JSON] %d\n", code);
  http.end();
}

static void postCsvChunk(const uint8_t* data, size_t len) {
  if (!WiFi.isConnected()) {
    Serial.println("[HTTP CSV] no wifi");
    return;
  }
  HTTPClient http;
  http.begin(URL_CSV_CHUNK);
  // library wants non-const ptr
  int code = http.POST((uint8_t*)data, len);
  Serial.printf("[HTTP CSV] %d (%u bytes)\n", code, (unsigned)len);
  http.end();
}

static void sendCsvDone() {
  if (!WiFi.isConnected()) {
    Serial.println("[HTTP CSV_DONE] no wifi");
    return;
  }
  HTTPClient http;
  http.begin(URL_CSV_DONE);
  int code = http.GET();
  Serial.printf("[HTTP CSV_DONE] %d\n", code);
  http.end();
}

// ---------------- frame parser ----------------
static void parseFrames() {
  // try to parse as many frames as we can
  while (rxLen >= 8) {
    // 1) find header
    size_t start = 0;
    while (start + 1 < rxLen && !(rxBuf[start] == 0xAA && rxBuf[start+1] == 0x55)) start++;
    if (start > 0) {
      memmove(rxBuf, rxBuf + start, rxLen - start);
      rxLen -= start;
      if (rxLen < 8) return;
    }

    // now rxBuf[0..] starts with 0xAA 0x55
    if (!(rxLen >= 8 && rxBuf[0] == 0xAA && rxBuf[1] == 0x55)) return;

    // 2) read inner length
    uint16_t innerLen = (uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8);
    if (innerLen < 2 || innerLen > 512) {
      // corrupt -> drop 1 byte and retry
      Serial.printf("[PARSE] bad innerLen=%u, resync\n", innerLen);
      memmove(rxBuf, rxBuf + 1, rxLen - 1);
      rxLen -= 1;
      continue;
    }

    uint16_t payloadLen = innerLen - 2;
    size_t frameLen = 2 + 2 + 2 + payloadLen + 2;   // hdr + len + seq + payload + crc
    if (rxLen < frameLen) {
      // not enough yet
      return;
    }

    const uint8_t* frame = rxBuf;
    uint16_t seq = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);

    // 3) verify CRC
    uint16_t calc = crc16_ccitt(&frame[2], 4 + payloadLen);
    uint16_t got  = ((uint16_t)frame[6 + payloadLen] << 8) | (uint16_t)frame[7 + payloadLen];
    if (calc != got) {
      Serial.println("[PARSE] frame CRC bad, drop 1");
      memmove(rxBuf, rxBuf + 1, rxLen - 1);
      rxLen -= 1;
      continue;
    }

    // 4) ACK it (enqueue)
    if (!ackEnq(seq)) {
      uint16_t tmp; ackDeq(tmp);
      ackEnq(seq);
      Serial.println("[ACK] queue full -> dropped oldest");
    }

    // 5) handle payload
    const uint8_t* payload = &frame[6];

    if (payloadLen == sizeof(DataRecord)) {
      // live sensor packet
      postRecordJSON((const DataRecord*)payload);
    } else if (payloadLen == 0) {
      // END of CSV from Mega
      Serial.println("[BLE] END frame -> /csv_done");
      sendCsvDone();
    } else {
      // CSV text slice
      postCsvChunk(payload, payloadLen);
    }

    // 6) consume this frame from buffer
    size_t remain = rxLen - frameLen;
    if (remain) memmove(rxBuf, rxBuf + frameLen, remain);
    rxLen = remain;
  }
}

// ---------------- BLE notify callback ----------------
static void notifyCallback(BLERemoteCharacteristic *, uint8_t *pData, size_t length, bool) {
  if (length) {
    size_t space = sizeof(rxBuf) - rxLen;
    if (length > space) {
      // drop oldest to make room
      size_t drop = min(length - space, rxLen);
      memmove(rxBuf, rxBuf + drop, rxLen - drop);
      rxLen -= drop;
      Serial.printf("[RX] dropped %u bytes\n", (unsigned)drop);
    }
    memcpy(rxBuf + rxLen, pData, length);
    rxLen += length;
  }
  parseFrames();
}

// ---------------- BLE callbacks ----------------
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient*) override { Serial.println("[BLE] connected"); }
  void onDisconnect(BLEClient*) override {
    connected = false;
    pRemoteCharacteristic = nullptr;
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

static bool enableNotifyCCCD(BLERemoteCharacteristic* ch) {
  BLEUUID cccdUUID((uint16_t)0x2902);
  BLERemoteDescriptor* cccd = ch->getDescriptor(cccdUUID);
  if (!cccd) {
    Serial.println("[BLE] CCCD not found");
    return false;
  }
  uint8_t v[2] = {0x01, 0x00};
  bool ok = cccd->writeValue(v, 2, true);
  Serial.printf("[BLE] CCCD write %s\n", ok ? "OK" : "FAIL");
  return ok;
}

static bool connectToServer() {
  if (gClient) {
    if (gClient->isConnected()) gClient->disconnect();
    delete gClient;
    gClient = nullptr;
  }

  gClient = BLEDevice::createClient();
  gClient->setClientCallbacks(new MyClientCallback());

  if (!gClient->connect(myDevice)) {
    Serial.println("[BLE] connect fail");
    return false;
  }

  BLERemoteService* svc = gClient->getService(serviceUUID);
  if (!svc) {
    Serial.println("[BLE] service not found");
    gClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = svc->getCharacteristic(charUUID);
  if (!pRemoteCharacteristic) {
    Serial.println("[BLE] char not found");
    gClient->disconnect();
    return false;
  }

  if (!(pRemoteCharacteristic->canNotify() || pRemoteCharacteristic->canIndicate())) {
    Serial.println("[BLE] char no notify");
    gClient->disconnect();
    pRemoteCharacteristic = nullptr;
    return false;
  }

  pRemoteCharacteristic->registerForNotify(notifyCallback);

  if (!enableNotifyCCCD(pRemoteCharacteristic)) {
    gClient->disconnect();
    pRemoteCharacteristic = nullptr;
    return false;
  }

  connected = true;
  Serial.println("[BLE] ready");
  return true;
}

// send queued ACKs back to Mega
static void serviceAckQueue() {
  if (!connected || !pRemoteCharacteristic) return;

  uint16_t seq;
  uint8_t sent = 0;
  while (sent < 4 && ackDeq(seq)) {
    uint8_t ack[2] = { (uint8_t)(seq & 0xFF), (uint8_t)(seq >> 8) };
    bool canResp = pRemoteCharacteristic->canWrite();
    bool ok = pRemoteCharacteristic->writeValue(ack, 2, canResp);
    if (ok) {
      Serial.printf("[ACK] seq=%u sent\n", seq);
    } else {
      // put back
      ackEnq(seq);
      break;
    }
    sent++;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // WiFi
  WiFi.mode(WIFI_STA);
  Serial.printf("[WiFi] connecting to %s ...\n", ssid);
  WiFi.begin(ssid, password);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[HTTP] trigger: http://%s/pull_all\n", WiFi.localIP().toString().c_str());
  }

  // --- *** FIXED WEB SERVER SETUP *** ---
  // 1. Add ALL routes
  web.on("/pull_all", HTTP_GET, []() {
    g_triggerDump = true;
    web.send(200, "text/plain", "OK");
  });

  // NEW: Trigger fast sync (last ~2KB only)
  web.on("/pull_fast", HTTP_GET, []() {
    if (connected && pRemoteCharacteristic) {
       pRemoteCharacteristic->writeValue((uint8_t*)"FAST", 4, false);
       Serial.println("[BLE] sent FAST to Mega");
       web.send(200, "text/plain", "OK_FAST");
    } else {
       web.send(500, "text/plain", "Not connected to Mega");
    }
  });

  // 2. Call web.begin() ONCE
  web.begin();
  // --- *** END OF FIX *** ---

  // BLE
  BLEDevice::init("");
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);
  scan->start(5, false);
}

void loop() {
  web.handleClient();

  if (doConnect) {
    connectToServer();
    doConnect = false;
  }

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

  // when Flask hits /api/csv -> it calls ESP /pull_all -> set flag here
  if (g_triggerDump && connected && pRemoteCharacteristic) {
    const char reqCmd[] = "REQ";
    bool ok = pRemoteCharacteristic->writeValue((uint8_t*)reqCmd, 3, false);
    Serial.printf("[BLE] sent REQ to Mega: %s\n", ok ? "OK" : "FAIL");
    g_triggerDump = false;
  }

  delay(5);
}