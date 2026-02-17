/*
======================== Arduino Nano Wiring ========================

1) OLED Display (I2C)
   - OLED VCC  -> 5V
   - OLED GND  -> GND
   - OLED SDA  -> A4
   - OLED SCL  -> A5

2) PN532 NFC Module (SPI Mode)
   - PN532 VCC  -> 5V
   - PN532 GND  -> GND
   - PN532 SCK  -> D13
   - PN532 MOSI -> D11
   - PN532 MISO -> D12
   - PN532 SS   -> D10
=====================================================================
*/


#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

/* ======================= Display Configuration ======================= */
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_ADDRESS   0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

/* ======================= PN532 (SPI) Pins ======================= */
#define PN532_SCK   (13)
#define PN532_MOSI  (11)
#define PN532_MISO  (12)
#define PN532_SS    (10)
Adafruit_PN532 nfc(PN532_SS);

/* ======================= UI Constants ======================= */
#define MAX_DISPLAY_LINES 6
#define READ_TIMEOUT       100
#define TAG_HOLD_TIME      10000UL   // 10 seconds to consider "removed"
#define LINE_HEIGHT        10
#define TEXT_SIZE          1

/* ======================= Keys ======================= */
const uint8_t DEFAULT_KEY[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ======================= State Variables ======================= */
uint8_t lastUID[7];
uint8_t lastUIDLength = 0;
unsigned long lastTagTime = 0;
bool tagPresent = false;

/* ---- Immediate Retry State for Same Tag ---- */
bool needRetry = false;                 // set true if any block/page failed
uint8_t retryCount = 0;                 // how many retries on same-present tag
unsigned long lastReadMillis = 0;       // last read (or retry) timestamp
const uint16_t RETRY_BACKOFF_MS = 200;  // delay between retries (tunable)
const uint8_t  RETRY_LIMIT = 8;         // safety cap to avoid infinite retries

/* ======================= Helpers (forward decl) ======================= */
void displayMessage(const char* message, uint8_t textSize);
void displayLinesC(const char lines[][17], int count);
bool isSameUID(uint8_t *uid, uint8_t uidLength);
bool readMifareSlots(uint8_t *uid, uint8_t uidLength);
bool readNTAG(uint8_t *uid, uint8_t uidLength);
bool authForBlock(uint8_t *uid, uint8_t uidLength, uint8_t block);
bool isBlockEmpty(const uint8_t data[16]);
int  slotToBlock(uint16_t slot);
static void toAscii16(const uint8_t data[16], char out[17]);
static bool extractKV(const char* line, const char* key, char* buf, size_t maxLen);

/* ===== parse compact day line "<d>D=<m2>,<m4>" ===== */
static bool parseDayCompact(const char text[17], uint16_t &day, uint16_t &m2, uint16_t &m4) {
  // expected: "<d>D=<m2>,<m4>", all decimal, may have trailing spaces
  // find 'D' and '=' and ','
  const char* D = strchr(text, 'D');
  if (!D) return false;
  const char* EQ = strchr(text, '=');
  if (!EQ) return false;
  const char* COM = strchr(text, ',');
  if (!COM) return false;
  if (!(D < EQ && EQ < COM)) return false;

  // day is from start to 'D'
  char bufDay[6] = {0}, bufM2[6] = {0}, bufM4[6] = {0};
  int lenDay = (int)(D - text);
  if (lenDay <= 0 || lenDay >= 6) return false;
  strncpy(bufDay, text, lenDay); bufDay[lenDay] = '\0';

  // m2 is between '=' and ','
  int lenM2 = (int)(COM - (EQ + 1));
  if (lenM2 <= 0 || lenM2 >= 6) return false;
  strncpy(bufM2, EQ + 1, lenM2); bufM2[lenM2] = '\0';

  // m4 is from after ',' up to space or end
  const char* p = COM + 1;
  int i = 0;
  while (*p && *p != ' ' && i < 5) { bufM4[i++] = *p++; }
  bufM4[i] = '\0';

  // check all are digits
  for (int k=0; bufDay[k]; ++k) if (bufDay[k] < '0' || bufDay[k] > '9') return false;
  for (int k=0; bufM2[k];  ++k) if (bufM2[k]  < '0' || bufM2[k]  > '9') return false;
  for (int k=0; bufM4[k];  ++k) if (bufM4[k]  < '0' || bufM4[k]  > '9') return false;

  day = (uint16_t)atoi(bufDay);
  m2  = (uint16_t)atoi(bufM2);
  m4  = (uint16_t)atoi(bufM4);
  return true;
}

/* ======================= Setup ======================= */
void setup() {
  Serial.begin(115200);
  Serial.println(F("NFC Reader Starting..."));

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1);
  }
  displayMessage("Initializing...", 1);

  // PN532 init
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println(F("PN532 not found!"));
    displayMessage("PN532 Error!\nCheck Wiring", 1);
    while (1);
  }
  Serial.print(F("Found PN532 v"));
  Serial.println((versiondata >> 24) & 0xFF, HEX);

  nfc.SAMConfig(); // configure PN532 to read MiFare cards
  displayMessage("NFC Ready\nWaiting for tag", 1);
  Serial.println(F("Waiting for NFC tag..."));
}

/* ======================= Loop ======================= */
void loop() {
  uint8_t uid[7];
  uint8_t uidLength;

  // Try to read a tag (short timeout to keep loop responsive)
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, READ_TIMEOUT)) {

    // New tag OR first time we see it after a removal
    if (!isSameUID(uid, uidLength) || !tagPresent) {
      memcpy(lastUID, uid, uidLength);
      lastUIDLength = uidLength;
      lastTagTime = millis();
      tagPresent = true;

      Serial.println(F("\n=== Tag Detected ==="));
      Serial.print(F("UID Length: ")); Serial.println(uidLength);
      Serial.print(F("UID: "));
      for (uint8_t i = 0; i < uidLength; i++) {
        if (uid[i] < 0x10) Serial.print('0');
        Serial.print(uid[i], HEX);
        if (i < uidLength - 1) Serial.print(' ');
      }
      Serial.println();

      // Reset retry state
      needRetry = false;
      retryCount = 0;
      lastReadMillis = 0; // force immediate first attempt

      // ---- First attempt immediately ----
      bool ok = false;
      if (uidLength == 4) {
        Serial.println(F("Tag Type: (heuristic) MIFARE Classic"));
        ok = readMifareSlots(uid, uidLength);
      } else if (uidLength == 7) {
        Serial.println(F("Tag Type: (heuristic) NTAG"));
        ok = readNTAG(uid, uidLength);
      } else {
        // Probe
        bool classicOK = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, (uint8_t*)DEFAULT_KEY);
        if (classicOK) {
          Serial.println(F("Probe -> Classic"));
          ok = readMifareSlots(uid, uidLength);
        } else {
          uint8_t test[4];
          if (nfc.ntag2xx_ReadPage(4, test)) {
            Serial.println(F("Probe -> NTAG"));
            ok = readNTAG(uid, uidLength);
          } else {
            Serial.println(F("Unknown tag type"));
            displayMessage("Unknown Tag\nType", 2);
          }
        }
      }

      needRetry = !ok;                 // if anything failed, arm immediate retry
      lastReadMillis = millis();

    } else {
      // Same tag still present
      lastTagTime = millis();

      // If last read had any failure, retry quickly (bounded)
      if (needRetry && retryCount < RETRY_LIMIT &&
          (millis() - lastReadMillis) >= RETRY_BACKOFF_MS) {

        bool ok = false;
        if (lastUIDLength == 4) {
          ok = readMifareSlots(lastUID, lastUIDLength);
        } else if (lastUIDLength == 7) {
          ok = readNTAG(lastUID, lastUIDLength);
        } else {
          bool classicOK = nfc.mifareclassic_AuthenticateBlock(lastUID, lastUIDLength, 4, 0, (uint8_t*)DEFAULT_KEY);
          if (classicOK) ok = readMifareSlots(lastUID, lastUIDLength);
          else {
            uint8_t test[4];
            if (nfc.ntag2xx_ReadPage(4, test)) ok = readNTAG(lastUID, lastUIDLength);
          }
        }

        needRetry = !ok;
        if (needRetry) retryCount++;
        lastReadMillis = millis();
      }
    }
  }

  // Consider tag removed only if we haven't seen it for TAG_HOLD_TIME
  if (tagPresent && (millis() - lastTagTime > TAG_HOLD_TIME)) {
    tagPresent = false;
    lastUIDLength = 0;
    needRetry = false;
    retryCount = 0;
    displayMessage("No Tag\nPresent", 2);
    Serial.println(F("Tag removed - ready for next scan"));
  }

  delay(50);
}

/* ======================= Utility Functions ======================= */

bool isSameUID(uint8_t *uid, uint8_t uidLength) {
  if (uidLength != lastUIDLength) return false;
  for (uint8_t i = 0; i < uidLength; i++) {
    if (uid[i] != lastUID[i]) return false;
  }
  return true;
}

void displayMessage(const char* message, uint8_t textSize) {
  display.clearDisplay();
  display.setTextSize(textSize);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 20);
  display.println(message);
  display.display();
}

// Print up to MAX_DISPLAY_LINES lines, 16 chars each
void displayLinesC(const char lines[][17], int count) {
  display.clearDisplay();
  display.setTextSize(TEXT_SIZE);
  display.setTextColor(SSD1306_WHITE);
  int y = 0;
  for (int i = 0; i < count && i < MAX_DISPLAY_LINES && y < SCREEN_HEIGHT; i++) {
    display.setCursor(0, y);
    display.println(lines[i]);
    y += LINE_HEIGHT;
  }
  display.display();
}

static void toAscii16(const uint8_t data[16], char out[17]) {
  for (uint8_t i=0;i<16;i++) {
    out[i] = (data[i] >= 32 && data[i] <= 126) ? (char)data[i] : '.';
  }
  out[16] = '\0';
}

// Extract value after a known 2–3 char key into buf (maxLen). Accepts ':' or '=' after key.
static bool extractKV(const char* line, const char* key, char* buf, size_t maxLen) {
  const char* p = strstr(line, key);
  if (!p) return false;
  p += strlen(key);
  if (*p == ':' || *p == '=') p++;
  while (*p == ' ') p++;
  size_t k = 0;
  while (*p && *p != ' ' && k+1 < maxLen) buf[k++] = *p++;
  buf[k] = '\0';
  return (k > 0);
}

/* ======================= MIFARE Classic mapping & read ======================= */

// Convert "slot index" -> physical block, skipping block 0 and all sector trailers.
// Slots: 1->blk1, 2->blk2, 3->blk4, 4->blk5, 5->blk6, 6->blk8, 7->blk9, 8->blk10, 9->blk12, ...
int slotToBlock(uint16_t slotIdx) {
  if (slotIdx == 0) return -1;
  if (slotIdx == 1) return 1;
  if (slotIdx == 2) return 2;
  // for slot >=3, subtract the first two and map into sectors >=1
  uint32_t rem = slotIdx - 3; // 0-based into [4,5,6, 8,9,10, 12,13,14, ...]
  uint32_t sectorOffset = rem / 3;     // 0..?
  uint32_t within = rem % 3;           // 0..2
  uint32_t sector = 1 + sectorOffset;  // sectors 1..15 valid on 1K
  if (sector > 15) return -1;
  uint32_t base = sector * 4;          // first block in that sector
  return (int)(base + within);         // base, base+1, base+2 (skip trailer)
}

bool authForBlock(uint8_t *uid, uint8_t uidLength, uint8_t block) {
  return nfc.mifareclassic_AuthenticateBlock(uid, uidLength, block, 0, (uint8_t*)DEFAULT_KEY);
}

bool isBlockEmpty(const uint8_t data[16]) {
  bool all0 = true, all20 = true;
  for (int i=0;i<16;i++) {
    if (data[i] != 0x00) all0  = false;
    if (data[i] != 0x20) all20 = false;
  }
  return all0 || all20;
}

// Return true only if ALL requested blocks were read OK.
bool readMifareSlots(uint8_t *uid, uint8_t uidLength) {
  char lines[MAX_DISPLAY_LINES][17] = {{0}};
  int lineIndex = 0;
  int blocksRead = 0;
  int blocksFailed = 0;

  // Buffers for compact summary
  char sdDate[17]="", stTime[17]="", ndDate[17]="", ntTime[17]="";
  char sm2[17]="", sm4[17]="", mq2[17]="", mq4[17]="";

  Serial.println(F("Reading MIFARE Classic (slots layout)…"));

  // ---------- Read base slots 1..6 ----------
  for (uint16_t slot=1; slot<=6; ++slot) {
    int blk = slotToBlock(slot);
    if (blk < 0) { blocksFailed++; continue; }

    if (!authForBlock(uid, uidLength, (uint8_t)blk)) {
      Serial.print(F("Auth failed for block ")); Serial.println(blk);
      blocksFailed++;
      continue;
    }
    uint8_t data[16];
    if (!nfc.mifareclassic_ReadDataBlock(blk, data)) {
      Serial.print(F("Read failed for block ")); Serial.println(blk);
      blocksFailed++;
      continue;
    }

    blocksRead++;

    // Serial HEX + ASCII
    Serial.print(F("Block ")); Serial.print(blk); Serial.print(F(": "));
    for (uint8_t i = 0; i < 16; i++) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX);
      Serial.print(' ');
    }
    Serial.print(F(" | "));

    char text[17]; toAscii16(data, text);
    Serial.println(text);

    // OLED: copy as-is
    if (lineIndex < MAX_DISPLAY_LINES) {
      strncpy(lines[lineIndex], text, 16);
      lines[lineIndex][16] = '\0';
      lineIndex++;
    }

    // Extract pieces for compact summary
    switch (slot) {
      case 1: extractKV(text, "SD", sdDate, sizeof(sdDate)); break;
      case 2: extractKV(text, "ST", stTime, sizeof(stTime)); break;
      case 3: extractKV(text, "ND", ndDate, sizeof(ndDate)); break;
      case 4: extractKV(text, "NT", ntTime, sizeof(ntTime)); break;
      case 5: {
        // "Sm2=xxx Sm4=yyy"
        char tmp[17]; strncpy(tmp, text, 16); tmp[16] = '\0';
        extractKV(tmp, "Sm2", sm2, sizeof(sm2));
        extractKV(tmp, "Sm4", sm4, sizeof(sm4));
      } break;
      case 6: {
        // "Mq2=xxx Mq4=yyy"
        char tmp[17]; strncpy(tmp, text, 16); tmp[16] = '\0';
        extractKV(tmp, "Mq2", mq2, sizeof(mq2));
        extractKV(tmp, "Mq4", mq4, sizeof(mq4));
      } break;
    }
  }

  // ---------- Read daily snapshots from slot 7 upwards ----------
  Serial.println(F("Reading Classic daily snapshot blocks..."));
  String daysStrStored = "";  // as stored, e.g. "1D=16327,2461"
  String daysStrNice   = "";  // friendly: "1Dm2=16327  1Dm4=2461"
  uint16_t day = 1;
  while (true) {
    uint16_t slot = 6 + day; // slot 7 => day 1, etc.
    int blk = slotToBlock(slot);
    if (blk < 0) break; // ran out of card

    if (!authForBlock(uid, uidLength, (uint8_t)blk)) {
      Serial.print(F("Auth failed for block ")); Serial.println(blk);
      blocksFailed++;
      break;
    }

    uint8_t data[16];
    if (!nfc.mifareclassic_ReadDataBlock(blk, data)) {
      Serial.print(F("Read failed for block ")); Serial.println(blk);
      blocksFailed++;
      break;
    }

    // Stop when this daily slot appears empty (means no more days)
    if (isBlockEmpty(data)) break;

    blocksRead++;

    // Serial HEX + ASCII
    Serial.print(F("Block ")); Serial.print(blk); Serial.print(F(": "));
    for (uint8_t i = 0; i < 16; i++) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX);
      Serial.print(' ');
    }
    Serial.print(F(" | "));

    char text[17]; toAscii16(data, text);
    Serial.println(text);

    // As stored (compact)
    if (daysStrStored.length() > 0) daysStrStored += " | ";
    daysStrStored += String(text);

    // Friendly parsed format
    uint16_t d, m2v, m4v;
    if (parseDayCompact(text, d, m2v, m4v)) {
      if (daysStrNice.length() > 0) daysStrNice += " | ";
      daysStrNice += String(d) + "Dm2=" + String(m2v) + "  " + String(d) + "Dm4=" + String(m4v);
    }

    day++;
  }

  // OLED
  if (lineIndex == 0) displayMessage("Read Failed\nCheck Key", 1);
  else                displayLinesC(lines, lineIndex);

  // ---- Compact summary line on Serial ----
  Serial.print(F("[TAG] start="));
  Serial.print(sdDate); Serial.print(F(" ")); Serial.print(stTime);
  Serial.print(F(" | now="));
  Serial.print(ndDate); Serial.print(F(" ")); Serial.print(ntTime);
  Serial.print(F(" | Smq2=")); Serial.print(sm2);
  Serial.print(F(" Smq4="));  Serial.print(sm4);
  Serial.print(F(" | MQ2="));  Serial.print(mq2);
  Serial.print(F(" | MQ4="));  Serial.print(mq4);

  if (daysStrNice.length() > 0) {
    Serial.print(F(" | ")); Serial.print(daysStrNice);
  } else if (daysStrStored.length() > 0) {
    // fallback to stored string if parsing failed (shouldn't usually)
    Serial.print(F(" | ")); Serial.print(daysStrStored);
  }
  Serial.println();

  Serial.print(F("Blocks read: ")); Serial.print(blocksRead);
  Serial.print(F(", Failed: "));     Serial.println(blocksFailed);

  return (blocksFailed == 0);
}

/* ======================= NTAG reader (unchanged) ======================= */
// Simple NTAG sweep: page 4..39 (adjust as needed)
#define NTAG_START_PAGE    4
#define NTAG_END_PAGE      39

bool readNTAG(uint8_t *uid, uint8_t uidLength) {
  char lines[MAX_DISPLAY_LINES][17] = {{0}};
  int lineIndex = 0;
  bool anyFail = false;

  char acc[64] = {0};
  int accLen = 0;

  Serial.println(F("Reading NTAG pages..."));
  for (uint8_t page = NTAG_START_PAGE; page <= NTAG_END_PAGE; page++) {
    uint8_t data[4];
    if (!nfc.ntag2xx_ReadPage(page, data)) {
      Serial.print(F("Failed to read page ")); Serial.println(page);
      anyFail = true;
      break;
    }

    Serial.print(F("Page ")); Serial.print(page); Serial.print(F(": "));
    for (uint8_t i = 0; i < 4; i++) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX);
      Serial.print(' ');
    }
    Serial.print(F(" | "));

    bool empty = ((data[0] | data[1] | data[2] | data[3]) == 0);
    for (uint8_t i = 0; i < 4; i++) {
      char c = (data[i] >= 32 && data[i] <= 126) ? (char)data[i] : '.';
      Serial.print(c);
      if (accLen < (int)sizeof(acc) - 1) acc[accLen++] = c;
    }
    Serial.println();

    if (empty && page > NTAG_START_PAGE + 2) break;

    while (accLen >= 16 && lineIndex < MAX_DISPLAY_LINES) {
      memcpy(lines[lineIndex], acc, 16);
      lines[lineIndex][16] = '\0';
      lineIndex++;
      memmove(acc, acc + 16, accLen - 16);
      accLen -= 16;
    }
    if (lineIndex >= MAX_DISPLAY_LINES) break;
  }

  if (accLen > 0 && lineIndex < MAX_DISPLAY_LINES) {
    memset(lines[lineIndex], 0, 17);
    memcpy(lines[lineIndex], acc, accLen);
    lines[lineIndex][16] = '\0';
    lineIndex++;
  }

  Serial.print(F("NTAG anyFail: ")); Serial.println(anyFail ? F("YES") : F("NO"));

  if (lineIndex == 0) displayMessage("NTAG Empty\nor Read Error", 1);
  else                displayLinesC(lines, lineIndex);

  return !anyFail;
}
