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

/* ======================= MIFARE Classic Config ======================= */
#define MIFARE_START_BLOCK 4
#define MIFARE_END_BLOCK   10
#define MIFARE_SKIP_BLOCK  7        // Sector trailer (for sector 1)
const uint8_t DEFAULT_KEY[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* ======================= NTAG Config ======================= */
#define NTAG_START_PAGE    4
#define NTAG_END_PAGE      39       // Adjust as needed for your NTAG capacity

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
bool readMifareBlocks(uint8_t *uid, uint8_t uidLength);
bool readNTAG(uint8_t *uid, uint8_t uidLength);

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

      // ---- First attempt immediately (keep your UID-length heuristic) ----
      bool ok = false;
      if (uidLength == 4) {
        Serial.println(F("Tag Type (heuristic): MIFARE Classic"));
        ok = readMifareBlocks(uid, uidLength);
      } else if (uidLength == 7) {
        Serial.println(F("Tag Type (heuristic): NTAG"));
        ok = readNTAG(uid, uidLength);
      } else {
        // Fallback probe: try Classic block 4 auth; otherwise try NTAG page 4
        bool classicOK = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, (uint8_t*)DEFAULT_KEY);
        if (classicOK) {
          Serial.println(F("Probe: Classic auth OK -> Classic"));
          ok = readMifareBlocks(uid, uidLength);
        } else {
          uint8_t test[4];
          if (nfc.ntag2xx_ReadPage(4, test)) {
            Serial.println(F("Probe: NTAG page read OK -> NTAG"));
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
          ok = readMifareBlocks(lastUID, lastUIDLength);
        } else if (lastUIDLength == 7) {
          ok = readNTAG(lastUID, lastUIDLength);
        } else {
          bool classicOK = nfc.mifareclassic_AuthenticateBlock(lastUID, lastUIDLength, 4, 0, (uint8_t*)DEFAULT_KEY);
          if (classicOK) ok = readMifareBlocks(lastUID, lastUIDLength);
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

/* ======================= Readers (return true if perfect read) ======================= */

// Return true only if ALL requested Classic blocks (except trailer) were read OK
bool readMifareBlocks(uint8_t *uid, uint8_t uidLength) {
  char lines[MAX_DISPLAY_LINES][17] = {{0}};
  int lineIndex = 0;
  int blocksRead = 0;
  int blocksFailed = 0;

  Serial.println(F("Reading MIFARE Classic blocks..."));
  for (uint8_t block = MIFARE_START_BLOCK; block <= MIFARE_END_BLOCK; block++) {
    if (block == MIFARE_SKIP_BLOCK) continue; // skip sector trailer

    // Authenticate sector/block
    if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, block, 0, (uint8_t*)DEFAULT_KEY)) {
      Serial.print(F("Auth failed for block ")); Serial.println(block);
      blocksFailed++;
      continue;
    }

    uint8_t data[16];
    if (!nfc.mifareclassic_ReadDataBlock(block, data)) {
      Serial.print(F("Read failed for block ")); Serial.println(block);
      blocksFailed++;
      continue;
    }

    blocksRead++;

    // Serial HEX + ASCII
    Serial.print(F("Block ")); Serial.print(block); Serial.print(F(": "));
    for (uint8_t i = 0; i < 16; i++) {
      if (data[i] < 0x10) Serial.print('0');
      Serial.print(data[i], HEX);
      Serial.print(' ');
    }
    Serial.print(F(" | "));

    char text[17];
    for (uint8_t i = 0; i < 16; i++) {
      text[i] = (data[i] >= 32 && data[i] <= 126) ? (char)data[i] : '.';
    }
    text[16] = '\0';
    Serial.println(text);

    if (lineIndex < MAX_DISPLAY_LINES) {
      strncpy(lines[lineIndex], text, 17);
      lineIndex++;
    }
  }

  Serial.print(F("Blocks read: ")); Serial.print(blocksRead);
  Serial.print(F(", Failed: "));     Serial.println(blocksFailed);

  if (lineIndex == 0) displayMessage("Read Failed\nCheck Key", 1);
  else                displayLinesC(lines, lineIndex);

  return (blocksFailed == 0);
}

// Return true only if NO page read failed during the sweep
bool readNTAG(uint8_t *uid, uint8_t uidLength) {
  char lines[MAX_DISPLAY_LINES][17] = {{0}};
  int lineIndex = 0;
  bool anyFail = false;

  // Accumulator to slice into 16-char OLED lines
  char acc[64] = {0};
  int accLen = 0;

  Serial.println(F("Reading NTAG pages..."));
  for (uint8_t page = NTAG_START_PAGE; page <= NTAG_END_PAGE; page++) {
    // NOTE: ntag2xx_ReadPage returns 4 bytes (one page), not 16
    uint8_t data[4];
    if (!nfc.ntag2xx_ReadPage(page, data)) {
      Serial.print(F("Failed to read page ")); Serial.println(page);
      anyFail = true;
      break;
    }

    // Serial HEX + ASCII
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

    // stop on empty run (after a few header pages)
    if (empty && page > NTAG_START_PAGE + 2) break;

    // Flush to 16-char lines for OLED
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
