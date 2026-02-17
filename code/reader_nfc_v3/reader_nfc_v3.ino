#include <SPI.h>
#include <Adafruit_PN532.h>

#define PN532_SCK   (13)
#define PN532_MOSI  (11)
#define PN532_MISO  (12)
#define PN532_SS    (10)

// Hardware-SPI constructor (uses pins 13/12/11 and SS)
Adafruit_PN532 nfc(PN532_SS);

void setup() {
  Serial.begin(115200);
  Serial.println("NFC Reader - Ready");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 not found");
    while (1) { delay(10); }
  }

  nfc.SAMConfig();  // configure board to read tags
  Serial.println("Waiting for tag...");
}

void loop() {
  uint8_t uid[7];
  uint8_t uidLength = 0;

  // Timeout 100ms so we can print again quickly
  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 100)) {
    Serial.println("\nTag detected");
    Serial.print("UID Length: "); Serial.println(uidLength);
    Serial.print("UID: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uid[i] < 0x10) Serial.print("0");
      Serial.print(uid[i], HEX); Serial.print(" ");
    }
    Serial.println();

    if (uidLength == 4) {
      Serial.println("Type: MIFARE Classic");
      readMifareBlocks(uid, uidLength);
    } else if (uidLength == 7) {
      Serial.println("Type: NTAG");
      readNTAG();
    } else {
      Serial.println("Unknown / Unsupported tag type length");
    }

    delay(1500);
    Serial.println("\nWaiting for tag...");
  }

  delay(50);
}

/* -------------------- Helpers -------------------- */
void printAscii16(const uint8_t data[16]) {
  for (int i = 0; i < 16; i++) {
    uint8_t c = data[i];
    if (c >= 32 && c <= 126) {
      Serial.write(c);
    } else {
      Serial.print(".");
    }
  }
}

void printHex16(const uint8_t data[16]) {
  for (int i = 0; i < 16; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
}

/* -------------------- MIFARE Classic -------------------- */
void readMifareBlocks(uint8_t *uid, uint8_t uidLength) {
  // Default Key A (factory)
  uint8_t keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  Serial.println("\nReading MIFARE Classic Blocks 4..10");

  // -------- Sector 1: blocks 4..7 (7 is sector trailer) --------
  if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keyA)) {
    Serial.println("Authentication failed for sector 1 (blocks 4..7)");
    return;
  }

  for (uint8_t block = 4; block <= 7; block++) {
    uint8_t data[16];
    if (!nfc.mifareclassic_ReadDataBlock(block, data)) {
      Serial.print("Failed to read block "); Serial.println(block);
      continue;
    }

    if (block == 7) {
      Serial.print("Block 7 (Sector Trailer) HEX: ");
      printHex16(data);
      Serial.println();
    } else {
      Serial.print("Block "); Serial.print(block); Serial.print(" ASCII: ");
      printAscii16(data); Serial.println();

      Serial.print("Block "); Serial.print(block); Serial.print(" HEX:   ");
      printHex16(data); Serial.println();
    }
  }

  // -------- Sector 2: blocks 8..11 (we need 8..10) --------
  if (!nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 8, 0, keyA)) {
    Serial.println("Authentication failed for sector 2 (blocks 8..11)");
    return;
  }

  for (uint8_t block = 8; block <= 10; block++) {
    uint8_t data[16];
    if (!nfc.mifareclassic_ReadDataBlock(block, data)) {
      Serial.print("Failed to read block "); Serial.println(block);
      continue;
    }

    Serial.print("Block "); Serial.print(block); Serial.print(" ASCII: ");
    printAscii16(data); Serial.println();

    Serial.print("Block "); Serial.print(block); Serial.print(" HEX:   ");
    printHex16(data); Serial.println();
  }
}

/* -------------------- NTAG (Type 2) -------------------- */
void readNTAG() {
  // NTAG pages are 4 bytes each. Read 4..10 inclusive.
  Serial.println("\nReading NTAG pages 4..10:");
  for (uint8_t page = 4; page <= 10; page++) {
    uint8_t data[4];
    if (nfc.ntag2xx_ReadPage(page, data)) {
      Serial.print("Page "); Serial.print(page); Serial.print(" ASCII: ");
      for (int i = 0; i < 4; i++) {
        uint8_t c = data[i];
        if (c >= 32 && c <= 126) Serial.write(c); else Serial.print(".");
      }
      Serial.print("  HEX: ");
      for (int i = 0; i < 4; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX); Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.print("Failed to read page "); Serial.println(page);
    }
  }
}
