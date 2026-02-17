#include <SPI.h>
#include <Adafruit_PN532.h>

#define PN532_SCK   (13)
#define PN532_MOSI  (11)
#define PN532_MISO  (12)
#define PN532_SS    (10)

Adafruit_PN532 nfc(PN532_SS);

void setup() {
  Serial.begin(115200);
  Serial.println("NFC Reader - Ready");

  nfc.begin();
  
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 not found");
    while (1);
  }

  nfc.SAMConfig();
  Serial.println("Waiting for tag...");
}

void loop() {
  uint8_t uid[7];
  uint8_t uidLength;

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
      readMifareBlocks(uid);
    } else if (uidLength == 7) {
      Serial.println("Type: NTAG");
      readNTAG(uid);
    }
    
    delay(2000);
  }
  
  delay(100);
}

void readMifareBlocks(uint8_t *uid) {
  Serial.println("\nReading Blocks 4-9:");
  
  uint8_t keyA[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  // Authenticate sector 1 (blocks 4-7)
  if (!nfc.mifareclassic_AuthenticateBlock(uid, 4, 4, 0, keyA)) {
    Serial.println("Authentication failed for sector 1");
    return;
  }

  // Read blocks 4, 5, 6
  for (uint8_t block = 4; block <= 6; block++) {
    uint8_t data[16];
    
    if (!nfc.mifareclassic_ReadDataBlock(block, data)) {
      Serial.print("Failed to read block ");
      Serial.println(block);
      continue;
    }

    // ASCII format
    Serial.print("Block ");
    Serial.print(block);
    Serial.print(" ASCII: ");
    for (int i = 0; i < 16; i++) {
      if (data[i] >= 32 && data[i] <= 126) {
        Serial.write(data[i]);
      } else {
        Serial.print(".");
      }
    }
    Serial.println();

    // HEX format
    Serial.print("Block ");
    Serial.print(block);
    Serial.print(" HEX:   ");
    for (int i = 0; i < 16; i++) {
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  // Authenticate sector 2 (blocks 8-11)
  if (!nfc.mifareclassic_AuthenticateBlock(uid, 4, 8, 0, keyA)) {
    Serial.println("Authentication failed for sector 2");
    return;
  }

  // Read blocks 8, 9
  for (uint8_t block = 8; block <= 9; block++) {
    uint8_t data[16];
    
    if (!nfc.mifareclassic_ReadDataBlock(block, data)) {
      Serial.print("Failed to read block ");
      Serial.println(block);
      continue;
    }

    // ASCII format
    Serial.print("Block ");
    Serial.print(block);
    Serial.print(" ASCII: ");
    for (int i = 0; i < 16; i++) {
      if (data[i] >= 32 && data[i] <= 126) {
        Serial.write(data[i]);
      } else {
        Serial.print(".");
      }
    }
    Serial.println();

    // HEX format
    Serial.print("Block ");
    Serial.print(block);
    Serial.print(" HEX:   ");
    for (int i = 0; i < 16; i++) {
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void readNTAG(uint8_t *uid) {
  Serial.println("\nReading NTAG pages 4-7:");
  
  for (uint8_t page = 4; page < 8; page++) {
    uint8_t data[4];
    if (nfc.ntag2xx_ReadPage(page, data)) {
      Serial.print("Page ");
      Serial.print(page);
      Serial.print(" ASCII: ");
      for (int i = 0; i < 4; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
          Serial.write(data[i]);
        } else {
          Serial.print(".");
        }
      }
      Serial.print(" HEX: ");
      for (int i = 0; i < 4; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.print("Failed to read page ");
      Serial.println(page);
    }
  }
}