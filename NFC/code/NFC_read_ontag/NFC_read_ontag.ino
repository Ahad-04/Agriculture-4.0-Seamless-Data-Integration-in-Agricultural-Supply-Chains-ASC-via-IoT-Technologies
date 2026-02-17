#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

#define PN532_SCK   (13)
#define PN532_MOSI  (11)
#define PN532_MISO  (12)
#define PN532_SS    (10)

Adafruit_PN532 nfc(PN532_SS);

void setup() {
  Serial.begin(115200);
  Serial.println("🔍 Detecting Tag Type...");

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("❌ PN532 not found!");
    while (1);
  }

  nfc.SAMConfig();
  Serial.println("✨ Ready! Bring tag close...");
}

void loop() {
  uint8_t uid[7];
  uint8_t uidLength;

  if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
    Serial.println("\n🎯 Tag detected!");
    Serial.print("UID Length: "); Serial.println(uidLength);
    Serial.print("UID: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      Serial.print(uid[i], HEX); Serial.print(" ");
    }
    Serial.println();

    if (uidLength == 4) {
      Serial.println("🧱 Likely MIFARE Classic (1K or 4K).");
      readMifareClassic(uid);
    } else if (uidLength == 7) {
      Serial.println("💡 Likely NTAG213/215/216.");
      readNTAG(uid);
    }
    delay(3000);
  }
}

void readMifareClassic(uint8_t *uid) {
  Serial.println("📖 Reading MIFARE Classic block 4...");
  uint8_t data[16];

  // ✅ Authenticate block 4 with KEY_A
  if (nfc.mifareclassic_AuthenticateBlock(
          uid,           // UID pointer
          4,             // UID length
          4,             // block number
          0,             // KEY_A
          (uint8_t*)"\xFF\xFF\xFF\xFF\xFF\xFF"  // key
      )) {

      if (nfc.mifareclassic_ReadDataBlock(4, data)) {
          Serial.print("Data Block 4: ");
          for (int i = 0; i < 16; i++) Serial.write(data[i]);
          Serial.println();
      } else Serial.println("❌ Failed to read block 4!");
  } else Serial.println("❌ Auth failed!");
}

void readNTAG(uint8_t *uid) {
  Serial.println("📖 Reading NTAG page 4–7...");
  uint8_t data[4];
  for (uint8_t page = 4; page < 8; page++) {
    if (nfc.ntag2xx_ReadPage(page, data)) {
      Serial.print("Page "); Serial.print(page); Serial.print(": ");
      for (int i = 0; i < 4; i++) Serial.write(data[i]);
      Serial.println();
    } else Serial.println("❌ Read fail for page " + String(page));
  }
}
