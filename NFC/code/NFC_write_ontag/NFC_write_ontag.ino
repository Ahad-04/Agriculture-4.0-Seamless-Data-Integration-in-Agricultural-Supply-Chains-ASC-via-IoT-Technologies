#include <Wire.h>
#include <Adafruit_PN532.h>

#define PN532_IRQ   (2)
#define PN532_RESET (3)

Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

void setup() {
  Serial.begin(115200);
  Serial.println("=== Mifare Classic Writer - Write 'ChipVista' to Block 8 ===");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("❌ Didn't find PN53x board. Check wiring!");
    while (1);
  }

  nfc.SAMConfig();
  Serial.println("✅ PN532 ready. Waiting for an NFC tag...");
}

void loop() {
  uint8_t success;
  uint8_t uid[7];      // Buffer to store UID
  uint8_t uidLength;   // UID length (4 or 7 bytes)

  // Look for a card
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    Serial.print("🔹 Found tag UID: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      Serial.print(uid[i], HEX); Serial.print(" ");
    }
    Serial.println();

    // Default key A (factory default)
    uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

    // Authenticate block 8 (Sector 2, first data block)
    success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 8, 0, keya);

    if (success) {
      Serial.println("✅ Authentication successful. Writing data...");

      // Prepare 16 bytes data (exact length)
      uint8_t data[16] = {
        'C','h','i','p','V','i','s','t','a',' ',' ',' ',' ',' ',' ',' '};

      // Write data to block 8
      success = nfc.mifareclassic_WriteDataBlock(8, data);

      if (success) {
        Serial.println("✅ Successfully wrote 'ChipVista' to block 8!");
      } else {
        Serial.println("❌ Failed to write to block!");
      }
    } else {
      Serial.println("❌ Authentication failed!");
    }

    Serial.println("Remove tag...");
    delay(3000);
  }
}
