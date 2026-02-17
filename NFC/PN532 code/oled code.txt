#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PN532.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  // -1 means no reset pin

// PN532 NFC Module settings
#define PN532_IRQ   2
#define PN532_RESET 3
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

void setup(void) {
  // Initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial); // wait for serial port to open
  
  // Initialize OLED Display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C is a common I2C address for OLEDs
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(1000);  // Pause for 1 second

  display.clearDisplay();
  display.setTextSize(1);      
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Initializing NFC...");
  display.display();

  // Initialize PN532 NFC Module
  nfc.begin();
  
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    display.clearDisplay();
    display.println("Didn't find PN53x module");
    display.display();
    while (1); // halt
  }

  // Display firmware version
  display.clearDisplay();
  display.println("Found PN532");
  display.print("Firmware Version: ");
  display.print((versiondata >> 16) & 0xFF, HEX);
  display.print('.');
  display.println((versiondata >> 8) & 0xFF, HEX);
  display.display();

  // Configure NFC to read RFID cards
  nfc.SAMConfig();

  delay(1000);
  display.clearDisplay();
  display.println("Waiting for NFC card...");
  display.display();
}

void loop(void) {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;  // Length of the UID (4 or 7 bytes depending on card type)

  // Look for an NFC card
  success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

  if (success) {
    // NFC card found!
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("NFC card detected!");
    display.print("UID Length: "); display.print(uidLength, DEC); display.println(" bytes");
    display.print("UID: ");
    for (uint8_t i=0; i < uidLength; i++) {
      display.print(" 0x"); display.print(uid[i], HEX);
    }
    display.display();
    
    delay(2000); // Pause for 2 seconds before scanning again
    display.clearDisplay();
    display.println("Waiting for NFC card...");
    display.display();
  }
}
