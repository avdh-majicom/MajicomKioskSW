#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

#include "NfcStandaloneTestModule.h"

// Arduino UNO + PN532 over I2C:
// - SDA=A4, SCL=A5 (hardware I2C)
// - For older Adafruit_PN532 versions, the I2C constructor still requires
//   (irq, reset) arguments. They can be dummy values when not physically wired.
static constexpr uint8_t PN532_IRQ_PIN = 2;
static constexpr uint8_t PN532_RST_PIN = 3;
static Adafruit_PN532 g_nfc(PN532_IRQ_PIN, PN532_RST_PIN);

class ArduinoPn532Platform : public NfcStandaloneTest::Platform {
public:
  void setPn532Reset(bool high) override {
    // No hardware reset line connected in this variant.
    (void)high;
  }

  void delayMs(uint32_t ms) override {
    delay(ms);
  }

  bool pn532Begin() override {
    g_nfc.begin();
    return true;
  }

  bool pn532SamConfig() override {
    // Adafruit API returns bool on newer versions, void on some older versions.
    // Cast to void-safe by always returning true after calling it.
    g_nfc.SAMConfig();
    return true;
  }

  bool inListPassiveTarget() override {
    return g_nfc.inListPassiveTarget();
  }

  bool inDataExchange(const uint8_t* tx, uint8_t txLen, uint8_t* rx, uint8_t* rxLen) override {
    // Adafruit API expects non-const tx pointer.
    return g_nfc.inDataExchange(const_cast<uint8_t*>(tx), txLen, rx, rxLen);
  }

  bool readPassiveTargetIdIso14443A(uint8_t* uid, uint8_t* uidLen, uint16_t timeoutMs) override {
    return g_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, uidLen, timeoutMs);
  }
};

static ArduinoPn532Platform g_platform;
static NfcStandaloneTest::Config makeConfig() {
  NfcStandaloneTest::Config c;
  // Allow same phone app or same token to retrigger after 1 second.
  c.phoneHoldoffMs = 1000;
  c.uidHoldoffMs = 1000;
  return c;
}
static NfcStandaloneTest::Config g_cfg = makeConfig();
static NfcStandaloneTest::Reader g_reader(g_platform, g_cfg);

static void printHex32(uint32_t v) {
  char buf[11];
  snprintf(buf, sizeof(buf), "0x%08lX", (unsigned long)v);
  Serial.print(buf);
}

static void printFirmwareVersion() {
  // Probe PN532 over I2C.
  g_nfc.begin();
  uint32_t ver = g_nfc.getFirmwareVersion();

  if (!ver) {
    Serial.println(F("PN532 firmware check: FAILED (no response)"));
    return;
  }

  uint8_t chip = (uint8_t)(ver >> 24);
  uint8_t major = (uint8_t)(ver >> 16);
  uint8_t minor = (uint8_t)(ver >> 8);
  uint8_t support = (uint8_t)(ver);

  Serial.print(F("PN532 firmware: chip=0x"));
  Serial.print(chip, HEX);
  Serial.print(F(" ver="));
  Serial.print(major);
  Serial.print(F("."));
  Serial.print(minor);
  Serial.print(F(" support=0x"));
  Serial.println(support, HEX);
}

void setup() {
  Wire.begin();

  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  Serial.println(F("=== NFC Standalone APDU Test ==="));
  Serial.println(F("Flow: APDU phone-credit first, then UID hash fallback"));
  printFirmwareVersion();

  g_reader.setEnabled(true);
}

void loop() {
  uint32_t hash = 0;
  NfcStandaloneTest::ScanResult r = g_reader.pollOnce(millis(), hash);

  if (r == NfcStandaloneTest::NFC_PHONE_CREDIT) {
    Serial.println(F("NFC event: PHONE/APDU credit success"));
  } else if (r == NfcStandaloneTest::NFC_KNOWN_TOKEN ||
             r == NfcStandaloneTest::NFC_UNKNOWN_TOKEN) {
    Serial.print(F("NFC event: token UID hash="));
    printHex32(hash);
    Serial.println();
  }

  delay(50);
}
