#include <Arduino.h>
#include <Adafruit_PN532.h>

#include "KioskIOpins.h" // canonical kiosk pin naming

// Optional hook (weak default). If your project provides its own implementation,
// it will override this.
__attribute__((weak)) bool isHashInEepromTokens(uint32_t) { return false; }

// -------------------- NFC Configuration (PN532) --------------------
// PN532 is connected via dedicated IRQ and RESET pins.
static Adafruit_PN532 nfc(KioskPins::PN532_IRQ, KioskPins::PN532_RESET);

// Simple versioning for the kiosk NFC protocol.
const uint8_t NFC_API_VERSION   = 1;
const uint8_t NFC_CREDIT_UNITS  = 1;

// AID SELECT APDU for the mobile APP credit protocol.
uint8_t NFC_SELECT_APDU[] = {
  0x00, 0xA4, 0x04, 0x00, 0x08,
  0x25, 0x2A, 0x59, 0xBF, 0x00,
  0x31, 0xE7, 0xC9, 0x00
};

const unsigned long NFCphoneCreditHoldoff = 10000; // 10 s
const unsigned long NFCtagUIDholdoff      = 5000;  // 5 s

uint32_t      lastUsedTagHash = 0;
unsigned long lastUidUsedAt   = 0;
unsigned long lastCreditTime  = 0;

int readNFC(uint32_t &lastSeenHash) {
  lastSeenHash = 0;

  // Bring PN532 out of reset, initialise for this scan.
  digitalWrite(KioskPins::PN532_RESET, HIGH);
  delay(100);

  nfc.begin();
  nfc.SAMConfig();

  uint8_t uid[7];
  uint8_t uidLen;

  if (!nfc.inListPassiveTarget()) {
    digitalWrite(KioskPins::PN532_RESET, LOW);
    return 0;
  }

  if ((millis() - lastCreditTime) >= NFCphoneCreditHoldoff) {
    uint8_t apduResponse[10];
    uint8_t apduResponseLength = sizeof(apduResponse);

    (void)nfc.inDataExchange(
      NFC_SELECT_APDU, sizeof(NFC_SELECT_APDU),
      apduResponse, &apduResponseLength
    );

    uint8_t idsResponse[128];
    uint8_t idsResponseLength = sizeof(idsResponse);

    uint8_t creditsSpent[3] = {1, NFC_CREDIT_UNITS, NFC_API_VERSION};

    bool creditSuccess = nfc.inDataExchange(
      creditsSpent, sizeof(creditsSpent),
      idsResponse, &idsResponseLength
    );

    if (creditSuccess) {
      lastCreditTime = millis();
      digitalWrite(KioskPins::PN532_RESET, LOW);
      return 1;
    }
  }

  if (!nfc.readPassiveTargetID(
        PN532_MIFARE_ISO14443A, uid, &uidLen, 300)) {
    digitalWrite(KioskPins::PN532_RESET, LOW);
    return 0;
  }

  uint32_t hash = 2166136261UL;
  for (uint8_t i = 0; i < uidLen; i++) {
    hash ^= uid[i];
    hash *= 16777619UL;
  }
  lastSeenHash = hash;

  if (hash == lastUsedTagHash &&
      (millis() - lastUidUsedAt < NFCtagUIDholdoff)) {
    digitalWrite(KioskPins::PN532_RESET, LOW);
    return 0;
  }

  lastUsedTagHash = hash;
  lastUidUsedAt   = millis();

  // Also check token hashes stored in EEPROM.
  if (isHashInEepromTokens(hash)) {
    digitalWrite(KioskPins::PN532_RESET, LOW);
    return 2;    // known EEPROM tag
  }

  digitalWrite(KioskPins::PN532_RESET, LOW);
  return 3;      // unknown tag
}
