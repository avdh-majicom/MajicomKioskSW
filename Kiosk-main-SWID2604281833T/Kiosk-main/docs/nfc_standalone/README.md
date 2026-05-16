# NFC Standalone Test Module

Files:
- `docs/nfc_standalone/NfcStandaloneTestModule.h`
- `docs/nfc_standalone/NfcStandaloneTestModule.cpp`
- `docs/nfc_standalone/ArduinoPn532SerialTest.ino`

Purpose:
- Extracts the kiosk NFC scan core from `KioskIO::readNfc()` into a platform-agnostic module.
- Keeps the same behavior:
  1. Try APDU phone-credit exchange first.
  2. Fallback to UID read + FNV-1a hash.
  3. UID anti-repeat holdoff.
  4. Optional token-hash lookup.

Return codes (same semantics as kiosk):
- `0` = no accepted event
- `1` = phone/APDU credit success
- `2` = known token (hash found)
- `3` = unknown token

## Arduino serial test sketch

Use `docs/nfc_standalone/ArduinoPn532SerialTest.ino` as a simple monitor test:
- Prints startup intro text.
- Probes and prints PN532 firmware version at boot.
- Configured for Arduino UNO + PN532 over I2C and `115200` baud serial.
- Uses no PN532 IRQ/RESET wiring in this variant (I2C only).
- Prints only non-zero NFC events:
  - `1` => phone/APDU credit success line
  - `2` and `3` => same handling, prints only token UID hash

Expected serial examples:
- `NFC event: PHONE/APDU credit success`
- `NFC event: token UID hash=0xA1B2C3D4`

## Integration steps

1. Implement `NfcStandaloneTest::Platform` on your target.
2. Create `NfcStandaloneTest::Reader reader(platform, cfg);`
3. Call `reader.setEnabled(true);`
4. In your loop, call:
   - `uint32_t hash = 0;`
   - `auto res = reader.pollOnce(nowMs, hash);`

## Minimal usage sketch (pseudocode)

```cpp
#include "NfcStandaloneTestModule.h"
using namespace NfcStandaloneTest;

class MyPlatform : public Platform {
public:
  void setPn532Reset(bool high) override { /* gpio write */ }
  void delayMs(uint32_t ms) override { /* sleep */ }
  bool pn532Begin() override { /* driver begin */ return true; }
  bool pn532SamConfig() override { /* SAMConfig */ return true; }
  bool inListPassiveTarget() override { /* PN532 poll */ return true; }
  bool inDataExchange(const uint8_t* tx, uint8_t txLen, uint8_t* rx, uint8_t* rxLen) override {
    /* PN532 InDataExchange */
    return false;
  }
  bool readPassiveTargetIdIso14443A(uint8_t* uid, uint8_t* uidLen, uint16_t timeoutMs) override {
    /* PN532 readPassiveTargetID */
    return false;
  }
};

static bool tokenLookup(uint32_t hash, void* user) {
  (void)user;
  // e.g. lookup in local table/db
  return hash == 0x12345678UL;
}

int main() {
  MyPlatform platform;
  Config cfg; // defaults match kiosk values/APDU
  Reader reader(platform, cfg);
  reader.setTokenLookup(tokenLookup, nullptr);
  reader.setEnabled(true);

  for (;;) {
    uint32_t hash = 0;
    uint32_t nowMs = /* monotonic ms */;
    ScanResult r = reader.pollOnce(nowMs, hash);
    // handle r / hash
  }
}
```
