#pragma once

#include <stddef.h>
#include <stdint.h>

namespace NfcStandaloneTest {

// Return codes aligned with existing kiosk NFC behavior.
enum ScanResult : int {
  NFC_NONE = 0,          // No target / no accepted event
  NFC_PHONE_CREDIT = 1,  // APDU phone-credit transaction accepted
  NFC_KNOWN_TOKEN = 2,   // UID hash recognized by token lookup
  NFC_UNKNOWN_TOKEN = 3  // UID read, but token lookup did not match
};

// Platform adapter. Implement this for your target platform/driver.
class Platform {
public:
  virtual ~Platform() {}

  // PN532 reset line control (HIGH = running, LOW = reset/disabled).
  virtual void setPn532Reset(bool high) = 0;
  virtual void delayMs(uint32_t ms) = 0;

  // Driver setup for one scan pass.
  virtual bool pn532Begin() = 0;
  virtual bool pn532SamConfig() = 0;

  // Low-level PN532 operations used by the kiosk flow.
  virtual bool inListPassiveTarget() = 0;
  virtual bool inDataExchange(const uint8_t* tx, uint8_t txLen,
                              uint8_t* rx, uint8_t* rxLen) = 0;
  virtual bool readPassiveTargetIdIso14443A(uint8_t* uid, uint8_t* uidLen,
                                            uint16_t timeoutMs) = 0;
};

// Optional token lookup callback for UID hash checks.
typedef bool (*TokenLookupFn)(uint32_t hash, void* user);

struct Config {
  // AID SELECT APDU (defaults to kiosk production value if nullptr/0).
  const uint8_t* selectApdu = nullptr;
  uint8_t selectApduLen = 0;

  // "creditsSpent" payload byte[3] = {1, creditUnits, apiVersion}
  uint8_t creditUnits = 1;
  uint8_t apiVersion = 1;

  // Timing behavior (matches kiosk defaults).
  uint32_t resetDelayMs = 100;
  uint32_t phoneHoldoffMs = 10000;
  uint32_t uidHoldoffMs = 5000;
  uint16_t uidReadTimeoutMs = 100;
};

class Reader {
public:
  Reader(Platform& platform, const Config& cfg = Config());

  void setEnabled(bool enabled);
  bool enabled() const { return enabled_; }

  // Optional token lookup callback.
  void setTokenLookup(TokenLookupFn fn, void* user = nullptr) {
    tokenLookup_ = fn;
    tokenLookupUser_ = user;
  }

  // Performs one scan cycle.
  // nowMs should be a monotonic millisecond timestamp from the host platform.
  ScanResult pollOnce(uint32_t nowMs, uint32_t& lastSeenHash);

  static uint32_t fnv1a32(const uint8_t* data, uint8_t len);

private:
  Platform& platform_;
  Config cfg_;
  bool enabled_ = false;

  uint32_t lastUsedTagHash_ = 0;
  uint32_t lastUidUsedAtMs_ = 0;
  uint32_t lastCreditTimeMs_ = 0;

  TokenLookupFn tokenLookup_ = nullptr;
  void* tokenLookupUser_ = nullptr;
};

} // namespace NfcStandaloneTest

