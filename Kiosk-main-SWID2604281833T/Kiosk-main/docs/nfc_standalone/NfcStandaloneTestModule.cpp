#include "NfcStandaloneTestModule.h"

namespace NfcStandaloneTest {

namespace {

// Production kiosk AID SELECT APDU.
static const uint8_t kDefaultSelectApdu[] = {
  0x00, 0xA4, 0x04, 0x00, 0x08,
  0x25, 0x2A, 0x59, 0xBF, 0x00,
  0x31, 0xE7, 0xC9, 0x00
};

// Safe unsigned elapsed-time check for wrapping millisecond counters.
static inline uint32_t elapsedMs(uint32_t nowMs, uint32_t thenMs) {
  return nowMs - thenMs;
}

} // namespace

Reader::Reader(Platform& platform, const Config& cfg)
  : platform_(platform), cfg_(cfg) {
  if (!cfg_.selectApdu || cfg_.selectApduLen == 0) {
    cfg_.selectApdu = kDefaultSelectApdu;
    cfg_.selectApduLen = static_cast<uint8_t>(sizeof(kDefaultSelectApdu));
  }
}

void Reader::setEnabled(bool enabled) {
  if (enabled_ == enabled) return;
  enabled_ = enabled;
  if (!enabled_) {
    platform_.setPn532Reset(false);
  }
}

ScanResult Reader::pollOnce(uint32_t nowMs, uint32_t& lastSeenHash) {
  lastSeenHash = 0;
  if (!enabled_) return NFC_NONE;

  platform_.setPn532Reset(true);
  platform_.delayMs(cfg_.resetDelayMs);

  if (!platform_.pn532Begin()) {
    platform_.setPn532Reset(false);
    return NFC_NONE;
  }
  if (!platform_.pn532SamConfig()) {
    platform_.setPn532Reset(false);
    return NFC_NONE;
  }

  struct ResetGuard {
    Platform& p;
    ~ResetGuard() { p.setPn532Reset(false); }
  } resetGuard{platform_};

  // Need a passive target before APDU/UID operations.
  if (!platform_.inListPassiveTarget()) return NFC_NONE;

  // 1) APDU phone-credit path first.
  if (elapsedMs(nowMs, lastCreditTimeMs_) >= cfg_.phoneHoldoffMs) {
    uint8_t selectResp[16] = {0};
    uint8_t selectRespLen = static_cast<uint8_t>(sizeof(selectResp));
    (void)platform_.inDataExchange(cfg_.selectApdu, cfg_.selectApduLen, selectResp, &selectRespLen);

    uint8_t creditsSpent[3] = {1, cfg_.creditUnits, cfg_.apiVersion};
    uint8_t creditResp[128] = {0};
    uint8_t creditRespLen = static_cast<uint8_t>(sizeof(creditResp));

    const bool creditSuccess = platform_.inDataExchange(
      creditsSpent, static_cast<uint8_t>(sizeof(creditsSpent)),
      creditResp, &creditRespLen
    );

    if (creditSuccess) {
      lastCreditTimeMs_ = nowMs;
      return NFC_PHONE_CREDIT;
    }
  }

  // 2) UID fallback path.
  uint8_t uid[7] = {0};
  uint8_t uidLen = 0;
  if (!platform_.readPassiveTargetIdIso14443A(uid, &uidLen, cfg_.uidReadTimeoutMs)) {
    return NFC_NONE;
  }

  const uint32_t hash = fnv1a32(uid, uidLen);
  lastSeenHash = hash;

  // 3) UID anti-repeat holdoff.
  if (hash == lastUsedTagHash_ && elapsedMs(nowMs, lastUidUsedAtMs_) < cfg_.uidHoldoffMs) {
    return NFC_NONE;
  }
  lastUsedTagHash_ = hash;
  lastUidUsedAtMs_ = nowMs;

  // 4) Token lookup (optional callback).
  if (tokenLookup_ && tokenLookup_(hash, tokenLookupUser_)) {
    return NFC_KNOWN_TOKEN;
  }
  return NFC_UNKNOWN_TOKEN;
}

uint32_t Reader::fnv1a32(const uint8_t* data, uint8_t len) {
  uint32_t h = 2166136261UL;
  for (uint8_t i = 0; i < len; ++i) {
    h ^= data[i];
    h *= 16777619UL;
  }
  return h;
}

} // namespace NfcStandaloneTest

