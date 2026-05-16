/*
  KioskEeprom.h
  ------------
  Shared EEPROM access module used by both the production kiosk firmware and the EEPROM editor.

  Design note (structure, not behaviour):
  - This file and KioskEeprom.cpp form the "shared" EEPROM library that other firmware modules can reuse.
  - The EEPROM editor (KioskEepromEditor.*) calls into this module for all reads/writes.
  - EEPROM layout is documented below and matches the editor's assumptions.

  Behaviour matches the baseline sketch unless explicitly noted; comments have been enhanced for readability.
  Note: default MeasuredProfile duration uses ms->100ms scaling to match duration100ms storage units.
*/

#pragma once
#include <Arduino.h>
#include <EEPROM.h>

namespace Kiosk {

class KioskEeprom {
public:
  // EEPROM layout details (addresses, magic values, register defaults) are kept private
  // in KioskEepromLayout.h and used by KioskEeprom.cpp.
  //
  // Public contract:
  // - Other modules obtain EEPROM-stored values via getters (no required RAM shadow).
  // - Runtime firmware writes are intentionally limited to counters and measured-dispense
  //   calibration values; broad parameter writes remain editor-oriented APIs.

  static constexpr uint8_t  PWM_SOLENOID_COUNT     = 4;
  static constexpr uint16_t EEPROM_TOKEN_MAX       = 256;
  static constexpr uint32_t EEPROM_TOKEN_EMPTY     = 0x00000000UL;
  static constexpr uint8_t  DISPENSE_COUNTER_COUNT = 4;



  // NFC reader timing parameters (API uses milliseconds).
  // These limits/defaults are used by both the EEPROM implementation (for clamping)
  // and the EEPROM editor UI (for validation and step sizes). They are intentionally
  // defined here (public API) so no other module must include KioskEepromLayout.h.
  static constexpr uint32_t NFC_INIT_DELAY_DEFAULT_MS   = 100UL;
  static constexpr uint32_t NFC_INIT_DELAY_MIN_MS       = 10UL;
  static constexpr uint32_t NFC_INIT_DELAY_MAX_MS       = 250UL;
  static constexpr uint32_t NFC_INIT_DELAY_STEP_MS      = 1UL;

  static constexpr uint32_t NFC_SCAN_DURATION_DEFAULT_MS = 150UL;
  static constexpr uint32_t NFC_SCAN_DURATION_MIN_MS     = 10UL;
  static constexpr uint32_t NFC_SCAN_DURATION_MAX_MS     = 2500UL;
  static constexpr uint32_t NFC_SCAN_DURATION_STEP_MS    = 10UL; // stored in 10ms units

  static constexpr uint32_t NFC_INTER_DELAY_DEFAULT_MS  = 50UL;
  static constexpr uint32_t NFC_INTER_DELAY_MIN_MS      = 10UL;
  static constexpr uint32_t NFC_INTER_DELAY_MAX_MS      = 250UL;
  static constexpr uint32_t NFC_INTER_DELAY_STEP_MS     = 1UL;

  // Measured-dispense modeSel
  enum WdMode : uint8_t { WD_MODE_TIME = 0, WD_MODE_PULSES = 1 };

  // Solenoid indices match your menu order:
  // 0 BackWashSol, 1 WaterSenseBypSol, 2 InletWaterSol, 3 WaterDispSol
  enum Solenoid : uint8_t { SOL_BACKWASH = 0, SOL_BYPASS = 1, SOL_INLET = 2, SOL_DISPENSE = 3 };

  enum DispenseCounter : uint8_t { DISP_APP = 0, DISP_COIN = 1, DISP_NFC = 2, DISP_BYPASS = 3 };

  struct __attribute__((packed)) MeasuredProfile {
    // NOTE: __attribute__((packed)) is intentional: the on-wire/on-EEPROM layout must remain stable.
    //       If you change this struct, you are changing the EEPROM format.
    // Stored in EEPROM as compact uint16 values:
    // - duration100ms: 100ms increments (0.1s per LSB).
    //   Helper dispMeasuredDurationMs() converts to milliseconds: duration100ms * 100.
    //   Example: 123 -> 12.3 seconds.
    // - pulses: raw pulse count (unitless).
    uint16_t duration100ms;
    uint16_t pulses;
    uint8_t  modeSel;
  };

  struct __attribute__((packed)) PwmSolenoidProfile {
    uint8_t startPwm;
    uint8_t holdPwm;
    uint8_t swDelaySec;
    // Legacy slots kept for EEPROM address compatibility (not used).
    uint8_t reserved0;
    int8_t  reserved1;
  };

  KioskEeprom() = default;

  // Computes token table base from EEPROM.length(). Call once early at boot.
  bool begin();

  bool isReady() const { return _ready; }
  int  tokenBaseAddr() const { return _tokenBaseAddr; }

  // Magic
  bool magicValid() const;
  void writeMagic();

  // Defaults + full init

  // Units glossary:
  // - duration100ms: stored_value * 100ms (0.1s/LSB)
  // - UV_OK_DELAY_10MS: stored_value * 10ms
  // - BACKWASH_DUR: stored_value * 5 seconds
  // - DAILY_BACKWASH_TIME_MIN: minutes since midnight (0..1439)
  // - DAILY_BACKWASH_DUR_10S: stored_value * 10 seconds (0 disables)
  // - *_Sec: seconds
  // - *_Min: minutes
  void reinitializeToDefaults(bool clearAllEepromBytes = true);
  void ensureMagicOrInitDefaults();

  // -------------------- Read APIs for main firmware --------------------
  uint8_t hwId() const;
  MeasuredProfile dispMeasuredProfile() const;

  // Individual MeasuredProfile field accessors (read directly from EEPROM).
  uint16_t dispMeasuredDuration100ms() const;   // raw stored units (100ms increments)
  uint16_t dispMeasuredPulses() const;          // raw stored pulses
  uint8_t  dispMeasuredModeSel() const;         // raw stored modeSel (WdMode)


  // -------------------- Time conversion helpers (always return milliseconds) --------------------
  // These helpers convert compact EEPROM storage units into milliseconds for the main firmware.
  // They do not change what is stored in EEPROM; they only convert on read.
  uint32_t dispMeasuredDurationMs() const;


  // Backward-compatible names (deprecated): use dispMeasured*()
  // The layout now stores a single measured-dispense profile at ADDR_DISP_PROFILE.
  // For legacy callers that expect an indexed table, only index 0 is valid.
  inline bool measuredProfile(uint8_t idx, MeasuredProfile& out) const {
    if (idx != 0) return false;
    out = dispMeasuredProfile();
    return true;
  }
  inline uint16_t measuredDuration100ms() const { return dispMeasuredDuration100ms(); }
  inline uint16_t measuredPulses() const { return dispMeasuredPulses(); }
  inline uint8_t measuredModeSel() const { return dispMeasuredModeSel(); }
  inline uint32_t measuredDurationMs() const { return dispMeasuredDurationMs(); }
  uint32_t backwashDurationMs() const;          // backwashDuration (5s units) -> ms
  uint32_t dailyBackwashDurationMs() const;     // dailyBackwashDuration (10s units) -> ms
  uint32_t klaranUvOkDelayMs() const;           // klaranUvOkDelay10ms * 10
  uint32_t klaranUvMaxOntimeMs() const;         // minutes -> ms
  uint32_t solenoidSwDelayMs(Solenoid s) const; // swDelaySec -> ms

  PwmSolenoidProfile solenoidProfile(Solenoid s) const;

  uint8_t backwashDuration() const;
  uint8_t backwashAfterNDispenses() const;
  // Stores the TOTAL dispensed units value the last time a backwash was run.
  uint16_t backwashDispenseCounter() const;
  uint8_t backwashManualShortSeconds() const;      // 1-second units
  uint8_t backwashManualLong2Minutes() const;      // 2-minute units
  // Daily backwash scheduling (RTC-required at runtime).
  uint16_t dailyBackwashTimeMinutes() const;       // minutes since midnight
  uint8_t dailyBackwashDuration10s() const;        // 10-second units (0 disables)
  uint8_t sensorBypassDuration100ms() const;       // 100ms units
  uint8_t sensorBypassPeriodMinutes() const;       // minutes
  uint8_t sensorBypassManualShort100ms() const;    // 100ms units
  uint8_t sensorBypassManualLong10s() const;       // 10-second units
  uint8_t preDispCircTSeconds() const;             // 1 LSB = 1 second
  uint8_t preDispCircP100Pulses() const;           // 1 LSB = 100 pulses
  uint8_t preDispPurgTSeconds() const;             // 1 LSB = 1 second
  uint8_t preDispPurgP100Pulses() const;           // 1 LSB = 100 pulses
  uint8_t autoCircDurationMinutes() const;         // 1 minute units (0 disables)
  uint8_t autoCircPeriod10Minutes() const;         // 10 minute units (0 disables)
  uint8_t manCircDurationShortMinutes() const;     // 1 minute units (0 disables)
  uint8_t manCircDurationLong10Minutes() const;    // 10 minute units (0 disables)
  uint8_t dnfReptPeriodMinutes() const;            // 1 minute units
  uint8_t dnfReptDurat100ms() const;               // 100ms units
  uint8_t preFlshBeepDelySeconds() const;          // 1-second units
  uint8_t beepOnTime10ms() const;                  // 10ms units
  uint8_t beepOffTime100ms() const;                // 100ms units
  uint8_t beepCount() const;                       // 1-count units
  // End-of-day durations are currently fixed defaults used by hydraulics.
  uint8_t eodCircDuration15s() const;              // 15-second units
  uint8_t eodBackwashDuration1s() const;           // 1-second units

  // KlaranUV configuration (layout v2)
  // - UV OK Delay: delay before treating the UV_OK input as valid, stored in 10ms increments.
  // - UV MAX ONtime: hard safety limit for how long the UV output may remain enabled.
  uint8_t klaranUvOkDelay10ms() const;
  uint8_t klaranUvMaxOntimeMinutes() const;

  // Water temperature sensing configuration (layout v3)
  // Select which DS18B20 (T1 or T2) is used as the water temperature sensor.
  // Stored as raw uint8_t:
  //   1 = T1 (default)
  //   2 = T2
  uint8_t waterTempSense() const;
  bool    coinAcceptorFitted() const;

  // NFC reader timing parameters (stored in EEPROM; API uses milliseconds).
  uint32_t nfcInitDelayMs() const;
  uint32_t nfcReadDurationMs() const;
  uint32_t nfcInterNfcDelayMs() const;

  uint32_t dispenseCounter(DispenseCounter idx) const;
  uint32_t totalDispensedUnits() const;

  // NFC token table read-only scan
  uint32_t tokenHashAt(uint16_t idx) const;
  uint16_t findTokenHash(uint32_t hash) const;
  bool     tokenHashExists(uint32_t hash) const;
  // Alias for tokenHashExists() (requested API name).
  bool     testTokenHashPresent(uint32_t hash) const;

  // Packs the token table in-place.
  //  1) De-duplicates: scans from index 0 upward and zeros the 2nd and
  //     subsequent occurrences of any token hash.
  //  2) Compacts: scans from index 0 upward; when an empty slot is found, it is
  //     filled with the furthest (highest-index) active token in the table.
  //
  // Returns the number of table modifications performed (duplicates cleared +
  // tokens moved), or 0 if EEPROM not ready.
  uint16_t packTokenHashTable();
  uint16_t countActiveTokens() const;

  // -------------------- Write APIs (limited for main firmware) --------------------
  // Intended runtime writes: counters + measured dispense calibration fields.
  void setDispenseCounter(DispenseCounter idx, uint32_t value);
  uint32_t incrementDispenseCounter(DispenseCounter idx, uint32_t delta = 1);

  void setMeasuredDurationMs(uint32_t ms);
  void setMeasuredPulses(uint32_t pulses);
  void setMeasuredMode(uint8_t modeSel);

  // -------------------- Editor-only write APIs --------------------
  void setHwId(uint8_t id);
  void storeMeasuredProfile(const MeasuredProfile& p);
  void storeSolenoidProfile(Solenoid s, const PwmSolenoidProfile& p);
  void setBackwashDuration(uint8_t v);
  void setBackwashAfterNDispenses(uint8_t v);
  void setBackwashDispenseCounter(uint16_t v);
  void setBackwashManualShortSeconds(uint8_t seconds);
  void setBackwashManualLong2Minutes(uint8_t units2min);
  void setDailyBackwashTimeMinutes(uint16_t minutes);
  void setDailyBackwashDuration10s(uint8_t units10s);
  void setSensorBypassDuration100ms(uint8_t units100ms);
  void setSensorBypassPeriodMinutes(uint8_t minutes);
  void setSensorBypassManualShort100ms(uint8_t units100ms);
  void setSensorBypassManualLong10s(uint8_t units10s);
  void setPreDispCircTSeconds(uint8_t seconds);
  void setPreDispCircP100Pulses(uint8_t units100p);
  void setPreDispPurgTSeconds(uint8_t seconds);
  void setPreDispPurgP100Pulses(uint8_t units100p);
  void setAutoCircDurationMinutes(uint8_t minutes);
  void setAutoCircPeriod10Minutes(uint8_t units10min);
  void setManCircDurationShortMinutes(uint8_t minutes);
  void setManCircDurationLong10Minutes(uint8_t units10min);
  void setDnfReptPeriodMinutes(uint8_t minutes);
  void setDnfReptDurat100ms(uint8_t units100ms);
  void setPreFlshBeepDelySeconds(uint8_t seconds);
  void setBeepOnTime10ms(uint8_t units10ms);
  void setBeepOffTime100ms(uint8_t units100ms);
  void setBeepCount(uint8_t count);

  void setKlaranUvOkDelay10ms(uint8_t v);
  void setKlaranUvMaxOntimeMinutes(uint8_t v);

  void setWaterTempSense(uint8_t v);
  void setCoinAcceptorFitted(bool fitted);

  void setNfcInitDelayMs(uint32_t ms);
  void setNfcReadDurationMs(uint32_t ms);
  void setNfcInterNfcDelayMs(uint32_t ms);

  bool addTokenHash(uint32_t hash);      // returns false if duplicate/full/invalid
  bool deleteTokenHash(uint32_t hash);   // returns false if not found/invalid
  void removeAllTokens();
  void writeDefaultTokenHashes();

  // Convenience: find solenoid index by ASCII name (matches strings used in your UI)
  static bool solenoidFromName(const char* name, Solenoid& out);

private:
  int  _tokenBaseAddr = 0;
  bool _ready = false;

  uint16_t findEmptyTokenSlot() const;

  static MeasuredProfile defaultMeasuredProfile();
  static PwmSolenoidProfile defaultSolenoidProfile(uint8_t idx);

};

} // namespace Kiosk
