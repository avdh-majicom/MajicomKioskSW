/*
  KioskEeprom.cpp
  --------------
  Implementation of the shared EEPROM access module.

  Refactor note (structure, not behaviour):
  - This module owns EEPROM layout and defaulting rules (magic validation, default profiles, token table).
  - The editor and the "real" firmware must both use this module to avoid layout drift.

  Behaviour matches the baseline sketch unless explicitly noted; comments have been enhanced for readability.
  Note: default MeasuredProfile duration uses ms->100ms scaling to match duration100ms storage units.
*/

#include "KioskEeprom.h"
#include "KioskEepromLayout.h"
#include "KioskIO.h"

// Ensure the documented EEPROM schema matches the compiled struct packing.
static_assert(sizeof(Kiosk::KioskEeprom::PwmSolenoidProfile) == Kiosk::EepromLayout::SOL_PROFILE_SIZE_BYTES,
              "PwmSolenoidProfile size changed; update EEPROM layout constants/migration.");
#include <string.h>

namespace Kiosk {

// Required for older AVR/Arduino toolchains when constexpr is ODR-used (e.g., via EEPROM.put const&)
constexpr uint32_t KioskEeprom::EEPROM_TOKEN_EMPTY;

bool KioskEeprom::begin() {
  const int len = EEPROM.length();
  const int tokenBytes = (int)(EEPROM_TOKEN_MAX * (int)sizeof(uint32_t));
  if (len <= tokenBytes) {
    _ready = false;
    _tokenBaseAddr = 0;
    return false;
  }
  _tokenBaseAddr = len - tokenBytes;
  _ready = true;
  return true;
}

bool KioskEeprom::magicValid() const {
  uint32_t magic = 0;
  EEPROM.get(EepromLayout::ADDR_MAGIC, magic);
  return (magic == EepromLayout::MAGIC_VALUE);
}

void KioskEeprom::writeMagic() {
  EEPROM.put(EepromLayout::ADDR_MAGIC, (uint32_t)EepromLayout::MAGIC_VALUE);
}

void KioskEeprom::ensureMagicOrInitDefaults() {
  // Migration strategy:
  // - If magic matches current layout, do nothing.
  // - If magic matches a known prior layout, apply targeted migrations/defaults.
  // - If magic is unknown/invalid, perform full default reinitialization.
  uint32_t magic = 0;
  EEPROM.get(EepromLayout::ADDR_MAGIC, magic);
  auto applyPeriodicFlushDefaults = []() {
    EEPROM.update(EepromLayout::ADDR_DNF_REPT_PERIOD_MIN, EepromLayout::DEFAULT_DNF_REPT_PERIOD_MIN);
    EEPROM.update(EepromLayout::ADDR_DNF_REPT_DURAT_100MS, EepromLayout::DEFAULT_DNF_REPT_DURAT_100MS);
    EEPROM.update(EepromLayout::ADDR_PREFLSH_BEEP_DELY_SEC, EepromLayout::DEFAULT_PREFLSH_BEEP_DELY_SEC);
    EEPROM.update(EepromLayout::ADDR_BEEP_ON_TIME_10MS, EepromLayout::DEFAULT_BEEP_ON_TIME_10MS);
    EEPROM.update(EepromLayout::ADDR_BEEP_OFF_TIME_100MS, EepromLayout::DEFAULT_BEEP_OFF_TIME_100MS);
    EEPROM.update(EepromLayout::ADDR_BEEP_COUNT, EepromLayout::DEFAULT_BEEP_COUNT);
  };
  auto migrateBackwashCounterU32ToU16 = []() {
    uint32_t oldCounter32 = 0;
    EEPROM.get(EepromLayout::ADDR_BACKWASH_COUNTER, oldCounter32);
    const uint16_t newCounter16 =
      (oldCounter32 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)oldCounter32;
    EEPROM.put(EepromLayout::ADDR_BACKWASH_COUNTER, newCounter16);
    // Clear legacy high bytes from the old uint32 representation.
    EEPROM.update(EepromLayout::ADDR_BACKWASH_COUNTER + 2, 0x00);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_COUNTER + 3, 0x00);
  };

  // Already on current layout; nothing to migrate.
  if (magic == EepromLayout::MAGIC_VALUE) return;

  // Layout versions MJC2..MJCC used uint32_t at ADDR_BACKWASH_COUNTER.
  // Migrate once up front so every version-path below lands on uint16_t semantics.
  if (magic == EepromLayout::MAGIC_VALUE_V12 ||
      magic == EepromLayout::MAGIC_VALUE_V11 ||
      magic == EepromLayout::MAGIC_VALUE_V10 ||
      magic == EepromLayout::MAGIC_VALUE_V9  ||
      magic == EepromLayout::MAGIC_VALUE_V8  ||
      magic == EepromLayout::MAGIC_VALUE_V7  ||
      magic == EepromLayout::MAGIC_VALUE_V6  ||
      magic == EepromLayout::MAGIC_VALUE_V5  ||
      magic == EepromLayout::MAGIC_VALUE_V4  ||
      magic == EepromLayout::MAGIC_VALUE_V3  ||
      magic == EepromLayout::MAGIC_VALUE_V2) {
    migrateBackwashCounterU32ToU16();
  }

  // Prior layout (MJCC) -> only magic bump after counter-width migration above.
  if (magic == EepromLayout::MAGIC_VALUE_V12) {
    writeMagic();
    return;
  }

  // Prior layout (MJCB) -> update Beep ON unit/default.
  if (magic == EepromLayout::MAGIC_VALUE_V11) {
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJCA) -> update periodic flush fields/units.
  if (magic == EepromLayout::MAGIC_VALUE_V10) {
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC9) -> add periodic flush defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V9) {
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC8) -> add manual sensor bypass duration defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V8) {
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC7) -> add manual backwash duration defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V7) {
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC6) -> add pre-dispense dispense parms defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V6) {
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC5) -> add water circulation control defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V5) {
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC4) -> add reserved byte at addr 53 (historically Disp Avg Count).
  if (magic == EepromLayout::MAGIC_VALUE_V4) {
    EEPROM.update(EepromLayout::ADDR_TIMED_DISP_UNUSED, 0);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC3) -> add sensor bypass timing defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V3) {
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_DUR_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_DUR_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_PERIOD_MIN, EepromLayout::DEFAULT_SENSOR_BYPASS_PERIOD_MIN);
    EEPROM.update(EepromLayout::ADDR_TIMED_DISP_UNUSED, 0);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Prior layout (MJC2) -> add daily backwash scheduling defaults.
  if (magic == EepromLayout::MAGIC_VALUE_V2) {
    EEPROM.put(EepromLayout::ADDR_DAILY_BACKWASH_TIME_MIN, (uint16_t)EepromLayout::DEFAULT_DAILY_BACKWASH_TIME_MIN);
    EEPROM.update(EepromLayout::ADDR_DAILY_BACKWASH_DUR_10S, EepromLayout::DEFAULT_DAILY_BACKWASH_DUR_10S);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_DUR_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_DUR_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_PERIOD_MIN, EepromLayout::DEFAULT_SENSOR_BYPASS_PERIOD_MIN);
    EEPROM.update(EepromLayout::ADDR_TIMED_DISP_UNUSED, 0);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    applyPeriodicFlushDefaults();
    writeMagic();
    return;
  }

  // Legacy layout migration: backwash counter widened from uint8_t to uint16_t at addr 33.
  // Prior layouts stored:
  //  - backwash counter as uint8_t at addr 33
  //  - UV OK delay at addr 34 (uint8_t)
  //  - UV MAX ontime at addr 35 (uint8_t)
  //  - WaterTempSense at addr 36 (uint8_t)
  // Current layout stores backwash counter as uint16_t at 33..34 and keeps shifted
  // uint8_t fields at 37..39 (bytes 35..36 reserved).
  if (magic == EepromLayout::MAGIC_VALUE_LEGACY) {
    const uint8_t oldBwCounterU8 = EEPROM.read(EepromLayout::ADDR_BACKWASH_COUNTER);
    const uint8_t oldUvOkDelay10ms = EEPROM.read(34);
    const uint8_t oldUvMaxOntimeMin = EEPROM.read(35);
    uint8_t oldWaterTempSense = EEPROM.read(36);
    if (oldWaterTempSense < 1 || oldWaterTempSense > 2) oldWaterTempSense = 1;

    // Write widened backwash counter (bytes 33..34).
    EEPROM.put(EepromLayout::ADDR_BACKWASH_COUNTER, (uint16_t)oldBwCounterU8);
    EEPROM.update(35, 0x00);
    EEPROM.update(36, 0x00);

    // Re-home shifted legacy bytes.
    EEPROM.update(EepromLayout::ADDR_UV_OK_DELAY_10MS, oldUvOkDelay10ms);
    EEPROM.update(EepromLayout::ADDR_UV_MAX_ONTIME_MIN, oldUvMaxOntimeMin);
    EEPROM.update(EepromLayout::ADDR_WATER_TEMP_SENSOR, oldWaterTempSense);
    EEPROM.put(EepromLayout::ADDR_DAILY_BACKWASH_TIME_MIN, (uint16_t)EepromLayout::DEFAULT_DAILY_BACKWASH_TIME_MIN);
    EEPROM.update(EepromLayout::ADDR_DAILY_BACKWASH_DUR_10S, EepromLayout::DEFAULT_DAILY_BACKWASH_DUR_10S);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_DUR_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_DUR_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_PERIOD_MIN, EepromLayout::DEFAULT_SENSOR_BYPASS_PERIOD_MIN);
    EEPROM.update(EepromLayout::ADDR_TIMED_DISP_UNUSED, 0);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN);
    EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN);
    EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
    EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
    EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
    EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
    applyPeriodicFlushDefaults();

    writeMagic();
    return;
  }

  // Unknown/invalid magic -> full reset.
  reinitializeToDefaults(true);
}

KioskEeprom::MeasuredProfile KioskEeprom::defaultMeasuredProfile() {
  MeasuredProfile p{};
  p.duration100ms = (uint16_t)(EepromLayout::DEFAULT_DISP_DURATION_MS / 100UL); // default specified in ms; EEPROM stores 100ms units
  p.pulses     = (uint16_t)((EepromLayout::DEFAULT_DISP_PULSES > 65535UL) ? 65535U : EepromLayout::DEFAULT_DISP_PULSES);
  p.modeSel    = EepromLayout::DEFAULT_DISP_MODESEL;
  return p;
}

KioskEeprom::PwmSolenoidProfile KioskEeprom::defaultSolenoidProfile(uint8_t idx) {
  PwmSolenoidProfile s{};
  if (idx == (uint8_t)SOL_DISPENSE) {
    s.startPwm   = EepromLayout::DEFAULT_DISP_SOL_START_PWM;
    s.holdPwm    = EepromLayout::DEFAULT_DISP_SOL_HOLD_PWM;
    s.swDelaySec = EepromLayout::DEFAULT_DISP_SOL_SW_DELAY_SEC;
  } else {
    s.startPwm   = EepromLayout::DEFAULT_SOL_START_PWM;
    s.holdPwm    = EepromLayout::DEFAULT_SOL_HOLD_PWM;
    s.swDelaySec = EepromLayout::DEFAULT_SOL_SW_DELAY_SEC;
  }
  return s;
}

void KioskEeprom::reinitializeToDefaults(bool clearAllEepromBytes) {
  if (!_ready) begin();
  if (!_ready) return;

  const int len = EEPROM.length();
  if (clearAllEepromBytes) {
    for (int i = 0; i < len; ++i) EEPROM.update(i, 0x00);
  }

  EEPROM.update(EepromLayout::ADDR_HWID, 0);

  const MeasuredProfile wd = defaultMeasuredProfile();
  EEPROM.put(EepromLayout::ADDR_DISP_PROFILE, wd);

  for (uint8_t i = 0; i < PWM_SOLENOID_COUNT; ++i) {
    const PwmSolenoidProfile s = defaultSolenoidProfile(i);
    const int addr = EepromLayout::ADDR_PWM_BASE + i * (int)sizeof(PwmSolenoidProfile);
    EEPROM.put(addr, s);
  }

  EEPROM.update(EepromLayout::ADDR_BACKWASH_DUR,     EepromLayout::DEFAULT_BACKWASH_DUR_10S);
  EEPROM.update(EepromLayout::ADDR_BACKWASH_N_DISP,  EepromLayout::DEFAULT_BACKWASH_N_DISP);
  EEPROM.put(EepromLayout::ADDR_BACKWASH_COUNTER, (uint16_t)EepromLayout::DEFAULT_BACKWASH_COUNTER);
  EEPROM.put(EepromLayout::ADDR_DAILY_BACKWASH_TIME_MIN, (uint16_t)EepromLayout::DEFAULT_DAILY_BACKWASH_TIME_MIN);
  EEPROM.update(EepromLayout::ADDR_DAILY_BACKWASH_DUR_10S, EepromLayout::DEFAULT_DAILY_BACKWASH_DUR_10S);
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_DUR_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_DUR_100MS);
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_PERIOD_MIN, EepromLayout::DEFAULT_SENSOR_BYPASS_PERIOD_MIN);
  EEPROM.update(EepromLayout::ADDR_TIMED_DISP_UNUSED, 0);
  EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN);
  EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN);
  EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN);
  EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN);
  EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC);
  EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, EepromLayout::DEFAULT_PREDISP_CIRC_P_100P);
  EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, EepromLayout::DEFAULT_PREDISP_PURG_T_SEC);
  EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, EepromLayout::DEFAULT_PREDISP_PURG_P_100P);
  EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC);
  EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN);
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS);
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, EepromLayout::DEFAULT_SENSOR_BYPASS_MAN_LONG_10S);
  EEPROM.update(EepromLayout::ADDR_DNF_REPT_PERIOD_MIN, EepromLayout::DEFAULT_DNF_REPT_PERIOD_MIN);
  EEPROM.update(EepromLayout::ADDR_DNF_REPT_DURAT_100MS, EepromLayout::DEFAULT_DNF_REPT_DURAT_100MS);
  EEPROM.update(EepromLayout::ADDR_PREFLSH_BEEP_DELY_SEC, EepromLayout::DEFAULT_PREFLSH_BEEP_DELY_SEC);
  EEPROM.update(EepromLayout::ADDR_BEEP_ON_TIME_10MS, EepromLayout::DEFAULT_BEEP_ON_TIME_10MS);
  EEPROM.update(EepromLayout::ADDR_BEEP_OFF_TIME_100MS, EepromLayout::DEFAULT_BEEP_OFF_TIME_100MS);
  EEPROM.update(EepromLayout::ADDR_BEEP_COUNT, EepromLayout::DEFAULT_BEEP_COUNT);

  // KlaranUV defaults (layout v2).
  EEPROM.update(EepromLayout::ADDR_UV_OK_DELAY_10MS,  EepromLayout::DEFAULT_UV_OK_DELAY_10MS);
  EEPROM.update(EepromLayout::ADDR_UV_MAX_ONTIME_MIN, EepromLayout::DEFAULT_UV_MAX_ONTIME_MIN);

  // Water temperature sensing configuration default (layout v3).
  EEPROM.update(EepromLayout::ADDR_WATER_TEMP_SENSOR, EepromLayout::DEFAULT_WATER_TEMP_SENSOR);
  EEPROM.update(EepromLayout::ADDR_COIN_ACCEPTOR_FITTED, EepromLayout::DEFAULT_COIN_ACCEPTOR_FITTED);
  // NFC reader timing defaults.
  setNfcInitDelayMs(NFC_INIT_DELAY_DEFAULT_MS);
  setNfcReadDurationMs(NFC_SCAN_DURATION_DEFAULT_MS);
  setNfcInterNfcDelayMs(NFC_INTER_DELAY_DEFAULT_MS);


  for (uint8_t i = 0; i < DISPENSE_COUNTER_COUNT; ++i) {
    const int addr = EepromLayout::ADDR_DISPENSE_BASE + i * (int)sizeof(uint32_t);
    EEPROM.put(addr, (uint32_t)0);
  }

  writeMagic();
  writeDefaultTokenHashes();

  // Reset DS3231 Alarm1-backed "last daily backwash" timestamp.
  // Alarm1 is used as compact RTC-backed storage (2000-01-01 00:00:00 default).
  (void)KioskIO::rtcResetDailyBackwashStamp();
  // Reset DS3231 Alarm2-backed "last triggered backwash" timestamp.
  (void)KioskIO::rtcResetTriggeredBackwashStamp();
}

uint8_t KioskEeprom::hwId() const {
  return EEPROM.read(EepromLayout::ADDR_HWID);
}

void KioskEeprom::setHwId(uint8_t id) {
  EEPROM.update(EepromLayout::ADDR_HWID, id);
}

KioskEeprom::MeasuredProfile KioskEeprom::dispMeasuredProfile() const {
  MeasuredProfile p{};
  EEPROM.get(EepromLayout::ADDR_DISP_PROFILE, p);
  return p;
}

uint16_t KioskEeprom::dispMeasuredDuration100ms() const {
  uint16_t v = 0;
  EEPROM.get(EepromLayout::ADDR_DISP_DURATION100MS, v);
  return v;
}

uint16_t KioskEeprom::dispMeasuredPulses() const {
  uint16_t v = 0;
  EEPROM.get(EepromLayout::ADDR_DISP_PULSES, v);
  return v;
}

uint8_t KioskEeprom::dispMeasuredModeSel() const {
  return EEPROM.read(EepromLayout::ADDR_DISP_MODESEL);
}



uint32_t KioskEeprom::dispMeasuredDurationMs() const {
  // duration100ms is stored in 100ms increments.
  return (uint32_t)dispMeasuredDuration100ms() * 100UL;
}


uint32_t KioskEeprom::backwashDurationMs() const {
  // Stored in 5-second increments.
  return (uint32_t)backwashDuration() * 5UL * 1000UL;
}

uint32_t KioskEeprom::dailyBackwashDurationMs() const {
  // Stored in 10-second increments.
  return (uint32_t)dailyBackwashDuration10s() * 10UL * 1000UL;
}

uint32_t KioskEeprom::klaranUvOkDelayMs() const {
  // Stored in 10ms increments.
  return (uint32_t)klaranUvOkDelay10ms() * 10UL;
}

uint32_t KioskEeprom::klaranUvMaxOntimeMs() const {
  // Stored in minutes.
  return (uint32_t)klaranUvMaxOntimeMinutes() * 60UL * 1000UL;
}

uint32_t KioskEeprom::solenoidSwDelayMs(Solenoid s) const {
  const PwmSolenoidProfile p = solenoidProfile(s);
  return (uint32_t)p.swDelaySec * 1000UL;
}


void KioskEeprom::storeMeasuredProfile(const MeasuredProfile& p) {
  EEPROM.put(EepromLayout::ADDR_DISP_PROFILE, p);
}

void KioskEeprom::setMeasuredDurationMs(uint32_t ms) {
  // Convert milliseconds to 100ms units for storage.
  const uint32_t units100ms = ms / 100UL;
  MeasuredProfile p = dispMeasuredProfile();
  p.duration100ms = (uint16_t)((units100ms > 65535UL) ? 65535U : units100ms);
  storeMeasuredProfile(p);
}

void KioskEeprom::setMeasuredPulses(uint32_t pulses) {
  MeasuredProfile p = dispMeasuredProfile();
  const uint32_t p32 = pulses;
  p.pulses = (uint16_t)((p32 > 65535UL) ? 65535U : p32);
  storeMeasuredProfile(p);
}

void KioskEeprom::setMeasuredMode(uint8_t modeSel) {
  MeasuredProfile p = dispMeasuredProfile();
  p.modeSel = modeSel;
  storeMeasuredProfile(p);
}

KioskEeprom::PwmSolenoidProfile KioskEeprom::solenoidProfile(Solenoid s) const {
  const uint8_t idx = (uint8_t)s;
  PwmSolenoidProfile cfg{};
  const int addr = EepromLayout::ADDR_PWM_BASE + idx * (int)sizeof(PwmSolenoidProfile);
  EEPROM.get(addr, cfg);
  return cfg;
}

void KioskEeprom::storeSolenoidProfile(Solenoid s, const PwmSolenoidProfile& p) {
  const uint8_t idx = (uint8_t)s;
  const int addr = EepromLayout::ADDR_PWM_BASE + idx * (int)sizeof(PwmSolenoidProfile);
  EEPROM.put(addr, p);
}

uint8_t KioskEeprom::backwashDuration() const { return EEPROM.read(EepromLayout::ADDR_BACKWASH_DUR); }
uint8_t KioskEeprom::backwashAfterNDispenses() const { return EEPROM.read(EepromLayout::ADDR_BACKWASH_N_DISP); }
uint16_t KioskEeprom::backwashDispenseCounter() const {
  uint16_t v = 0;
  EEPROM.get(EepromLayout::ADDR_BACKWASH_COUNTER, v);
  return v;
}
uint8_t KioskEeprom::backwashManualShortSeconds() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_BACKWASH_MAN_SHORT_SEC : v;
}
uint8_t KioskEeprom::backwashManualLong2Minutes() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_BACKWASH_MAN_LONG_2MIN : v;
}

uint16_t KioskEeprom::dailyBackwashTimeMinutes() const {
  uint16_t v = 0;
  EEPROM.get(EepromLayout::ADDR_DAILY_BACKWASH_TIME_MIN, v);
  if (v > 1439) v = 0;
  return v;
}

uint8_t KioskEeprom::dailyBackwashDuration10s() const {
  return EEPROM.read(EepromLayout::ADDR_DAILY_BACKWASH_DUR_10S);
}

uint8_t KioskEeprom::sensorBypassDuration100ms() const {
  // Full 0..255 range is valid here (255 => 25.5s), so do not treat 0xFF as "unset".
  return EEPROM.read(EepromLayout::ADDR_SENSOR_BYPASS_DUR_100MS);
}

uint8_t KioskEeprom::sensorBypassPeriodMinutes() const {
  // Full 0..255 range is valid here, so do not treat 0xFF as "unset".
  return EEPROM.read(EepromLayout::ADDR_SENSOR_BYPASS_PERIOD_MIN);
}
uint8_t KioskEeprom::sensorBypassManualShort100ms() const {
  // Full 0..255 range is valid here, so do not treat 0xFF as "unset".
  return EEPROM.read(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS);
}
uint8_t KioskEeprom::sensorBypassManualLong10s() const {
  // Full 0..255 range is valid here, so do not treat 0xFF as "unset".
  return EEPROM.read(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S);
}

uint8_t KioskEeprom::preDispCircTSeconds() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_PREDISP_CIRC_T_SEC);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_PREDISP_CIRC_T_SEC : v;
}

uint8_t KioskEeprom::preDispCircP100Pulses() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_PREDISP_CIRC_P_100P);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_PREDISP_CIRC_P_100P : v;
}

uint8_t KioskEeprom::preDispPurgTSeconds() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_PREDISP_PURG_T_SEC);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_PREDISP_PURG_T_SEC : v;
}

uint8_t KioskEeprom::preDispPurgP100Pulses() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_PREDISP_PURG_P_100P);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_PREDISP_PURG_P_100P : v;
}

uint8_t KioskEeprom::autoCircDurationMinutes() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_AUTOCIRC_DUR_MIN);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_AUTOCIRC_DUR_MIN : v;
}

uint8_t KioskEeprom::autoCircPeriod10Minutes() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_AUTOCIRC_PERIOD_10MIN : v;
}

uint8_t KioskEeprom::manCircDurationShortMinutes() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_MANCIRC_DUR_SHORT_MIN : v;
}

uint8_t KioskEeprom::manCircDurationLong10Minutes() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_MANCIRC_DUR_LONG_10MIN : v;
}

uint8_t KioskEeprom::dnfReptPeriodMinutes() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_DNF_REPT_PERIOD_MIN);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_DNF_REPT_PERIOD_MIN : v;
}

uint8_t KioskEeprom::dnfReptDurat100ms() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_DNF_REPT_DURAT_100MS);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_DNF_REPT_DURAT_100MS : v;
}

uint8_t KioskEeprom::preFlshBeepDelySeconds() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_PREFLSH_BEEP_DELY_SEC);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_PREFLSH_BEEP_DELY_SEC : v;
}

uint8_t KioskEeprom::beepOnTime10ms() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_BEEP_ON_TIME_10MS);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_BEEP_ON_TIME_10MS : v;
}

uint8_t KioskEeprom::beepOffTime100ms() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_BEEP_OFF_TIME_100MS);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_BEEP_OFF_TIME_100MS : v;
}

uint8_t KioskEeprom::beepCount() const {
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_BEEP_COUNT);
  return (v == 0xFFU) ? EepromLayout::DEFAULT_BEEP_COUNT : v;
}

uint8_t KioskEeprom::eodCircDuration15s() const {
  // Compatibility fallback: EoD EEPROM fields are not present in this layout.
  return 120U; // 30 minutes
}

uint8_t KioskEeprom::eodBackwashDuration1s() const {
  // Compatibility fallback: EoD EEPROM fields are not present in this layout.
  return 180U; // 3 minutes
}

void KioskEeprom::setBackwashDuration(uint8_t v) { EEPROM.update(EepromLayout::ADDR_BACKWASH_DUR, v); }
void KioskEeprom::setBackwashAfterNDispenses(uint8_t v) { EEPROM.update(EepromLayout::ADDR_BACKWASH_N_DISP, v); }
void KioskEeprom::setBackwashDispenseCounter(uint16_t v) {
  uint16_t cur = 0;
  EEPROM.get(EepromLayout::ADDR_BACKWASH_COUNTER, cur);
  if (cur != v) EEPROM.put(EepromLayout::ADDR_BACKWASH_COUNTER, v);
}
void KioskEeprom::setBackwashManualShortSeconds(uint8_t seconds) {
  EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_SHORT_SEC, seconds);
}
void KioskEeprom::setBackwashManualLong2Minutes(uint8_t units2min) {
  EEPROM.update(EepromLayout::ADDR_BACKWASH_MAN_LONG_2MIN, units2min);
}
void KioskEeprom::setDailyBackwashTimeMinutes(uint16_t minutes) {
  if (minutes > 1439) minutes = 0;
  EEPROM.put(EepromLayout::ADDR_DAILY_BACKWASH_TIME_MIN, minutes);
}

void KioskEeprom::setDailyBackwashDuration10s(uint8_t units10s) {
  EEPROM.update(EepromLayout::ADDR_DAILY_BACKWASH_DUR_10S, units10s);
}

void KioskEeprom::setSensorBypassDuration100ms(uint8_t units100ms) {
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_DUR_100MS, units100ms);
}

void KioskEeprom::setSensorBypassPeriodMinutes(uint8_t minutes) {
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_PERIOD_MIN, minutes);
}
void KioskEeprom::setSensorBypassManualShort100ms(uint8_t units100ms) {
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_SHORT_100MS, units100ms);
}
void KioskEeprom::setSensorBypassManualLong10s(uint8_t units10s) {
  EEPROM.update(EepromLayout::ADDR_SENSOR_BYPASS_MAN_LONG_10S, units10s);
}

void KioskEeprom::setPreDispCircTSeconds(uint8_t seconds) {
  EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_T_SEC, seconds);
}

void KioskEeprom::setPreDispCircP100Pulses(uint8_t units100p) {
  EEPROM.update(EepromLayout::ADDR_PREDISP_CIRC_P_100P, units100p);
}

void KioskEeprom::setPreDispPurgTSeconds(uint8_t seconds) {
  EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_T_SEC, seconds);
}

void KioskEeprom::setPreDispPurgP100Pulses(uint8_t units100p) {
  EEPROM.update(EepromLayout::ADDR_PREDISP_PURG_P_100P, units100p);
}

void KioskEeprom::setAutoCircDurationMinutes(uint8_t minutes) {
  EEPROM.update(EepromLayout::ADDR_AUTOCIRC_DUR_MIN, minutes);
}

void KioskEeprom::setAutoCircPeriod10Minutes(uint8_t units10min) {
  EEPROM.update(EepromLayout::ADDR_AUTOCIRC_PERIOD_10MIN, units10min);
}

void KioskEeprom::setManCircDurationShortMinutes(uint8_t minutes) {
  EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_SHORT_MIN, minutes);
}

void KioskEeprom::setManCircDurationLong10Minutes(uint8_t units10min) {
  EEPROM.update(EepromLayout::ADDR_MANCIRC_DUR_LONG_10MIN, units10min);
}

void KioskEeprom::setDnfReptPeriodMinutes(uint8_t minutes) {
  EEPROM.update(EepromLayout::ADDR_DNF_REPT_PERIOD_MIN, minutes);
}

void KioskEeprom::setDnfReptDurat100ms(uint8_t units100ms) {
  EEPROM.update(EepromLayout::ADDR_DNF_REPT_DURAT_100MS, units100ms);
}

void KioskEeprom::setPreFlshBeepDelySeconds(uint8_t seconds) {
  EEPROM.update(EepromLayout::ADDR_PREFLSH_BEEP_DELY_SEC, seconds);
}

void KioskEeprom::setBeepOnTime10ms(uint8_t units10ms) {
  EEPROM.update(EepromLayout::ADDR_BEEP_ON_TIME_10MS, units10ms);
}

void KioskEeprom::setBeepOffTime100ms(uint8_t units100ms) {
  EEPROM.update(EepromLayout::ADDR_BEEP_OFF_TIME_100MS, units100ms);
}

void KioskEeprom::setBeepCount(uint8_t count) {
  EEPROM.update(EepromLayout::ADDR_BEEP_COUNT, count);
}

// -------------------- KlaranUV configuration (layout v2) --------------------
// Stored as raw uint8_t values:
// - klaranUvOkDelay10ms(): 10ms increments (150 => 1500ms)
// - klaranUvMaxOntimeMinutes(): minutes (60 => 1 hour)
uint8_t KioskEeprom::klaranUvOkDelay10ms() const { return EEPROM.read(EepromLayout::ADDR_UV_OK_DELAY_10MS); }
uint8_t KioskEeprom::klaranUvMaxOntimeMinutes() const { return EEPROM.read(EepromLayout::ADDR_UV_MAX_ONTIME_MIN); }

void KioskEeprom::setKlaranUvOkDelay10ms(uint8_t v) { EEPROM.update(EepromLayout::ADDR_UV_OK_DELAY_10MS, v); }
void KioskEeprom::setKlaranUvMaxOntimeMinutes(uint8_t v) { EEPROM.update(EepromLayout::ADDR_UV_MAX_ONTIME_MIN, v); }

// -------------------- Water temperature sensing configuration (layout v3) --------------------
// Select which DS18B20 (T1 or T2) is used as the water temperature sensor.
// Stored as raw uint8_t: 1=T1 (default), 2=T2.
uint8_t KioskEeprom::waterTempSense() const {
  uint8_t v = EEPROM.read(EepromLayout::ADDR_WATER_TEMP_SENSOR);
  if (v < 1 || v > 2) v = 1;
  return v;
}

void KioskEeprom::setWaterTempSense(uint8_t v) {
  if (v < 1 || v > 2) v = 1;
  EEPROM.update(EepromLayout::ADDR_WATER_TEMP_SENSOR, v);
}

bool KioskEeprom::coinAcceptorFitted() const {
  return EEPROM.read(EepromLayout::ADDR_COIN_ACCEPTOR_FITTED) == 1;
}

void KioskEeprom::setCoinAcceptorFitted(bool fitted) {
  EEPROM.update(EepromLayout::ADDR_COIN_ACCEPTOR_FITTED, fitted ? 1 : 0);
}

// -------------------- NFC reader timing parameters (layout v4) --------------------
// Stored as compact uint8_t values in EEPROM.
// API contracts:
//  - getters return milliseconds
//  - setters take milliseconds and clamp/quantize to storage units
uint32_t KioskEeprom::nfcInitDelayMs() const {
  uint8_t v = EEPROM.read(EepromLayout::ADDR_NFC_INIT_DELAY_MS);
  if (v < (uint8_t)KioskEeprom::NFC_INIT_DELAY_MIN_MS || v > (uint8_t)KioskEeprom::NFC_INIT_DELAY_MAX_MS) {
    v = (uint8_t)KioskEeprom::NFC_INIT_DELAY_DEFAULT_MS;
  }
  return (uint32_t)v;
}

uint32_t KioskEeprom::nfcInterNfcDelayMs() const {
  uint8_t v = EEPROM.read(EepromLayout::ADDR_NFC_INTER_DELAY_MS);
  if (v < (uint8_t)KioskEeprom::NFC_INTER_DELAY_MIN_MS || v > (uint8_t)KioskEeprom::NFC_INTER_DELAY_MAX_MS) {
    v = (uint8_t)KioskEeprom::NFC_INTER_DELAY_DEFAULT_MS;
  }
  return (uint32_t)v;
}

uint32_t KioskEeprom::nfcReadDurationMs() const {
  // Stored in 10ms units.
  const uint8_t v = EEPROM.read(EepromLayout::ADDR_NFC_SCAN_DURATION_10MS);

  const uint8_t min10 = (uint8_t)(KioskEeprom::NFC_SCAN_DURATION_MIN_MS / 10UL);
  const uint8_t max10 = (uint8_t)(KioskEeprom::NFC_SCAN_DURATION_MAX_MS / 10UL);
  const uint8_t def10 = (uint8_t)(KioskEeprom::NFC_SCAN_DURATION_DEFAULT_MS / 10UL);

  uint8_t units10 = v;
  if (units10 < min10 || units10 > max10) {
    units10 = def10;
  }
  return (uint32_t)units10 * 10UL;
}

void KioskEeprom::setNfcInitDelayMs(uint32_t ms) {
  uint32_t v = ms;
  if (v < KioskEeprom::NFC_INIT_DELAY_MIN_MS) v = KioskEeprom::NFC_INIT_DELAY_MIN_MS;
  if (v > KioskEeprom::NFC_INIT_DELAY_MAX_MS) v = KioskEeprom::NFC_INIT_DELAY_MAX_MS;
  EEPROM.update(EepromLayout::ADDR_NFC_INIT_DELAY_MS, (uint8_t)v);
}

void KioskEeprom::setNfcInterNfcDelayMs(uint32_t ms) {
  uint32_t v = ms;
  if (v < KioskEeprom::NFC_INTER_DELAY_MIN_MS) v = KioskEeprom::NFC_INTER_DELAY_MIN_MS;
  if (v > KioskEeprom::NFC_INTER_DELAY_MAX_MS) v = KioskEeprom::NFC_INTER_DELAY_MAX_MS;
  EEPROM.update(EepromLayout::ADDR_NFC_INTER_DELAY_MS, (uint8_t)v);
}

void KioskEeprom::setNfcReadDurationMs(uint32_t ms) {
  // Stored in 10ms units.
  uint32_t v = ms;
  if (v < KioskEeprom::NFC_SCAN_DURATION_MIN_MS) v = KioskEeprom::NFC_SCAN_DURATION_MIN_MS;
  if (v > KioskEeprom::NFC_SCAN_DURATION_MAX_MS) v = KioskEeprom::NFC_SCAN_DURATION_MAX_MS;

  // Quantize to nearest 10ms (ties up). Always keep within declared range.
  uint32_t units10 = (v + 5UL) / 10UL;
  const uint32_t min10 = (KioskEeprom::NFC_SCAN_DURATION_MIN_MS / 10UL);
  const uint32_t max10 = (KioskEeprom::NFC_SCAN_DURATION_MAX_MS / 10UL);
  if (units10 < min10) units10 = min10;
  if (units10 > max10) units10 = max10;

  EEPROM.update(EepromLayout::ADDR_NFC_SCAN_DURATION_10MS, (uint8_t)units10);
}

uint32_t KioskEeprom::dispenseCounter(DispenseCounter idx) const {
  const uint8_t i = (uint8_t)idx;
  const int addr = EepromLayout::ADDR_DISPENSE_BASE + i * (int)sizeof(uint32_t);
  uint32_t v = 0;
  EEPROM.get(addr, v);
  return v;
}

void KioskEeprom::setDispenseCounter(DispenseCounter idx, uint32_t value) {
  const uint8_t i = (uint8_t)idx;
  const int addr = EepromLayout::ADDR_DISPENSE_BASE + i * (int)sizeof(uint32_t);
  EEPROM.put(addr, value);
}

uint32_t KioskEeprom::incrementDispenseCounter(DispenseCounter idx, uint32_t delta) {
  const uint8_t i = (uint8_t)idx;
  const int addr = EepromLayout::ADDR_DISPENSE_BASE + i * (int)sizeof(uint32_t);
  uint32_t v = 0;
  EEPROM.get(addr, v);

  // Saturating add to avoid wrap.
  if (UINT32_MAX - v < delta) v = UINT32_MAX;
  else v += delta;

  EEPROM.put(addr, v);
  return v;
}

uint32_t KioskEeprom::totalDispensedUnits() const {
  uint32_t total = 0;
  for (uint8_t i = 0; i < DISPENSE_COUNTER_COUNT; ++i) {
    total += dispenseCounter((DispenseCounter)i);
  }
  return total;
}

uint32_t KioskEeprom::tokenHashAt(uint16_t idx) const {
  if (!_ready) return EEPROM_TOKEN_EMPTY;
  if (idx >= EEPROM_TOKEN_MAX) return EEPROM_TOKEN_EMPTY;
  const int addr = _tokenBaseAddr + (int)idx * (int)sizeof(uint32_t);
  uint32_t h = EEPROM_TOKEN_EMPTY;
  EEPROM.get(addr, h);
  return h;
}

uint16_t KioskEeprom::countActiveTokens() const {
  uint16_t c = 0;
  for (uint16_t i = 0; i < EEPROM_TOKEN_MAX; ++i) {
    if (tokenHashAt(i) != EEPROM_TOKEN_EMPTY) ++c;
  }
  return c;
}

uint16_t KioskEeprom::findTokenHash(uint32_t hash) const {
  if (hash == EEPROM_TOKEN_EMPTY) return EEPROM_TOKEN_MAX;
  for (uint16_t i = 0; i < EEPROM_TOKEN_MAX; ++i) {
    if (tokenHashAt(i) == hash) return i;
  }
  return EEPROM_TOKEN_MAX;
}

bool KioskEeprom::tokenHashExists(uint32_t hash) const {
  return findTokenHash(hash) < EEPROM_TOKEN_MAX;
}

bool KioskEeprom::testTokenHashPresent(uint32_t hash) const {
  return tokenHashExists(hash);
}

uint16_t KioskEeprom::packTokenHashTable() {
  if (!_ready) return 0;

  // Step 1: De-duplicate.
  // Keep the first occurrence (lowest index) and clear later duplicates.
  uint16_t changes = 0;
  for (uint16_t i = 0; i < EEPROM_TOKEN_MAX; ++i) {
    const uint32_t hi = tokenHashAt(i);
    if (hi == EEPROM_TOKEN_EMPTY) continue;
    for (uint16_t j = i + 1; j < EEPROM_TOKEN_MAX; ++j) {
      if (tokenHashAt(j) != hi) continue;
      const int addrJ = _tokenBaseAddr + (int)j * (int)sizeof(uint32_t);
      EEPROM.put(addrJ, EEPROM_TOKEN_EMPTY);
      ++changes;
    }
  }

  // Step 2: Pack/compact to remove "holes".
  int32_t last = (int32_t)EEPROM_TOKEN_MAX - 1;

  for (uint16_t i = 0; i < EEPROM_TOKEN_MAX; ++i) {
    const uint32_t h = tokenHashAt(i);
    if (h != EEPROM_TOKEN_EMPTY) continue;

    // Find the furthest (highest-index) active token after i.
    while (last > (int32_t)i) {
      const uint32_t tail = tokenHashAt((uint16_t)last);
      if (tail != EEPROM_TOKEN_EMPTY) {
        const int addrI = _tokenBaseAddr + (int)i * (int)sizeof(uint32_t);
        const int addrL = _tokenBaseAddr + (int)last * (int)sizeof(uint32_t);
        EEPROM.put(addrI, tail);
        EEPROM.put(addrL, EEPROM_TOKEN_EMPTY);
        ++changes;
        --last;
        break;
      }
      --last;
    }

    // No more active tokens beyond i.
    if (last <= (int32_t)i) break;
  }

  return changes;
}

uint16_t KioskEeprom::findEmptyTokenSlot() const {
  for (uint16_t i = 0; i < EEPROM_TOKEN_MAX; ++i) {
    if (tokenHashAt(i) == EEPROM_TOKEN_EMPTY) return i;
  }
  return EEPROM_TOKEN_MAX;
}

bool KioskEeprom::addTokenHash(uint32_t hash) {
  if (hash == EEPROM_TOKEN_EMPTY) return false;
  if (tokenHashExists(hash)) return false;
  const uint16_t slot = findEmptyTokenSlot();
  if (slot >= EEPROM_TOKEN_MAX) return false;
  const int addr = _tokenBaseAddr + (int)slot * (int)sizeof(uint32_t);
  EEPROM.put(addr, hash);
  return true;
}

bool KioskEeprom::deleteTokenHash(uint32_t hash) {
  if (hash == EEPROM_TOKEN_EMPTY) return false;
  const uint16_t slot = findTokenHash(hash);
  if (slot >= EEPROM_TOKEN_MAX) return false;
  const int addr = _tokenBaseAddr + (int)slot * (int)sizeof(uint32_t);
  EEPROM.put(addr, EEPROM_TOKEN_EMPTY);
  return true;
}

void KioskEeprom::removeAllTokens() {
  if (!_ready) return;
  for (uint16_t i = 0; i < EEPROM_TOKEN_MAX; ++i) {
    const int addr = _tokenBaseAddr + (int)i * (int)sizeof(uint32_t);
    EEPROM.put(addr, EEPROM_TOKEN_EMPTY);
  }
}

void KioskEeprom::writeDefaultTokenHashes() {
  removeAllTokens();
  for (uint8_t i = 0; i < EepromLayout::DEFAULT_NFC_TOKEN_COUNT; ++i) {
    const int addr = _tokenBaseAddr + (int)i * (int)sizeof(uint32_t);
    EEPROM.put(addr, EepromLayout::DEFAULT_NFC_TOKEN_HASHES[i]);
  }
}

bool KioskEeprom::solenoidFromName(const char* name, Solenoid& out) {
  if (!name) return false;
  if (!strcmp(name, "BackWashSol"))         { out = SOL_BACKWASH; return true; }
  if (!strcmp(name, "WaterSenseBypSol"))    { out = SOL_BYPASS;   return true; }
  if (!strcmp(name, "InletWaterSol"))       { out = SOL_INLET;    return true; }
  if (!strcmp(name, "WaterDispSol"))        { out = SOL_DISPENSE; return true; }
  return false;
}

} // namespace Kiosk
