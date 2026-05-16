/*
  KioskEepromLayout.h
  ------------------
  Private EEPROM schema + default values.

  Intent:
  - Keep EEPROM layout details (addresses, magic values, default register values)
    in one canonical location.
  - This header is *not* part of the public API. It should be included only by
    KioskEeprom.cpp (and, if ever required, by the EEPROM editor .cpp for
    advanced diagnostics).

  NOTE:
  - The current layout matches the existing in-field format used by the baseline
    firmware/editor. Do not change offsets casually.
  - If you change the on-EEPROM format, you should also add a versioning/migration
    strategy (or reinitialize).
*/

#pragma once

#include <Arduino.h>

namespace Kiosk {
namespace EepromLayout {

// -------------------- Fixed register addresses (legacy/stable) --------------------
static constexpr int ADDR_HWID                = 0;
static constexpr int ADDR_DISP_PROFILE          = 1;
// MeasuredProfile is stored at ADDR_DISP_PROFILE as a packed block:
//   duration100ms : uint16_t (100ms units)  [offset +0]
//   pulses       : uint16_t                [offset +2]
//   modeSel      : uint8_t                 [offset +4]
// Individual field addresses (for direct access without reading the whole struct):
static constexpr int ADDR_DISP_DURATION100MS    = ADDR_DISP_PROFILE + 0;
static constexpr int ADDR_DISP_PULSES           = ADDR_DISP_PROFILE + 2;
static constexpr int ADDR_DISP_MODESEL          = ADDR_DISP_PROFILE + 4;

static constexpr int ADDR_PWM_BASE            = 10;

// -------------------- Solenoid PWM profiles --------------------
// 4 x PwmSolenoidProfile blocks are stored contiguously starting at ADDR_PWM_BASE.
//
// PwmSolenoidProfile is defined in KioskEeprom.h as a packed 5-byte block:
//   startPwm        : uint8_t  [offset +0]
//   holdPwm         : uint8_t  [offset +1]
//   swDelaySec      : uint8_t  [offset +2]
//   reserved0       : uint8_t  [offset +3] (unused; kept for compatibility)
//   reserved1       : int8_t   [offset +4] (unused; kept for compatibility)
//
// Solenoid indices (KioskEeprom::Solenoid):
//   0 = BackWashSol
//   1 = WaterSenseBypSol
//   2 = InletWaterSol
//   3 = WaterDispSol
static constexpr int SOL_PROFILE_SIZE_BYTES   = 5;

// Base addresses for each solenoid profile (for clarity / direct inspection).
static constexpr int ADDR_SOL_BACKWASH_BASE   = ADDR_PWM_BASE + 0 * SOL_PROFILE_SIZE_BYTES; // 10
static constexpr int ADDR_SOL_BYPASS_BASE     = ADDR_PWM_BASE + 1 * SOL_PROFILE_SIZE_BYTES; // 15
static constexpr int ADDR_SOL_INLET_BASE      = ADDR_PWM_BASE + 2 * SOL_PROFILE_SIZE_BYTES; // 20
static constexpr int ADDR_SOL_DISPENSE_BASE   = ADDR_PWM_BASE + 3 * SOL_PROFILE_SIZE_BYTES; // 25

// Active field addresses for each solenoid (Start/Hold/SwDelay).
static constexpr int ADDR_SOL_BACKWASH_STARTPWM   = ADDR_SOL_BACKWASH_BASE + 0;
static constexpr int ADDR_SOL_BACKWASH_HOLDPWM    = ADDR_SOL_BACKWASH_BASE + 1;
static constexpr int ADDR_SOL_BACKWASH_SWDELAYSEC = ADDR_SOL_BACKWASH_BASE + 2;

static constexpr int ADDR_SOL_BYPASS_STARTPWM      = ADDR_SOL_BYPASS_BASE + 0;
static constexpr int ADDR_SOL_BYPASS_HOLDPWM       = ADDR_SOL_BYPASS_BASE + 1;
static constexpr int ADDR_SOL_BYPASS_SWDELAYSEC    = ADDR_SOL_BYPASS_BASE + 2;

static constexpr int ADDR_SOL_INLET_STARTPWM       = ADDR_SOL_INLET_BASE + 0;
static constexpr int ADDR_SOL_INLET_HOLDPWM        = ADDR_SOL_INLET_BASE + 1;
static constexpr int ADDR_SOL_INLET_SWDELAYSEC     = ADDR_SOL_INLET_BASE + 2;

static constexpr int ADDR_SOL_DISPENSE_STARTPWM    = ADDR_SOL_DISPENSE_BASE + 0;
static constexpr int ADDR_SOL_DISPENSE_HOLDPWM     = ADDR_SOL_DISPENSE_BASE + 1;
static constexpr int ADDR_SOL_DISPENSE_SWDELAYSEC  = ADDR_SOL_DISPENSE_BASE + 2;

// Backwash frequency field is deprecated/unused; keep byte reserved for compatibility.
static constexpr int ADDR_BACKWASH_FREQ_UNUSED = 30;
// Backwash default duration in 5s units.
// 0 disables default-duration backwash starts.
static constexpr int ADDR_BACKWASH_DUR        = 31;
static constexpr int ADDR_BACKWASH_N_DISP     = 32;
// Backwash "dispense counter" stores the TOTAL dispensed units value the last time a backwash ran.
// Stored as uint16_t at ADDR_BACKWASH_COUNTER.
static constexpr int ADDR_BACKWASH_COUNTER    = 33;

// KlaranUV behaviour parameters (added in layout v2).
// Stored as raw uint8_t.
static constexpr int ADDR_UV_OK_DELAY_10MS    = 37;  // stored_value * 10ms
static constexpr int ADDR_UV_MAX_ONTIME_MIN   = 38;  // minutes

// Water temperature sensing configuration (added in layout v3).
// Select which DS18B20 (T1 or T2) is used as the water temperature sensor.
// Stored as raw uint8_t:
//   1 = T1 (default)
//   2 = T2
static constexpr int ADDR_WATER_TEMP_SENSOR     = 39;

// NFC reader timing parameters (added in layout v4; stored in previously-reserved bytes).
// - NFC_InitDelay: delay after bringing PN532 out of reset before first read.
//   Stored as raw uint8_t: 1 LSB = 1ms.
// - NFC_SCANDuration: maximum time spent waiting for a tag in a single read cycle.
//   Stored as raw uint8_t: 1 LSB = 10ms.
// - NFC_InterNFCdelay: delay between consecutive NFC operations.
//   Stored as raw uint8_t: 1 LSB = 1ms.
static constexpr int ADDR_NFC_INIT_DELAY_MS      = 40;  // stored_value * 1ms
static constexpr int ADDR_NFC_SCAN_DURATION_10MS = 41;  // stored_value * 10ms
static constexpr int ADDR_NFC_INTER_DELAY_MS     = 42;  // stored_value * 1ms

// Daily backwash scheduling (layout v5).
// - DailyBackwashTime: minutes since midnight (0..1439), stored as uint16_t.
// - DailyBackwashDur: 10-second units (0..255), stored as uint8_t. 0 = disabled.
static constexpr int ADDR_DAILY_BACKWASH_TIME_MIN = 43;  // uint16_t
static constexpr int ADDR_DAILY_BACKWASH_DUR_10S  = 45;  // uint8_t
// Coin acceptor fitted flag (layout v6).
// Stored as raw uint8_t: 0 = No, 1 = Yes.
static constexpr int ADDR_COIN_ACCEPTOR_FITTED   = 46;  // uint8_t

// Backwash RTC stamp metadata (stored in reserved bytes).
// Alarm registers store day/time; these EEPROM bytes store year-offset and month:
//   yearOffset = year - 2000, valid 0..225 (years 2000..2225).
static constexpr int ADDR_DAILY_BW_YEAR_OFFSET    = 47;  // uint8_t (0..225)
static constexpr int ADDR_TRIG_BW_YEAR_OFFSET     = 48;  // uint8_t (0..225)
static constexpr int ADDR_DAILY_BW_MONTH          = 49;  // uint8_t (1..12)
static constexpr int ADDR_TRIG_BW_MONTH           = 50;  // uint8_t (1..12)
// SensorBypass timing parameters.
// - Duration: raw uint8_t, 1 LSB = 100ms.
// - Period: raw uint8_t, 1 LSB = 1 minute.
static constexpr int ADDR_SENSOR_BYPASS_DUR_100MS = 51;  // uint8_t
static constexpr int ADDR_SENSOR_BYPASS_PERIOD_MIN= 52;  // uint8_t
// Reserved/unused byte (formerly timed dispense averaging count).
static constexpr int ADDR_TIMED_DISP_UNUSED      = 53;  // uint8_t
// Water circulation control (Hydr Parm Edit -> Water Circ Ctrl).
// - AutoCircDuratn: 1 LSB = 1 minute, 0 disables auto circulation.
// - AutoCircPeriod: 1 LSB = 10 minutes, 0 disables auto circulation.
// - ManCircDur Shrt: 1 LSB = 1 minute, 0 disables short manual circulation.
// - ManCircDur Long: 1 LSB = 10 minutes, 0 disables long manual circulation.
static constexpr int ADDR_AUTOCIRC_DUR_MIN       = 54;  // uint8_t
static constexpr int ADDR_AUTOCIRC_PERIOD_10MIN  = 55;  // uint8_t
static constexpr int ADDR_MANCIRC_DUR_SHORT_MIN  = 56;  // uint8_t
static constexpr int ADDR_MANCIRC_DUR_LONG_10MIN = 57;  // uint8_t
// Water Disp Ctrl pre-dispense parameters.
// - PreDispCircT: timed-mode circulation pre-run, 1 LSB = 1 second.
// - PreDispCircP: pulse-mode circulation pre-run, 1 LSB = 100 pulses.
// - PreDispPurgT: timed-mode purge pre-run, 1 LSB = 1 second.
// - PreDispPurgP: pulse-mode purge pre-run, 1 LSB = 100 pulses.
static constexpr int ADDR_PREDISP_CIRC_T_SEC     = 58;  // uint8_t
static constexpr int ADDR_PREDISP_CIRC_P_100P    = 59;  // uint8_t
static constexpr int ADDR_PREDISP_PURG_T_SEC     = 60;  // uint8_t
static constexpr int ADDR_PREDISP_PURG_P_100P    = 61;  // uint8_t
// Backwash manual durations.
// - BW MAN Short: 1 LSB = 1 second.
// - BW MAN Long: 1 LSB = 2 minutes.
static constexpr int ADDR_BACKWASH_MAN_SHORT_SEC = 62;  // uint8_t
static constexpr int ADDR_BACKWASH_MAN_LONG_2MIN = 63;  // uint8_t
// SensorBypass manual durations.
// - MAN Bypass Short: 1 LSB = 100ms.
// - MAN Bypass Long: 1 LSB = 10s.
static constexpr int ADDR_SENSOR_BYPASS_MAN_SHORT_100MS = 64; // uint8_t
static constexpr int ADDR_SENSOR_BYPASS_MAN_LONG_10S    = 65; // uint8_t
// Periodic Flush parameters (Hydra Parm Edit -> Periodic Flush).
// - DNF Rept Period: 1 LSB = 1 minute.
// - DNF DispDurat: 1 LSB = 100ms.
// - PreFlshBeepDely: 1 LSB = 1 second.
// - Beep ON Time: 1 LSB = 10ms.
// - Beep Period: 1 LSB = 100ms.
// - Beep Count: 1 LSB = 1.
static constexpr int ADDR_DNF_REPT_PERIOD_MIN      = 66; // uint8_t
static constexpr int ADDR_DNF_REPT_DURAT_100MS     = 67; // uint8_t
static constexpr int ADDR_PREFLSH_BEEP_DELY_SEC    = 68; // uint8_t
static constexpr int ADDR_BEEP_ON_TIME_10MS        = 69; // uint8_t
static constexpr int ADDR_BEEP_OFF_TIME_100MS      = 70; // uint8_t
static constexpr int ADDR_BEEP_COUNT               = 71; // uint8_t

// NOTE: Layout migration
// - Prior layouts stored ADDR_UV_OK_DELAY_10MS at 34, ADDR_UV_MAX_ONTIME_MIN at 35, and
//   ADDR_WATER_TEMP_SENSOR at 36 (all uint8_t). The backwash counter was uint8_t at 33.
// - Layout v12 widened the backwash counter at 33 to uint32_t (bytes 33..36), shifting the
//   later uint8_t fields to 37..39.
// - Current layout stores backwash counter as uint16_t at 33..34. Bytes 35..36 are reserved.

// Reserved bytes 72..123.

static constexpr int ADDR_MAGIC               = 124;
static constexpr int ADDR_DISPENSE_BASE       = 128; // 4 x uint32 counters

// Magic marker for initialization.
// Current magic value for backwash counter uint16 layout ("MJCD").
static constexpr uint32_t MAGIC_VALUE         = 0x4D4A4344UL;
// Prior magic used before backwash counter width change ("MJCC").
static constexpr uint32_t MAGIC_VALUE_V12     = 0x4D4A4343UL;
// Prior magic used before Beep ON unit update ("MJCB").
static constexpr uint32_t MAGIC_VALUE_V11     = 0x4D4A4342UL;
// Prior magic used before updated periodic flush fields ("MJCA").
static constexpr uint32_t MAGIC_VALUE_V10     = 0x4D4A4341UL;
// Prior magic used before periodic flush fields ("MJC9").
static constexpr uint32_t MAGIC_VALUE_V9      = 0x4D4A4339UL;
// Prior magic used before manual sensor bypass duration fields ("MJC8").
static constexpr uint32_t MAGIC_VALUE_V8      = 0x4D4A4338UL;
// Prior magic used before manual backwash duration fields ("MJC7").
static constexpr uint32_t MAGIC_VALUE_V7      = 0x4D4A4337UL;
// Prior magic used before pre-dispense dispense parms ("MJC6").
static constexpr uint32_t MAGIC_VALUE_V6      = 0x4D4A4336UL;
// Prior magic used before water-circ controls ("MJC5").
static constexpr uint32_t MAGIC_VALUE_V5      = 0x4D4A4335UL;
// Prior magic used before timed-disp avg-count field ("MJC4").
static constexpr uint32_t MAGIC_VALUE_V4      = 0x4D4A4334UL;
// Prior magic used before sensor bypass timing fields ("MJC3").
static constexpr uint32_t MAGIC_VALUE_V3      = 0x4D4A4333UL;
// Prior magic used before daily backwash scheduling ("MJC2").
static constexpr uint32_t MAGIC_VALUE_V2      = 0x4D4A4332UL;
// Legacy magic ("MJCM") used by the prior layout.
static constexpr uint32_t MAGIC_VALUE_LEGACY  = 0x4D4A434DUL;

// -------------------- Defaults (register values) --------------------

// Backwash defaults.
static constexpr uint8_t DEFAULT_BACKWASH_DUR_10S    = 0;   // stored in 5s units
static constexpr uint8_t DEFAULT_BACKWASH_N_DISP     = 0;
static constexpr uint16_t DEFAULT_BACKWASH_COUNTER   = 0U;
static constexpr uint16_t DEFAULT_DAILY_BACKWASH_TIME_MIN = 60; // 01:00
static constexpr uint8_t  DEFAULT_DAILY_BACKWASH_DUR_10S  = 18; // 180s (3 minutes)
static constexpr uint8_t  DEFAULT_SENSOR_BYPASS_DUR_100MS = 0;  // 0 = disabled
static constexpr uint8_t  DEFAULT_SENSOR_BYPASS_PERIOD_MIN = 0; // 0 = disabled

// KlaranUV defaults.
static constexpr uint8_t DEFAULT_UV_OK_DELAY_10MS    = 150; // 150 * 10ms = 1500ms
static constexpr uint8_t DEFAULT_UV_MAX_ONTIME_MIN   = 60;  // 60 minutes

// Water temperature sensing configuration default (layout v3).
// 1=T1, 2=T2
static constexpr uint8_t DEFAULT_WATER_TEMP_SENSOR    = 1;
static constexpr uint8_t DEFAULT_COIN_ACCEPTOR_FITTED = 0; // 0 = No

// Measured dispense defaults (note: duration stored as 100ms units, but default is specified in ms).
static constexpr uint32_t DEFAULT_DISP_DURATION_MS     = 20000UL;
static constexpr uint16_t DEFAULT_DISP_DURATION_100MS  = (uint16_t)(DEFAULT_DISP_DURATION_MS / 100UL);
static constexpr uint32_t DEFAULT_DISP_PULSES          = 2000UL;
// WD_MODE_TIME == 0
static constexpr uint8_t  DEFAULT_DISP_MODESEL         = 0;
static constexpr uint8_t  DEFAULT_AUTOCIRC_DUR_MIN       = 0;
static constexpr uint8_t  DEFAULT_AUTOCIRC_PERIOD_10MIN  = 0;
static constexpr uint8_t  DEFAULT_MANCIRC_DUR_SHORT_MIN  = 1;
static constexpr uint8_t  DEFAULT_MANCIRC_DUR_LONG_10MIN = 1;
static constexpr uint8_t  DEFAULT_PREDISP_CIRC_T_SEC      = 0;
static constexpr uint8_t  DEFAULT_PREDISP_CIRC_P_100P     = 0;
static constexpr uint8_t  DEFAULT_PREDISP_PURG_T_SEC      = 0;
static constexpr uint8_t  DEFAULT_PREDISP_PURG_P_100P     = 0;
static constexpr uint8_t  DEFAULT_BACKWASH_MAN_SHORT_SEC   = 15; // 15 seconds
static constexpr uint8_t  DEFAULT_BACKWASH_MAN_LONG_2MIN   = 15; // 30 minutes
static constexpr uint8_t  DEFAULT_SENSOR_BYPASS_MAN_SHORT_100MS = 50; // 5.0 seconds
static constexpr uint8_t  DEFAULT_SENSOR_BYPASS_MAN_LONG_10S    = 6;  // 1 minute
static constexpr uint8_t  DEFAULT_DNF_REPT_PERIOD_MIN      = 60; // 60 minutes
static constexpr uint8_t  DEFAULT_DNF_REPT_DURAT_100MS     = 20; // 2s
static constexpr uint8_t  DEFAULT_PREFLSH_BEEP_DELY_SEC    = 15; // 15s
static constexpr uint8_t  DEFAULT_BEEP_ON_TIME_10MS        = 5;  // 50ms
// Requested default is 950ms with 100ms LSB; nearest stored value is 1000ms.
static constexpr uint8_t  DEFAULT_BEEP_OFF_TIME_100MS      = 10; // 1000ms
static constexpr uint8_t  DEFAULT_BEEP_COUNT               = 0;

// Solenoid defaults.
static constexpr uint8_t DEFAULT_SOL_START_PWM       = 255;
static constexpr uint8_t DEFAULT_SOL_HOLD_PWM        = 130;
static constexpr uint8_t DEFAULT_SOL_SW_DELAY_SEC    = 5;

// Dispense solenoid (WaterDispSol) defaults differ.
static constexpr uint8_t DEFAULT_DISP_SOL_START_PWM       = 128;
static constexpr uint8_t DEFAULT_DISP_SOL_HOLD_PWM        = 100;
static constexpr uint8_t DEFAULT_DISP_SOL_SW_DELAY_SEC    = 5;



// -------------------- Default NFC token hashes --------------------
//
// These are the "known good" NFC token hashes used to seed the token table when
// EEPROM is first initialized (or reinitialized).
//
static const uint32_t DEFAULT_NFC_TOKEN_HASHES[] = {
  0x0A5D281BUL, // 14 November Tag
  0x0D1193C2UL, // Tag OK #1
  0x2022301EUL, // 14 November Tag
  0x356BB66DUL, // Tag OK #2
  0x4800E934UL, // 14 November Tag
  0x4DBEB28AUL, // Tag OK #3
  0x591AC9A8UL, // 14 November Tag
  0x5CC8C061UL, // Tag sent with KIOSK3
  0x7F026686UL, // 14 November Tag
  0x841709BFUL,
  0x84AEC09CUL, // 14 November Tag
  0x9A9FEE82UL, // 14 November Tag
  0xE2963725UL, // 14 November Tag
  0xE3557210UL  // 14 November Tag
};
static constexpr uint8_t DEFAULT_NFC_TOKEN_COUNT =
  (uint8_t)(sizeof(DEFAULT_NFC_TOKEN_HASHES) / sizeof(DEFAULT_NFC_TOKEN_HASHES[0]));

} // namespace EepromLayout
} // namespace Kiosk
