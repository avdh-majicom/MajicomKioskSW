// KioskEepromEditor.cpp
// ----------------------
// Call-once EEPROM editor module for the kiosk firmware.
//
// Design intent:
// - This module is invoked after boot if a specific “boot chord” is pressed.
// - It does not return to the caller; exit is via watchdog reset.
// - All I/O is assumed already initialised by the host firmware. For standalone testing,
//   cfg.initPins can be set to true to configure only the buttons used by this module.
//
// Hardware / UI:
// - OLED: U8x8 text mode, 16 columns x 16 rows (0..15).
// - LCD: 16x4 HD44780 over I2C.
// - Buttons: active-low, external pullups + RC debouncing.
// - PN532: held in RESET (LOW) except during explicit NFC scans, to prevent PN532
//   from holding the I2C bus in a bad state.
//
// LCD (EEPROM editor mode):
//   Line0 = "EEPROM Editor Mode"  (matches OLED title)
//   Line1 = "----------------"
//   Line2 = "HWID:" + full alpha/numeric string of the HWID
//   Line3 = "SWID:" + build timestamp + status char
//   Line1 = used for transient “Saving/Saved” or NFC status, otherwise blank.
//
// OLED layout (typical):
//   Row0  : Title
//   Row2  : HWID (text)
//   Row3  : SWID (build timestamp + status)
//   Row5-11 : Menu list (varies by layer)
//   Row11 : Header line for certain edit layers (solenoid/backwash/dispensed) to reduce flicker
//   Row12 : Value/details line for current selection/edit
//   Row14 : Hint line
//   Row15 : Status line (Saving/Saved/Resetting/Completed)
//
// Functional overview:
// - Edit / inspect EEPROM-backed settings:
//     * PCB/HWID
//     * Water dispense control (duration/pulses/mode)
//     * Solenoid PWM profiles
//     * Backwash + KlaranUV parameters
//     * Dispensed counters
//     * NFC token table (browse/add/delete; writes can be disabled)
// - UI conventions:
//     * UP/DOWN: menu navigation; in numeric edit UP increases and DOWN decreases
//     * Long SELECT: save current edit / confirm destructive actions
//     * Long BACK: exit via watchdog reset (no return to caller)
// - Unit display notes:
//     * Water dispense duration stored as 100ms units, displayed as seconds with 0.1s resolution
//     * Backwash auto duration stored as 5s units
// Note on RAM usage:
// - This module uses static storage for its UI state and caches. That RAM is allocated for the
//   whole program lifetime (not only while run() is executing). If you need to reclaim RAM when
//   the editor is not active, this would need refactoring to allocate state dynamically.

#include "KioskEepromEditor.h"

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>

#include "KioskIO.h"

namespace Kiosk {
namespace KioskEepromEditor {

// -------------------- Timing constants --------------------
// Button repeat behaviour:
// - initial press triggers one action immediately
// - after BTN_REPEAT_DELAY_MS, repeat begins at BTN_REPEAT_RATE_MS
// - after BTN_FAST_THRESHOLD, repeat rate increases to BTN_REPEAT_FAST_MS
static constexpr unsigned long BTN_REPEAT_DELAY_MS = 2000UL;
static constexpr unsigned long BTN_REPEAT_RATE_MS  = 100UL;
static constexpr unsigned long BTN_REPEAT_FAST_MS  = 50UL;
static constexpr unsigned long BTN_FAST_THRESHOLD  = 5000UL;

// Long-press thresholds:
// - SELECT long press typically used to “save” current edit
// - BACK long press triggers immediate watchdog reset
static constexpr unsigned long BTN_SELECT_LONG_MS  = 3000UL;
static constexpr unsigned long BTN_BACK_LONG_MS    = 4000UL;

// “Saved !” persistence on OLED + LCD line1.
// Keep long enough that an operator sees it reliably.
// Standard hold time: 3 seconds.
static constexpr unsigned long SAVED_DISPLAY_MS    = 3000UL;

// NFC UX timing:
// - per-operation message display (success/fail) on OLED/LCD
// - LCD line1 “result” is kept longer than OLED message
static constexpr unsigned long NFC_MESSAGE_DISPLAY_MS  = 3000UL;
static constexpr unsigned long NFC_LCD_MESSAGE_HOLD_MS = 30000UL;
static constexpr unsigned long NFC_SCAN_TIMEOUT_SEC    = 10;

// Special return value from readNfcHash() when the operator presses BACK.
// This avoids treating a user-requested exit as a token read error.
static constexpr uint32_t NFC_HASH_ABORT = 0xFFFFFFFFUL;

// EEPROM full reset behaviour:
// - user must hold SELECT for >= 10s (REINIT_START_HOLD_MS) to start reset
// - after reset completes, “Completed” is shown and held until button release or >= 3s
static constexpr unsigned long REINIT_START_HOLD_MS    = 10000UL;
static constexpr unsigned long REINIT_COMPLETE_WAIT_MS = 3000UL;

// -------------------- Helpers: clamp --------------------
// Simple clamp used throughout for editing numeric values.
static int32_t clamp32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// -------------------- Button helpers --------------------
// RepeatButton: for UP/DOWN buttons with auto-repeat.
// direction: the “menu direction” (UP=-1, DOWN=+1). For numeric editing, we invert in handleUpDown().
struct RepeatButton {
  uint8_t pin;
  bool lastState;
  bool isRepeating;
  unsigned long pressStartMs;
  unsigned long lastRepeatMs;
  int8_t direction; // -1 or +1 (menu)
  uint16_t step100msApplied;
};

// LongPressButton: for SELECT/BACK with short/long press actions.
struct LongPressButton {
  uint8_t pin;
  bool lastState;
  bool longHandled;
  unsigned long pressStartMs;
};

// -------------------- Layers --------------------
// Each layer represents a “screen mode” (menu or edit).
enum : uint8_t {
  LAYER_TOP=0,
  LAYER_WD_MENU=1,
  LAYER_WD_EDIT=2,
  LAYER_SOL_MENU=3,
  LAYER_SOL_PARAM_EDIT=4,
  LAYER_PCB=5,
  LAYER_DISPENSED_MENU=6,
  LAYER_DISPENSED_EDIT=7,
  LAYER_BACKWASH_MENU=8,
  LAYER_BACKWASH_EDIT=9,
  LAYER_NFC_MENU=10,
  LAYER_NFC_BROWSE=11,
  LAYER_NFC_ADD=12,
  LAYER_NFC_DEL=13,
  LAYER_REINIT=14,
  // Kiosk parameter edit category.
  LAYER_KIOSKPARM_MENU=15,
  // Hydraulic parameter edit category.
  LAYER_HYDRPARM_MENU=16,
  // KlaranUV configuration (under KioskParm Edit).
  LAYER_KLARAN_MENU=17,
  LAYER_KLARAN_EDIT=18,
  // Water temperature sense selection (under KioskParm Edit).
  LAYER_WATERTEMP_EDIT=19,
  // Coin acceptor fitted flag (under KioskParm Edit).
  LAYER_COINACC_EDIT=20,
  // SensorBypass timing configuration (under Hydr Parm Edit).
  LAYER_SENSORBYP_MENU=21,
  LAYER_SENSORBYP_EDIT=22,
  // RTC configuration/status (under KioskParm Edit).
  LAYER_RTC_CONFIG=23,
  // NFC reader timing parameters (under KioskParm Edit -> NFC Parms).
  LAYER_NFC_PARMS_MENU=24,
  LAYER_NFC_PARMS_EDIT=25,
  // Water circulation control (under Hydr Parm Edit).
  LAYER_WCIRC_MENU=26,
  LAYER_WCIRC_EDIT=27,
  // Periodic flush configuration (under Hydr Parm Edit).
  LAYER_PFLUSH_MENU=28,
  LAYER_PFLUSH_EDIT=29
};

// Top menu indices (order matches TOP_LABELS).
// Note: parameter sections are grouped under category menus.
//   TOP_KIOSKPARM -> LAYER_KIOSKPARM_MENU.
enum : uint8_t {
  TOP_PCBHWID=0,
  TOP_DISPENSED=1,
  TOP_KIOSKPARM=2,
  TOP_HYDRPARM=3,
  TOP_NFC_MANAGE=4,
  TOP_REINIT=5,
  TOP_COUNT=6
};


// KioskParm Edit menu (subcategory under TOP_KIOSKPARM).
enum : uint8_t {
  KPARM_MENU_SOLENOID=0,
  KPARM_MENU_KLARANUV=1,
  KPARM_MENU_WATERTEMP=2,
  KPARM_MENU_COINACC=3,
  KPARM_MENU_RTC=4,
  KPARM_MENU_NFC_PARMS=5,
  KPARM_MENU_COUNT=6
};
// Hydr Parm Edit menu (subcategory under TOP_HYDRPARM).
enum : uint8_t {
  HPARM_MENU_BACKWASH=0,
  HPARM_MENU_SENSORBYP=1,
  HPARM_MENU_WATERCIRC=2,
  HPARM_MENU_WATERDISP=3,
  HPARM_MENU_PFLUSH=4,
  HPARM_MENU_COUNT=5
};
// WD menu/edit fields.
enum : uint8_t {
  WD_MENU_MODESEL=0,
  WD_MENU_TIMED=1,
  WD_MENU_PULSES=2,
  WD_MENU_PREDISP_CIRCT=3,
  WD_MENU_PREDISP_CIRCP=4,
  WD_MENU_PREDISP_PURGT=5,
  WD_MENU_PREDISP_PURGP=6,
  WD_MENU_COUNT=7
};
enum : uint8_t {
  WD_FIELD_MODESEL=0,
  WD_FIELD_TIME=1,
  WD_FIELD_PULSES=2,
  WD_FIELD_PREDISP_CIRCT=3,
  WD_FIELD_PREDISP_CIRCP=4,
  WD_FIELD_PREDISP_PURGT=5,
  WD_FIELD_PREDISP_PURGP=6,
  WD_FIELD_COUNT=7
};

// Solenoid params.
enum : uint8_t {
  SOL_PARAM_STARTPWM=0,
  SOL_PARAM_HOLDPWM=1,
  SOL_PARAM_SWDELAY=2,
  SOL_PARAM_COUNT=3
};

// Dispensed menu.
enum : uint8_t { DISP_MENU_APP=0, DISP_MENU_COIN=1, DISP_MENU_NFC=2, DISP_MENU_BYPASS=3, DISP_MENU_TOTAL=4, DISP_MENU_COUNT=5 };

// Backwash menu (Frequency removed).
enum : uint8_t {
  BW_MENU_DUR=0,
  BW_MENU_NDISP=1,
  BW_MENU_COUNTER=2,
  BW_MENU_DAILY_DUR=3,
  BW_MENU_DAILY_TIME=4,
  BW_MENU_MAN_SHORT=5,
  BW_MENU_MAN_LONG=6,
  BW_MENU_COUNT=7
};

// KlaranUV menu/edit fields.
enum : uint8_t { UV_MENU_OK_DELAY=0, UV_MENU_MAX_ONTIME=1, UV_MENU_COUNT=2 };
// SensorBypass menu/edit fields.
enum : uint8_t {
  SBYP_MENU_DUR=0,
  SBYP_MENU_PERIOD=1,
  SBYP_MENU_MAN_SHORT=2,
  SBYP_MENU_MAN_LONG=3,
  SBYP_MENU_COUNT=4
};
// Water circulation menu/edit fields.
enum : uint8_t {
  WCIRC_MENU_AUTODUR=0,
  WCIRC_MENU_AUTOPERIOD=1,
  WCIRC_MENU_MANSHORT=2,
  WCIRC_MENU_MANLONG=3,
  WCIRC_MENU_COUNT=4
};
// Periodic Flush menu/edit fields.
enum : uint8_t {
  PFLUSH_MENU_DNF_PERIOD=0,
  PFLUSH_MENU_DNF_DURAT=1,
  PFLUSH_MENU_PREFLSH_BEEP_DELY=2,
  PFLUSH_MENU_BEEP_PERIOD=3,
  PFLUSH_MENU_BEEP_ON=4,
  PFLUSH_MENU_BEEP_COUNT=5,
  PFLUSH_MENU_COUNT=6
};

// NFC reader timing parameters menu/edit fields.
enum : uint8_t { NFC_PARMS_INIT_DELAY=0, NFC_PARMS_READ_DURATION=1, NFC_PARMS_INTER_DELAY=2, NFC_PARMS_COUNT=3 };

// NFC menu.
enum : uint8_t { NFC_MENU_BROWSE=0, NFC_MENU_ADD=1, NFC_MENU_DEL=2, NFC_MENU_PACK=3, NFC_MENU_REMOVE_ALL=4, NFC_MENU_COUNT=5 };

// NFC browse/delete prompt state.
enum : uint8_t { TOKEN_STATE_BROWSE=0, TOKEN_STATE_DELETE_PROMPT=1 };

// NFC operation state machine for ADD/DEL:
// - To reduce false reads, a token is accepted only after two consecutive scans
//   return the same 32-bit hash (SCAN1 then SCAN2).
enum : uint8_t {
  NFC_OP_STATE_IDLE=0,
  NFC_OP_STATE_SCAN1=1,
  NFC_OP_STATE_SCAN2=2,
  NFC_OP_STATE_SUCCESS=3,
  NFC_OP_STATE_MISMATCH=4,
  NFC_OP_STATE_TIMEOUT=5,
  NFC_OP_STATE_DUPLICATE=6,
  NFC_OP_STATE_NOT_FOUND=7,
  NFC_OP_STATE_FULL=8
};

// -------------------- PROGMEM strings --------------------
// Put most static UI strings in PROGMEM to save SRAM.
static const char STR_EEPROM_EDIT[] PROGMEM = "EEPROM Edit Mode";
static const char STR_DASHES[] PROGMEM     = "----------------";
static const char STR_HINT[] PROGMEM       = "< Select    +/-<";
static const char STR_SAVING[] PROGMEM     = "   Saving....   ";
static const char STR_SAVED[] PROGMEM      = "    Saved !!    ";
static const char STR_BUSY[] PROGMEM       = "Packing!";
static const char STR_RESETTING[] PROGMEM  = "Resetting";
static const char STR_COMPLETED[] PROGMEM  = "Completed";

static const char STR_TOP_PCBHWID[] PROGMEM   = "PCB HW ID";
static const char STR_TOP_WATERDISP[] PROGMEM = "Water Disp Ctrl";
static const char STR_TOP_SOLENOID[] PROGMEM  = "Solenoid Config";
static const char STR_TOP_BACKWASH[] PROGMEM  = "Backwash Config";
static const char STR_TOP_DISPENSED[] PROGMEM = "Dispensed Units";
static const char STR_TOP_KIOSKPARM[] PROGMEM  = "Kiosk Parm Edit";
static const char STR_TOP_HYDRPARM[] PROGMEM = "Hydra Parm Edit";
static const char STR_TOP_WATERCIRC[] PROGMEM = "Water Circ Ctrl";
static const char STR_TOP_PFLUSH[] PROGMEM = "Periodic Flush";
static const char STR_TOP_KLARANUV[] PROGMEM   = "KlaranUV Config";
static const char STR_TOP_WATERTEMPSENSE[] PROGMEM = "WaterTempSensor";
static const char STR_KPARM_COINACC[] PROGMEM  = "CoinAcc Config";
static const char STR_HPARM_SENSORBYP[] PROGMEM = "SensBypass Edit";
static const char STR_KPARM_RTC[] PROGMEM      = "RTC Config";
static const char STR_KPARM_NFC_PARMS[] PROGMEM = "NFC Params Mgt";
static const char STR_TOP_NFC[] PROGMEM       = "NFC Token Mgt";
static const char STR_TOP_REINIT[] PROGMEM    = "Re-init EEPROM";

static const char* const TOP_LABELS[] PROGMEM = {
  STR_TOP_PCBHWID,
  STR_TOP_DISPENSED,
  STR_TOP_KIOSKPARM,
  STR_TOP_HYDRPARM,
  STR_TOP_NFC,
  STR_TOP_REINIT
};



// KioskParm Edit submenu labels (shown after selecting "KioskParm Edit" on the top menu).
// These were formerly top-level menu items.
static const char* const KPARM_LABELS[] PROGMEM = {
  STR_TOP_SOLENOID,   // "Solenoid Config"
  STR_TOP_KLARANUV,   // "KlaranUV Config"
  STR_TOP_WATERTEMPSENSE, // "WaterTempSense"
  STR_KPARM_COINACC,  // "CoinAcc Config"
  STR_KPARM_RTC,       // "Config RTC"
  STR_KPARM_NFC_PARMS // "NFC Parms"
};
static const char* const HPARM_LABELS[] PROGMEM = {
  STR_TOP_BACKWASH,     // "Backwash Config"
  STR_HPARM_SENSORBYP,  // "SensBypass Edit"
  STR_TOP_WATERCIRC,    // "Water Circ Ctrl"
  STR_TOP_WATERDISP,    // "Water Disp Ctrl"
  STR_TOP_PFLUSH        // "Periodic Flush"
};

// Water temperature sense edit header.
static const char STR_WTEMP_HDR[] PROGMEM = "WaterTempSensor";
static const char STR_COINACC_HDR[] PROGMEM = "Coin Acceptor";

static const char STR_WD_MODESEL[] PROGMEM = "Disp Ctrl Mode";
static const char STR_WD_TIMED[] PROGMEM   = "TimedDisp Dur";
static const char STR_WD_PULSES[] PROGMEM  = "PulseDisp Count";
static const char STR_WD_PREDISP_CIRCT[] PROGMEM = "PreDispCircT";
static const char STR_WD_PREDISP_CIRCP[] PROGMEM = "PreDispCircP";
static const char STR_WD_PREDISP_PURGT[] PROGMEM = "PreDispPurgeT";
static const char STR_WD_PREDISP_PURGP[] PROGMEM = "PreDispPurgeP";
static const char* const WD_MENU_LABELS[] PROGMEM = {
  STR_WD_MODESEL,
  STR_WD_TIMED,
  STR_WD_PULSES,
  STR_WD_PREDISP_CIRCT,
  STR_WD_PREDISP_CIRCP,
  STR_WD_PREDISP_PURGT,
  STR_WD_PREDISP_PURGP
};

// Water Disp Ctrl edit headers (OLED row 11)
static const char STR_WD_HDR_MODE[] PROGMEM   = "Disp Ctrl Mode";
static const char STR_WD_HDR_TIME[] PROGMEM   = "TimedDisp Dur";
static const char STR_WD_HDR_PULSES[] PROGMEM = "PulseDisp Count";
static const char STR_WD_HDR_PREDISP_CIRCT[] PROGMEM = "PreDispCircT";
static const char STR_WD_HDR_PREDISP_CIRCP[] PROGMEM = "PreDispCircP";
static const char STR_WD_HDR_PREDISP_PURGT[] PROGMEM = "PreDispPurgeT";
static const char STR_WD_HDR_PREDISP_PURGP[] PROGMEM = "PreDispPurgeP";

// Solenoid names are operator-facing and should remain stable if possible.
static const char STR_SOL0[] PROGMEM = "BackWashSol";
static const char STR_SOL1[] PROGMEM = "WaterSenseBypSol";
static const char STR_SOL2[] PROGMEM = "InletWaterSol";
static const char STR_SOL3[] PROGMEM = "WaterDispSol";
static const char* const SOLENOID_NAMES[] PROGMEM = { STR_SOL0, STR_SOL1, STR_SOL2, STR_SOL3 };

// Solenoid parameter names (kept <= 16 chars).
static const char STR_SOLP_START[] PROGMEM = "StartPWM";
static const char STR_SOLP_HOLD[]  PROGMEM = "HoldPWM";
static const char STR_SOLP_SW[]    PROGMEM = "SwOnDelay";
static const char* const SOL_PARAM_NAMES[] PROGMEM = { STR_SOLP_START, STR_SOLP_HOLD, STR_SOLP_SW };

// Dispensed Units menu labels (requested):
// NOTE: These are used in the menu list (rows 5..11). Long strings may be truncated visually.
static const char STR_DISP_MENU_APP[]    PROGMEM = "App Disp";
static const char STR_DISP_MENU_COIN[]   PROGMEM = "Coin Disp";
static const char STR_DISP_MENU_NFC[]    PROGMEM = "NFC Token Disp";
static const char STR_DISP_MENU_BYPASS[] PROGMEM = "Bypassed Disp";
static const char STR_DISP_TOTAL[] PROGMEM = "TOTAL Dispensed";
static const char* const DISPENSE_MENU_NAMES[] PROGMEM = {
  STR_DISP_MENU_APP,
  STR_DISP_MENU_COIN,
  STR_DISP_MENU_NFC,
  STR_DISP_MENU_BYPASS,
  STR_DISP_TOTAL
};
// STR_DISP_TOTAL is used both in the menu and elsewhere in the editor.

// “Short” labels (currently reused for edit header row 11 and for compact value display in menu mode).
// If you want a truly short on-row-12 prefix, define separate strings here.
static const char* const DISPENSE_SHORT_NAMES[] PROGMEM = {
  STR_DISP_MENU_APP, STR_DISP_MENU_COIN, STR_DISP_MENU_NFC, STR_DISP_MENU_BYPASS
};

// Backwash menu labels.
static const char STR_BW_DUR[] PROGMEM     = "BW AutoDuration";
static const char STR_BW_NDISP[] PROGMEM   = "BW AutoAfter N";
static const char STR_BW_COUNTER[] PROGMEM = "BW DispCount";
static const char STR_BW_MAN_SHORT[] PROGMEM = "BW MAN Short";
static const char STR_BW_MAN_LONG[] PROGMEM  = "BW MAN Long";
static const char STR_BW_DAILY_TIME[] PROGMEM = "BW Daily Time";
static const char STR_BW_DAILY_DUR[] PROGMEM  = "BW Daily Duratn";
static const char* const BW_MENU_LABELS[] PROGMEM = {
  STR_BW_DUR,
  STR_BW_NDISP,
  STR_BW_COUNTER,
  STR_BW_DAILY_DUR,
  STR_BW_DAILY_TIME,
  STR_BW_MAN_SHORT,
  STR_BW_MAN_LONG
};

// Backwash edit header lines (row 11).
static const char STR_BW_HDR_DUR[]     PROGMEM = "BW AutoDuration";
static const char STR_BW_HDR_NDISP[]   PROGMEM = "BW Auto After N";
static const char STR_BW_HDR_COUNTER[] PROGMEM = "BW Disp Count";
static const char STR_BW_HDR_MAN_SHORT[] PROGMEM = "BW MAN Short";
static const char STR_BW_HDR_MAN_LONG[]  PROGMEM = "BW MAN Long";
static const char STR_BW_HDR_DAILY_TIME[] PROGMEM = "Daily BW Time";
static const char STR_BW_HDR_DAILY_DUR[] PROGMEM  = "Daily BW Dur";

// KlaranUV labels.
static const char STR_UV_OK_DELAY[] PROGMEM   = "UV OK Delay";
static const char STR_UV_MAX_ONTIME[] PROGMEM = "UV MAX ON time";
// SensorBypass labels.
static const char STR_SBYP_DUR[] PROGMEM = "SensByp Duratn";
static const char STR_SBYP_PERIOD[] PROGMEM = "SensByp Period";
static const char STR_SBYP_MAN_SHORT[] PROGMEM = "MAN Bypass Short";
static const char STR_SBYP_MAN_LONG[] PROGMEM = "MAN Bypass Long";
static const char* const SBYP_MENU_LABELS[] PROGMEM = {
  STR_SBYP_DUR,
  STR_SBYP_PERIOD,
  STR_SBYP_MAN_SHORT,
  STR_SBYP_MAN_LONG
};
// Water circulation labels.
static const char STR_WCIRC_AUTODUR[] PROGMEM = "AutoCirc Duratn";
static const char STR_WCIRC_AUTOPERIOD[] PROGMEM = "AutoCirc Period";
static const char STR_WCIRC_MANSHORT[] PROGMEM = "ManCircDur Shrt";
static const char STR_WCIRC_MANLONG[] PROGMEM = "ManCircDur Long";
static const char* const WCIRC_MENU_LABELS[] PROGMEM = {
  STR_WCIRC_AUTODUR,
  STR_WCIRC_AUTOPERIOD,
  STR_WCIRC_MANSHORT,
  STR_WCIRC_MANLONG
};
// Periodic flush labels.
static const char STR_PFLUSH_DNF_PERIOD[] PROGMEM = "DNF Rept Period";
static const char STR_PFLUSH_DNF_DURAT[] PROGMEM = "DNF DispDurat";
static const char STR_PFLUSH_PREFLSH_BEEP_DELY[] PROGMEM = "PreFlshBeepDely";
static const char STR_PFLUSH_BEEP_PERIOD[] PROGMEM = "Beep Period";
static const char STR_PFLUSH_BEEP_ON[] PROGMEM = "Beep ON Time";
static const char STR_PFLUSH_BEEP_COUNT[] PROGMEM = "Beep Count";
static const char* const PFLUSH_MENU_LABELS[] PROGMEM = {
  STR_PFLUSH_DNF_PERIOD,
  STR_PFLUSH_DNF_DURAT,
  STR_PFLUSH_PREFLSH_BEEP_DELY,
  STR_PFLUSH_BEEP_PERIOD,
  STR_PFLUSH_BEEP_ON,
  STR_PFLUSH_BEEP_COUNT
};

// KlaranUV edit header lines (row 11).
static const char STR_UV_HDR_OK_DELAY[] PROGMEM   = "UV OK Delay";
static const char STR_UV_HDR_MAX_ONTIME[] PROGMEM = "UV MAX ON time";
static const char STR_SBYP_HDR_DUR[] PROGMEM = "SensByp Duratn";
static const char STR_SBYP_HDR_PERIOD[] PROGMEM = "SensByp Period";
static const char STR_SBYP_HDR_MAN_SHORT[] PROGMEM = "MAN Bypass Short";
static const char STR_SBYP_HDR_MAN_LONG[] PROGMEM = "MAN Bypass Long";
static const char STR_WCIRC_HDR_AUTODUR[] PROGMEM = "AutoCirc Duratn";
static const char STR_WCIRC_HDR_AUTOPERIOD[] PROGMEM = "AutoCirc Period";
static const char STR_WCIRC_HDR_MANSHORT[] PROGMEM = "ManCircDur Shrt";
static const char STR_WCIRC_HDR_MANLONG[] PROGMEM = "ManCircDur Long";
static const char STR_PFLUSH_HDR_DNF_PERIOD[] PROGMEM = "DNF Rept Period";
static const char STR_PFLUSH_HDR_DNF_DURAT[] PROGMEM = "DNF DispDurat";
static const char STR_PFLUSH_HDR_PREFLSH_BEEP_DELY[] PROGMEM = "PreFlshBeepDely";
static const char STR_PFLUSH_HDR_BEEP_PERIOD[] PROGMEM = "Beep Period";
static const char STR_PFLUSH_HDR_BEEP_ON[] PROGMEM = "Beep ON Time";
static const char STR_PFLUSH_HDR_BEEP_COUNT[] PROGMEM = "Beep Count";

static const char* const UV_MENU_LABELS[] PROGMEM = { STR_UV_OK_DELAY, STR_UV_MAX_ONTIME };

// NFC reader timing parameters (submenu under KioskParm Edit -> NFC Parms).
static const char STR_NFCP_INIT_DELAY[] PROGMEM = "NFC Init Delay";
static const char STR_NFCP_READ_DURATION[] PROGMEM = "NFC Scan Duratn";
static const char STR_NFCP_INTER_DELAY[] PROGMEM = "Inter NFC Delay";
static const char* const NFC_PARMS_LABELS[] PROGMEM = { STR_NFCP_INIT_DELAY, STR_NFCP_READ_DURATION, STR_NFCP_INTER_DELAY };

// NFC menu labels.
static const char STR_NFC_BROWSE[] PROGMEM     = "Browse/Delete";
static const char STR_NFC_ADD[] PROGMEM        = "NFC Add Token";
static const char STR_NFC_DEL[] PROGMEM        = "NFC Del Token";
static const char STR_NFC_PACK[] PROGMEM       = "Pack NFC Table";
static const char STR_NFC_REMOVE_ALL[] PROGMEM = "Del ALL Tokens";
static const char* const NFC_MENU_LABELS[] PROGMEM = { STR_NFC_BROWSE, STR_NFC_ADD, STR_NFC_DEL, STR_NFC_PACK, STR_NFC_REMOVE_ALL };

// LCD NFC overlays (line1).
static const char STR_LCD_WAITING[] PROGMEM   = "Waiting for NFC ";
static const char STR_LCD_TOKN_ADD[] PROGMEM  = "ToknAdd:";
static const char STR_LCD_TOKN_DEL[] PROGMEM  = "ToknDel:";
static const char STR_LCD_TOKN_DUP[] PROGMEM  = "ToknDup:";
static const char STR_LCD_TOKN_INV[] PROGMEM  = "NFC Tokn Invalid";
static const char STR_LCD_TOKN_NOT[] PROGMEM  = "ToknNot:";
static const char STR_LCD_TOKN_FULL[] PROGMEM = "Token Table Full";
static const char STR_LCD_CLEAR[] PROGMEM     = "                ";

static const char STR_REINIT_PROMPT[] PROGMEM = "Reinit? Sel=Yes";
static const char STR_REMOVEALL_PROMPT[] PROGMEM = "RemoveAll? Sel=Y";

// -------------------- Per-run editor state --------------------
// The EEPROM editor is only invoked during a boot-time maintenance path and exits via watchdog reset.
// To minimise the resident SRAM footprint when the editor is not used, all mutable editor state lives
// in an EditorState object allocated on the run() stack.
//
// Internal helpers were originally written against file-scope g_* variables. For minimal behavioural risk,
// this refactor maps those legacy names onto the active EditorState instance via macros.

struct EditorState {
  const Config* cfg = nullptr;
  uint8_t currentLayer = LAYER_TOP;
  uint8_t topCursor = TOP_PCBHWID;
  uint8_t kioskParmCursor = KPARM_MENU_SOLENOID;
  uint8_t hydrParmCursor = HPARM_MENU_BACKWASH;
  uint8_t uvMenuCursor = UV_MENU_OK_DELAY;
  uint8_t sensorBypMenuCursor = SBYP_MENU_DUR;
  uint8_t currentSensorBypIndex = SBYP_MENU_DUR;
  uint8_t wcircMenuCursor = WCIRC_MENU_AUTODUR;
  uint8_t currentWcircIndex = WCIRC_MENU_AUTODUR;
  uint8_t pflushMenuCursor = PFLUSH_MENU_DNF_PERIOD;
  uint8_t currentPflushIndex = PFLUSH_MENU_DNF_PERIOD;
  uint8_t nfcParmsMenuCursor = NFC_PARMS_INIT_DELAY;
  uint8_t currentNfcParmIndex = NFC_PARMS_INIT_DELAY;
  uint8_t wdMenuCursor = WD_MENU_MODESEL;
  uint8_t wdMenuTop = 0;
  uint8_t wdEditField = WD_FIELD_MODESEL;
  uint8_t solMenuCursor = 0;
  uint8_t currentSolenoidIndex = 0;
  uint8_t solParamCursor = SOL_PARAM_STARTPWM;
  uint8_t dispMenuCursor = DISP_MENU_APP;
  uint8_t currentDispenseIndex = 0;
  uint8_t bwMenuCursor = BW_MENU_DUR;
  uint8_t currentBwIndex = BW_MENU_DUR;
  uint8_t nfcMenuCursor = NFC_MENU_BROWSE;
  uint16_t tokenBrowseIndex = 0;
  uint8_t tokenState = TOKEN_STATE_BROWSE;
  bool tokenDeleteConfirm = false;
  bool removeAllPromptActive = false;
  bool removeAllConfirm = false;
  bool reinitPromptActive = false;
  bool reinitConfirmYes = false;
  uint8_t workingPcbId = 0;
  bool showSavingMessage = false;
  bool showSavedMessage = false;
  unsigned long savedStartMs = 0;
  unsigned long nfcOpMessageMs = 0;
  char oledRowCache[16][17] = { {0} };
  bool lcdStatusActive = false;
  unsigned long lcdStatusStartMs = 0;
  bool reinitPostWaitActive = false;
  unsigned long reinitPostWaitStartMs = 0;
  KioskEeprom::MeasuredProfile wdProfileCache {};
  bool wdCacheDirty = false;
  uint8_t preDispCircTCache = 0;
  uint8_t preDispCircPCache = 0;
  uint8_t preDispPurgTCache = 0;
  uint8_t preDispPurgPCache = 0;
  bool preDispDirty = false;
  KioskEeprom::PwmSolenoidProfile solProfileCache {};
  bool solCacheDirty = false;
  uint32_t dispCounterCache = 0;
  bool dispCacheDirty = false;
  uint8_t backwashDurCache = 0;
  uint8_t backwashNDispCache = 0;
  uint16_t backwashCounterCache = 0;
  uint8_t backwashManShortCache = 0;
  uint8_t backwashManLongCache = 0;
  uint16_t backwashDailyTimeCache = 0;
  uint8_t backwashDailyDurCache = 0;
  bool bwCacheDirty = false;
  bool uvCacheDirty = false;
  uint8_t uvOkDelayCache = 0;       // 10ms units
  uint8_t uvMaxOntimeCache = 0;     // minutes
  uint8_t currentUvIndex = UV_MENU_OK_DELAY;
  uint8_t sensorBypDurCache = 0;    // 100ms units
  uint8_t sensorBypPeriodCache = 0; // minutes
  uint8_t sensorBypManShortCache = 0; // 100ms units
  uint8_t sensorBypManLongCache = 0;  // 10s units
  bool sensorBypDirty = false;
  uint8_t autoCircDurCache = 0;       // 1 minute units
  uint8_t autoCircPeriodCache = 0;    // 10 minute units
  uint8_t manCircShortCache = 1;      // 1 minute units
  uint8_t manCircLongCache = 1;       // 10 minute units
  bool wcircDirty = false;
  uint8_t dnfReptPeriodCache = 0;     // 1 minute units
  uint8_t dnfReptDuratCache = 0;      // 100ms units
  uint8_t preFlshBeepDelyCache = 0;   // 1-second units
  uint8_t beepOnTimeCache = 0;        // 10ms units
  uint8_t beepOffTimeCache = 0;       // 100ms units
  uint8_t beepCountCache = 0;         // count units
  bool pflushDirty = false;
  uint8_t waterTempSenseCache = 1;  // 1=T1, 2=T2
  bool waterTempSenseDirty = false;
  bool coinAccFittedCache = false;
  bool coinAccFittedDirty = false;
  bool rtcPresent = false;
  bool rtcDirty = false;
  char rtcDigits[14] = {};   // YYYYMMDDHHMMSS, no separators
  uint8_t rtcClusterIndex = 0; // 0..5 (Y,M,D,H,M,S)
  char rtcLine5[17] = {};
  char rtcLine6[17] = {};
  char rtcLine7[17] = {};
  char rtcLine13[17] = {};
  char bwDailyAutoLine[17] = {};
  char bwDailyTrigLine[17] = {};
  unsigned long rtcLastPollMs = 0;
  uint8_t rtcLastSecond = 0xFF;
  uint16_t nfcInitDelayMsCache = 0;
  uint16_t nfcReadDurationMsCache = 0;
  uint16_t nfcInterDelayMsCache = 0;
  bool nfcParmsDirty = false;
  bool pcbIdDirty = false;
  uint8_t nfcOpState = NFC_OP_STATE_IDLE;
  uint32_t nfcHash1 = 0;
  uint32_t nfcHash2 = 0;
  bool lcdNfcMessageActive = false;
  unsigned long lcdNfcMessageMs = 0;
  bool reinitHoldActive = false;
  unsigned long reinitHoldStartMs = 0;
  RepeatButton btnUp {};
  RepeatButton btnDown {};
  LongPressButton btnSelect {};
  LongPressButton btnBack {};
};

static EditorState* S = nullptr;  // Valid only while run() is executing.

// Legacy name compatibility (do not use outside run()).
#define G (S->cfg)
#define g_currentLayer (S->currentLayer)
#define g_topCursor (S->topCursor)
#define g_kioskParmCursor (S->kioskParmCursor)
#define g_hydrParmCursor (S->hydrParmCursor)
#define g_uvMenuCursor (S->uvMenuCursor)
#define g_sensorBypMenuCursor (S->sensorBypMenuCursor)
#define g_currentSensorBypIndex (S->currentSensorBypIndex)
#define g_wcircMenuCursor (S->wcircMenuCursor)
#define g_currentWcircIndex (S->currentWcircIndex)
#define g_pflushMenuCursor (S->pflushMenuCursor)
#define g_currentPflushIndex (S->currentPflushIndex)
#define g_nfcParmsMenuCursor (S->nfcParmsMenuCursor)
#define g_currentNfcParmIndex (S->currentNfcParmIndex)
#define g_uvOkDelayCache (S->uvOkDelayCache)
#define g_uvMaxOntimeCache (S->uvMaxOntimeCache)
#define g_nfcInitDelayMsCache (S->nfcInitDelayMsCache)
#define g_nfcReadDurationMsCache (S->nfcReadDurationMsCache)
#define g_nfcInterDelayMsCache (S->nfcInterDelayMsCache)
#define g_nfcParmsDirty (S->nfcParmsDirty)
#define g_waterTempSenseCache (S->waterTempSenseCache)
#define g_waterTempSenseDirty (S->waterTempSenseDirty)
#define g_coinAccFittedCache (S->coinAccFittedCache)
#define g_coinAccFittedDirty (S->coinAccFittedDirty)
#define g_rtcPresent (S->rtcPresent)
#define g_rtcDirty (S->rtcDirty)
#define g_rtcDigits (S->rtcDigits)
#define g_rtcClusterIndex (S->rtcClusterIndex)
#define g_rtcLine5 (S->rtcLine5)
#define g_rtcLine6 (S->rtcLine6)
#define g_rtcLine7 (S->rtcLine7)
#define g_rtcLine13 (S->rtcLine13)
#define g_bwDailyAutoLine (S->bwDailyAutoLine)
#define g_bwDailyTrigLine (S->bwDailyTrigLine)
#define g_rtcLastPollMs (S->rtcLastPollMs)
#define g_rtcLastSecond (S->rtcLastSecond)
#define g_wdMenuCursor (S->wdMenuCursor)
#define g_wdMenuTop (S->wdMenuTop)
#define g_wdEditField (S->wdEditField)
#define g_solMenuCursor (S->solMenuCursor)
#define g_currentSolenoidIndex (S->currentSolenoidIndex)
#define g_solParamCursor (S->solParamCursor)
#define g_dispMenuCursor (S->dispMenuCursor)
#define g_currentDispenseIndex (S->currentDispenseIndex)
#define g_bwMenuCursor (S->bwMenuCursor)
#define g_currentBwIndex (S->currentBwIndex)
#define g_nfcMenuCursor (S->nfcMenuCursor)
#define g_tokenBrowseIndex (S->tokenBrowseIndex)
#define g_tokenState (S->tokenState)
#define g_tokenDeleteConfirm (S->tokenDeleteConfirm)
#define g_removeAllPromptActive (S->removeAllPromptActive)
#define g_removeAllConfirm (S->removeAllConfirm)
#define g_reinitPromptActive (S->reinitPromptActive)
#define g_reinitConfirmYes (S->reinitConfirmYes)
#define g_workingPcbId (S->workingPcbId)
#define g_showSavingMessage (S->showSavingMessage)
#define g_showSavedMessage (S->showSavedMessage)
#define g_savedStartMs (S->savedStartMs)
#define g_nfcOpMessageMs (S->nfcOpMessageMs)
#define g_oledRowCache (S->oledRowCache)
#define g_lcdStatusActive (S->lcdStatusActive)
#define g_lcdStatusStartMs (S->lcdStatusStartMs)
#define g_reinitPostWaitActive (S->reinitPostWaitActive)
#define g_reinitPostWaitStartMs (S->reinitPostWaitStartMs)
#define g_wdProfileCache (S->wdProfileCache)
#define g_wdCacheDirty (S->wdCacheDirty)
#define g_preDispCircTCache (S->preDispCircTCache)
#define g_preDispCircPCache (S->preDispCircPCache)
#define g_preDispPurgTCache (S->preDispPurgTCache)
#define g_preDispPurgPCache (S->preDispPurgPCache)
#define g_preDispDirty (S->preDispDirty)
#define g_solProfileCache (S->solProfileCache)
#define g_solCacheDirty (S->solCacheDirty)
#define g_dispCounterCache (S->dispCounterCache)
#define g_dispCacheDirty (S->dispCacheDirty)
#define g_backwashDurCache (S->backwashDurCache)
#define g_backwashNDispCache (S->backwashNDispCache)
#define g_backwashCounterCache (S->backwashCounterCache)
#define g_backwashManShortCache (S->backwashManShortCache)
#define g_backwashManLongCache (S->backwashManLongCache)
#define g_backwashDailyTimeCache (S->backwashDailyTimeCache)
#define g_backwashDailyDurCache (S->backwashDailyDurCache)
#define g_bwCacheDirty (S->bwCacheDirty)
#define g_uvCacheDirty (S->uvCacheDirty)
#define g_currentUvIndex (S->currentUvIndex)
#define g_sensorBypDurCache (S->sensorBypDurCache)
#define g_sensorBypPeriodCache (S->sensorBypPeriodCache)
#define g_sensorBypManShortCache (S->sensorBypManShortCache)
#define g_sensorBypManLongCache (S->sensorBypManLongCache)
#define g_sensorBypDirty (S->sensorBypDirty)
#define g_autoCircDurCache (S->autoCircDurCache)
#define g_autoCircPeriodCache (S->autoCircPeriodCache)
#define g_manCircShortCache (S->manCircShortCache)
#define g_manCircLongCache (S->manCircLongCache)
#define g_wcircDirty (S->wcircDirty)
#define g_dnfReptPeriodCache (S->dnfReptPeriodCache)
#define g_dnfReptDuratCache (S->dnfReptDuratCache)
#define g_preFlshBeepDelyCache (S->preFlshBeepDelyCache)
#define g_beepOnTimeCache (S->beepOnTimeCache)
#define g_beepOffTimeCache (S->beepOffTimeCache)
#define g_beepCountCache (S->beepCountCache)
#define g_pflushDirty (S->pflushDirty)
#define g_pcbIdDirty (S->pcbIdDirty)
#define g_nfcOpState (S->nfcOpState)
#define g_nfcHash1 (S->nfcHash1)
#define g_nfcHash2 (S->nfcHash2)
#define g_lcdNfcMessageActive (S->lcdNfcMessageActive)
#define g_lcdNfcMessageMs (S->lcdNfcMessageMs)
#define g_reinitHoldActive (S->reinitHoldActive)
#define g_reinitHoldStartMs (S->reinitHoldStartMs)
#define g_btnUp (S->btnUp)
#define g_btnDown (S->btnDown)
#define g_btnSelect (S->btnSelect)
#define g_btnBack (S->btnBack)

// True when the editor is in an active NFC Add/Del session.
static inline bool isNfcAddDelLayer(uint8_t layer) {
  return (layer == LAYER_NFC_ADD || layer == LAYER_NFC_DEL);
}

// Consume the current BACK press so it does not trigger the long-press reset
// (or a second back action on release) after we programmatically exit a mode.
static inline void consumeBackPress(unsigned long now) {
  g_btnBack.lastState = LOW;
  g_btnBack.pressStartMs = now;
  g_btnBack.longHandled = true;
}

// -------------------- Weak override hook for HWID text --------------------
// If the host firmware provides KioskFormatHwIdText(hwid, out, outLen), this module will use it.
// If not provided, it falls back to "PCB%03u".
extern "C" void KioskFormatHwIdText(uint8_t hwid, char* out, size_t outLen) __attribute__((weak));
// Optional host override for SWID formatting.
extern "C" void KioskFormatSwidText(char* out, size_t outLen) __attribute__((weak));

// -------------------- Low-level helpers --------------------
static void copyProgmemString(char* dest, const char* progmemSrc, size_t maxLen) {
  if (!dest || maxLen == 0) return;
  strncpy_P(dest, progmemSrc, maxLen - 1);
  dest[maxLen - 1] = '\0';
}

static void copyProgmemTableEntry(char* dest, const char* const* table, uint8_t index, uint8_t maxIndex, size_t maxLen) {
  if (index >= maxIndex) { if (maxLen) dest[0] = '\0'; return; }
  const char* ptr = (const char*)pgm_read_ptr(&table[index]);
  copyProgmemString(dest, ptr, maxLen);
}

// Write a 16-character row padded with spaces (OLED/LCD friendly).
static void fillDisplayRow(char* row, const char* text) {
  memset(row, ' ', 16);
  row[16] = '\0';
  if (!text) return;
  size_t len = strlen(text);
  if (len > 16) len = 16;
  memcpy(row, text, len);
}

// Format a 32-bit hash as 8 hex characters (uppercase).
static void formatHash32(uint32_t hash, char out[9]) {
  for (int i = 0; i < 8; ++i) {
    uint8_t nibble = (hash >> (28 - 4*i)) & 0x0F;
    out[i] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
  }
  out[8] = '\0';
}

static void updateBackwashStampLinesOnEntry() {
  fillDisplayRow(g_bwDailyAutoLine, "");
  fillDisplayRow(g_bwDailyTrigLine, "");

  KioskIO::RtcTime stamp{};
  if (KioskIO::rtcReadDailyBackwashStamp(stamp)) {
    char buf[32];
    snprintf_P(buf, sizeof(buf), PSTR("Day %02u%02u%02u %02u:%02u"),
               (unsigned)(stamp.year % 100U), (unsigned)stamp.month, (unsigned)stamp.day,
               (unsigned)stamp.hour, (unsigned)stamp.minute);
    fillDisplayRow(g_bwDailyAutoLine, buf);
  }

  KioskIO::RtcTime trig{};
  if (KioskIO::rtcReadTriggeredBackwashStamp(trig)) {
    char buf[32];
    snprintf_P(buf, sizeof(buf), PSTR("Trg %02u%02u%02u-%02u:%02u"),
               (unsigned)(trig.year % 100U), (unsigned)trig.month, (unsigned)trig.day,
               (unsigned)trig.hour, (unsigned)trig.minute);
    fillDisplayRow(g_bwDailyTrigLine, buf);
  }
}

// Watchdog reset: used as the only “exit” from this module.
[[noreturn]] static void resetNow() {
  wdt_enable(WDTO_15MS);
  while (1) { }
}

static bool readPressed(uint8_t pin) {
  return (digitalRead(pin) == LOW);
}

// Generic wrapped cursor movement for menus.
static void moveCursorWrapped(uint8_t& cursor, uint8_t count, int8_t dir) {
  int v = (int)cursor + (int)dir;
  while (v < 0) v += (int)count;
  while (v >= (int)count) v -= (int)count;
  cursor = (uint8_t)v;
}

// HWID text formatting:
// - prefer host’s KioskFormatHwIdText override if provided
// - else fall back to PCB%03u
static void formatHwidText(uint8_t hwid, char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  if (KioskFormatHwIdText) {
    KioskFormatHwIdText(hwid, out, outLen);
    out[outLen - 1] = '\0';
    if (out[0] != '\0') return;
  }
  snprintf_P(out, outLen, PSTR("PCB%03u"), (unsigned)hwid);
}

static void formatSwidText(char* out, size_t outLen) {
  if (!out || outLen == 0) return;
  if (KioskFormatSwidText) {
    KioskFormatSwidText(out, outLen);
    out[outLen - 1] = '\0';
    if (out[0] != '\0') return;
  }
  snprintf_P(out, outLen, PSTR("%s"), KIOSK_SWID);
}

static uint8_t bcdToDec(uint8_t v) {
  return (uint8_t)((v >> 4) * 10 + (v & 0x0F));
}

static bool rtcProbeDs3231() {
  Wire.beginTransmission(0x68);
  return (Wire.endTransmission() == 0);
}

static bool rtcReadDs3231(uint16_t &year, uint8_t &month, uint8_t &day,
                          uint8_t &hour, uint8_t &minute, uint8_t &second) {
  Wire.beginTransmission(0x68);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission() != 0) return false;

  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)7) != 7) return false;
  uint8_t rawSec = Wire.read();
  uint8_t rawMin = Wire.read();
  uint8_t rawHour = Wire.read();
  Wire.read(); // day-of-week
  uint8_t rawDay = Wire.read();
  uint8_t rawMonth = Wire.read();
  uint8_t rawYear = Wire.read();

  second = bcdToDec(rawSec & 0x7F);
  minute = bcdToDec(rawMin & 0x7F);
  if (rawHour & 0x40) {
    uint8_t hr12 = bcdToDec(rawHour & 0x1F);
    bool pm = (rawHour & 0x20) != 0;
    if (hr12 == 12) hour = pm ? 12 : 0;
    else hour = pm ? (uint8_t)(hr12 + 12) : hr12;
  } else {
    hour = bcdToDec(rawHour & 0x3F);
  }
  day = bcdToDec(rawDay & 0x3F);
  month = bcdToDec(rawMonth & 0x1F);
  year = (uint16_t)(2000 + bcdToDec(rawYear));
  return true;
}

static uint8_t decToBcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static bool rtcWriteDs3231(uint16_t year, uint8_t month, uint8_t day,
                           uint8_t hour, uint8_t minute, uint8_t second) {
  Wire.beginTransmission(0x68);
  Wire.write((uint8_t)0x00);
  Wire.write(decToBcd(second));
  Wire.write(decToBcd(minute));
  Wire.write(decToBcd(hour)); // 24-hour mode
  Wire.write((uint8_t)0x01);  // day-of-week (unused)
  Wire.write(decToBcd(day));
  Wire.write(decToBcd(month)); // century bit cleared
  Wire.write(decToBcd((uint8_t)(year % 100)));
  return (Wire.endTransmission() == 0);
}

static uint8_t daysInMonth(uint16_t year, uint8_t month) {
  switch (month) {
    case 1: case 3: case 5: case 7: case 8: case 10: case 12: return 31;
    case 4: case 6: case 9: case 11: return 30;
    case 2: {
      const bool leap = ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0);
      return leap ? 29 : 28;
    }
    default: return 31;
  }
}

static void rtcDigitsFromFields(uint16_t year, uint8_t month, uint8_t day,
                                uint8_t hour, uint8_t minute, uint8_t second,
                                char *digits) {
  if (!digits) return;
  digits[0] = (char)('0' + ((year / 1000) % 10));
  digits[1] = (char)('0' + ((year / 100) % 10));
  digits[2] = (char)('0' + ((year / 10) % 10));
  digits[3] = (char)('0' + (year % 10));
  digits[4] = (char)('0' + (month / 10));
  digits[5] = (char)('0' + (month % 10));
  digits[6] = (char)('0' + (day / 10));
  digits[7] = (char)('0' + (day % 10));
  digits[8] = (char)('0' + (hour / 10));
  digits[9] = (char)('0' + (hour % 10));
  digits[10] = (char)('0' + (minute / 10));
  digits[11] = (char)('0' + (minute % 10));
  digits[12] = (char)('0' + (second / 10));
  digits[13] = (char)('0' + (second % 10));
}

static void rtcFieldsFromDigits(const char *digits,
                                uint16_t &year, uint8_t &month, uint8_t &day,
                                uint8_t &hour, uint8_t &minute, uint8_t &second) {
  year = (uint16_t)((digits[0]-'0')*1000 + (digits[1]-'0')*100 +
                    (digits[2]-'0')*10 + (digits[3]-'0'));
  month = (uint8_t)((digits[4]-'0')*10 + (digits[5]-'0'));
  day = (uint8_t)((digits[6]-'0')*10 + (digits[7]-'0'));
  hour = (uint8_t)((digits[8]-'0')*10 + (digits[9]-'0'));
  minute = (uint8_t)((digits[10]-'0')*10 + (digits[11]-'0'));
  second = (uint8_t)((digits[12]-'0')*10 + (digits[13]-'0'));
}

static void rtcClampFields(uint16_t &year, uint8_t &month, uint8_t &day,
                           uint8_t &hour, uint8_t &minute, uint8_t &second) {
  if (year < 2000) year = 2000;
  if (year > 2099) year = 2099;
  if (month < 1) month = 1;
  if (month > 12) month = 12;
  const uint8_t maxDay = daysInMonth(year, month);
  if (day < 1) day = 1;
  if (day > maxDay) day = maxDay;
  if (hour > 23) hour = 23;
  if (minute > 59) minute = 59;
  if (second > 59) second = 59;
}

static void centerText16(char *out16, const char *text) {
  if (!out16) return;
  if (!text) {
    fillDisplayRow(out16, "");
    return;
  }
  char buf[17];
  fillDisplayRow(buf, "");
  size_t rawLen = strlen(text);
  if (rawLen > 16) rawLen = 16;

  size_t first = 0;
  while (first < rawLen && text[first] == ' ') ++first;

  size_t last = rawLen;
  while (last > first && text[last - 1] == ' ') --last;

  size_t len = last - first;
  size_t start = (16 - len) / 2;
  for (size_t i = 0; i < len && (start + i) < 16; ++i) {
    buf[start + i] = text[first + i];
  }
  memcpy(out16, buf, 17);
}

static void rtcUpdateInfoLinesFromDigits() {
  char dateBuf[17];
  char timeBuf[17];
  snprintf(dateBuf, sizeof(dateBuf), "%c%c%c%c/%c%c/%c%c",
           g_rtcDigits[0], g_rtcDigits[1], g_rtcDigits[2], g_rtcDigits[3],
           g_rtcDigits[4], g_rtcDigits[5], g_rtcDigits[6], g_rtcDigits[7]);
  snprintf(timeBuf, sizeof(timeBuf), "%c%c:%c%c:%c%c",
           g_rtcDigits[8], g_rtcDigits[9], g_rtcDigits[10], g_rtcDigits[11],
           g_rtcDigits[12], g_rtcDigits[13]);
  centerText16(g_rtcLine6, dateBuf);
  centerText16(g_rtcLine7, timeBuf);
}

static void rtcUpdateInfoLinesFromFields(uint16_t year, uint8_t month, uint8_t day,
                                         uint8_t hour, uint8_t minute, uint8_t second) {
  char digits[14];
  rtcDigitsFromFields(year, month, day, hour, minute, second, digits);
  char dateBuf[17];
  char timeBuf[17];
  snprintf(dateBuf, sizeof(dateBuf), "%c%c%c%c/%c%c/%c%c",
           digits[0], digits[1], digits[2], digits[3],
           digits[4], digits[5], digits[6], digits[7]);
  snprintf(timeBuf, sizeof(timeBuf), "%c%c:%c%c:%c%c",
           digits[8], digits[9], digits[10], digits[11],
           digits[12], digits[13]);
  centerText16(g_rtcLine6, dateBuf);
  centerText16(g_rtcLine7, timeBuf);
}

static void rtcUpdateEditLineFromDigits() {
  fillDisplayRow(g_rtcLine13, "");
  for (uint8_t i = 0; i < 8; ++i) {
    g_rtcLine13[i] = g_rtcDigits[i];
  }
  g_rtcLine13[8] = ' ';
  for (uint8_t i = 0; i < 6; ++i) {
    g_rtcLine13[9 + i] = g_rtcDigits[8 + i];
  }
}

static void rtcUpdateTimeDigitsFromFields(uint16_t year, uint8_t month, uint8_t day,
                                          uint8_t hour, uint8_t minute, uint8_t second) {
  char digits[14];
  rtcDigitsFromFields(year, month, day, hour, minute, second, digits);
  memcpy(g_rtcDigits + 8, digits + 8, 6);
  rtcUpdateEditLineFromDigits();
}

static void rtcClusterSpan(uint8_t cluster, uint8_t &start, uint8_t &len) {
  switch (cluster) {
    case 0: start = 0;  len = 4; break;  // YYYY
    case 1: start = 4;  len = 2; break;  // MM
    case 2: start = 6;  len = 2; break;  // DD
    case 3: start = 9;  len = 2; break;  // HH
    case 4: start = 11; len = 2; break;  // MM
    case 5: start = 13; len = 2; break;  // SS
    default: start = 0; len = 0; break;
  }
}

static void rtcSetClusterValue(uint8_t cluster, int8_t dir) {
  uint16_t y = 2000;
  uint8_t mo = 1, d = 1, h = 0, mi = 0, s = 0;
  rtcFieldsFromDigits(g_rtcDigits, y, mo, d, h, mi, s);
  switch (cluster) {
    case 0: // year
      if (dir > 0) y = (y < 2099) ? (uint16_t)(y + 1) : 2000;
      else y = (y > 2000) ? (uint16_t)(y - 1) : 2099;
      break;
    case 1: // month
      if (dir > 0) mo = (mo < 12) ? (uint8_t)(mo + 1) : 1;
      else mo = (mo > 1) ? (uint8_t)(mo - 1) : 12;
      break;
    case 2: // day
      if (dir > 0) {
        uint8_t maxDay = daysInMonth(y, mo);
        d = (d < maxDay) ? (uint8_t)(d + 1) : 1;
      } else {
        uint8_t maxDay = daysInMonth(y, mo);
        d = (d > 1) ? (uint8_t)(d - 1) : maxDay;
      }
      break;
    case 3: // hour
      if (dir > 0) h = (h < 23) ? (uint8_t)(h + 1) : 0;
      else h = (h > 0) ? (uint8_t)(h - 1) : 23;
      break;
    case 4: // minute
      if (dir > 0) mi = (mi < 59) ? (uint8_t)(mi + 1) : 0;
      else mi = (mi > 0) ? (uint8_t)(mi - 1) : 59;
      break;
    case 5: // second
      if (dir > 0) s = (s < 59) ? (uint8_t)(s + 1) : 0;
      else s = (s > 0) ? (uint8_t)(s - 1) : 59;
      break;
    default:
      break;
  }
  rtcClampFields(y, mo, d, h, mi, s);
  rtcDigitsFromFields(y, mo, d, h, mi, s, g_rtcDigits);
  rtcUpdateEditLineFromDigits();
  g_rtcDirty = true;
}

// -------------------- LCD helpers --------------------
// lcdWriteFixed(): write a left-aligned 16-char padded row.
static void lcdWriteFixed(uint8_t row, const char* text) {
  if (!G->lcd) return;
  char line[17];
  fillDisplayRow(line, text);
  G->lcd->setCursor(0, row);
  G->lcd->print(line);
}

static void lcdPrintLine(uint8_t row, const char* text16) {
  if (!G->lcd) return;
  G->lcd->setCursor(0, row);
  G->lcd->print(text16);
}

// Base LCD “frame” for editor mode (leaves line1 free for overlays).
static void renderLcdBase() {
  if (!G->lcd) return;

  char buf[17];

  // Line0 = EEPROM EDIT MODE
  copyProgmemString(buf, STR_EEPROM_EDIT, sizeof(buf));
  lcdWriteFixed(0, buf);

  // Line1 = separator
  copyProgmemString(buf, STR_DASHES, sizeof(buf));
  lcdWriteFixed(1, buf);

  // Line2 = HWID: + text
  char hwidText[16];
  formatHwidText(g_workingPcbId, hwidText, sizeof(hwidText));
  snprintf_P(buf, sizeof(buf), PSTR("HWID:%s"), hwidText);
  lcdWriteFixed(2, buf);

  // Line3 = SWID: + build timestamp + status char (packed into 16 chars)
  char swid[17];
  formatSwidText(swid, sizeof(swid));
  lcdWriteFixed(3, swid);
}

// Update LCD line1 with a prefix + optional hash.
// The “Saving/Saved” overlay takes priority and blocks NFC from overwriting line1.
static void updateLcdLine1Progmem(const char* progmemPrefix, uint32_t hash) {
  if (!G->lcd) return;

  if (g_lcdStatusActive) return;

  char line[17];
  if (progmemPrefix == nullptr) {
    copyProgmemString(line, STR_LCD_CLEAR, sizeof(line));
  } else if (hash != 0) {
    char prefix[9]; copyProgmemString(prefix, progmemPrefix, sizeof(prefix));
    char hashStr[9]; formatHash32(hash, hashStr);
    snprintf_P(line, sizeof(line), PSTR("%-8s%s"), prefix, hashStr); // 8 + 8 = 16
  } else {
    copyProgmemString(line, progmemPrefix, sizeof(line));
    size_t len = strlen(line);
    while (len < 16) line[len++] = ' ';
    line[16] = '\0';
  }

  lcdPrintLine(1, line);
}

static void showLcdWaiting() {
  updateLcdLine1Progmem(STR_LCD_WAITING, 0);
  g_lcdNfcMessageActive = false;
}

static void showLcdNfcResult(const char* progmemPrefix, uint32_t hash, unsigned long now) {
  updateLcdLine1Progmem(progmemPrefix, hash);
  g_lcdNfcMessageActive = true;
  g_lcdNfcMessageMs = now;
}

static void clearLcdLine1() {
  char buf[17];
  copyProgmemString(buf, STR_DASHES, sizeof(buf));
  lcdWriteFixed(1, buf);
  g_lcdNfcMessageActive = false;
}

// -------------------- NFC scan: keep PN532 in reset except while scanning --------------------
// This function explicitly brings PN532 out of reset, performs a short read loop, and then
// forces PN532 back into reset. This is the safety mechanism to avoid I2C bus lockups.
static uint32_t readNfcHash(unsigned long timeoutSeconds) {
  if (!G->nfc || G->pinPn532Reset == 255) return 0;

  // Allow operator to abort immediately (e.g. exit NFC add/del mode).
  if (digitalRead(g_btnBack.pin) == LOW) return NFC_HASH_ABORT;

  digitalWrite(G->pinPn532Reset, HIGH);
  delay(100);

  G->nfc->begin();
  G->nfc->SAMConfig();

  uint8_t uid[7] = {0};
  uint8_t uidLen = 0;

  uint32_t result = 0;
  const unsigned long startTime = millis();
  const unsigned long timeoutMillis = timeoutSeconds * 1000UL;

  while ((millis() - startTime) < timeoutMillis) {
    // Abort scan if BACK is pressed.
    if (digitalRead(g_btnBack.pin) == LOW) { result = NFC_HASH_ABORT; break; }

    if (G->nfc->readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 50)) {
      // FNV-1a hash of UID
      uint32_t hash = 2166136261UL;
      for (uint8_t i = 0; i < uidLen; i++) {
        hash ^= uid[i];
        hash *= 16777619UL;
      }
      result = hash;
      break;
    }
  }

  // Always return PN532 to reset state.
  digitalWrite(G->pinPn532Reset, LOW);
  return result;
}

// -------------------- Layer transitions --------------------
// onEnterLayer(): load caches / initialise per-layer state.
static void onEnterLayer(uint8_t layer) {
  switch (layer) {
    case LAYER_WD_EDIT:
      g_wdProfileCache = G->ee->dispMeasuredProfile();
      g_wdCacheDirty = false;
      g_preDispCircTCache = G->ee->preDispCircTSeconds();
      g_preDispCircPCache = G->ee->preDispCircP100Pulses();
      g_preDispPurgTCache = G->ee->preDispPurgTSeconds();
      g_preDispPurgPCache = G->ee->preDispPurgP100Pulses();
      g_preDispDirty = false;
      break;

    case LAYER_WD_MENU:
      g_wdMenuTop = 0;
      break;

    case LAYER_SOL_PARAM_EDIT:
      g_solParamCursor = SOL_PARAM_STARTPWM;
      g_solProfileCache = G->ee->solenoidProfile((KioskEeprom::Solenoid)g_currentSolenoidIndex);
      g_solCacheDirty = false;
      break;

    case LAYER_BACKWASH_EDIT:
      if (g_currentBwIndex == BW_MENU_DUR)     g_backwashDurCache  = G->ee->backwashDuration();
      if (g_currentBwIndex == BW_MENU_NDISP)   g_backwashNDispCache= G->ee->backwashAfterNDispenses();
      if (g_currentBwIndex == BW_MENU_COUNTER) g_backwashCounterCache = G->ee->backwashDispenseCounter();
      if (g_currentBwIndex == BW_MENU_MAN_SHORT) g_backwashManShortCache = G->ee->backwashManualShortSeconds();
      if (g_currentBwIndex == BW_MENU_MAN_LONG)  g_backwashManLongCache = G->ee->backwashManualLong2Minutes();
      if (g_currentBwIndex == BW_MENU_DAILY_TIME) g_backwashDailyTimeCache = G->ee->dailyBackwashTimeMinutes();
      if (g_currentBwIndex == BW_MENU_DAILY_DUR)  g_backwashDailyDurCache = G->ee->dailyBackwashDuration10s();
      if (g_currentBwIndex == BW_MENU_DAILY_TIME) updateBackwashStampLinesOnEntry();
      g_bwCacheDirty = false;
      break;

    case LAYER_KLARAN_EDIT:
      // Load KlaranUV values for editing.
      g_uvOkDelayCache   = G->ee->klaranUvOkDelay10ms();
      g_uvMaxOntimeCache = G->ee->klaranUvMaxOntimeMinutes();
      g_uvCacheDirty = false;
      break;

    case LAYER_HYDRPARM_MENU:
      // Preserve cursor to keep operator context when returning from submenus.
      break;

    case LAYER_SENSORBYP_MENU:
      // Preserve cursor to keep operator context when returning from edit.
      break;

    case LAYER_SENSORBYP_EDIT:
      g_sensorBypDurCache = G->ee->sensorBypassDuration100ms();
      g_sensorBypPeriodCache = G->ee->sensorBypassPeriodMinutes();
      g_sensorBypManShortCache = G->ee->sensorBypassManualShort100ms();
      g_sensorBypManLongCache = G->ee->sensorBypassManualLong10s();
      g_sensorBypDirty = false;
      break;

    case LAYER_WCIRC_MENU:
      // Preserve cursor to keep operator context when returning from edit.
      break;

    case LAYER_WCIRC_EDIT:
      g_autoCircDurCache = G->ee->autoCircDurationMinutes();
      g_autoCircPeriodCache = G->ee->autoCircPeriod10Minutes();
      g_manCircShortCache = G->ee->manCircDurationShortMinutes();
      g_manCircLongCache = G->ee->manCircDurationLong10Minutes();
      g_wcircDirty = false;
      break;

    case LAYER_PFLUSH_MENU:
      // Preserve cursor to keep operator context when returning from edit.
      break;

    case LAYER_PFLUSH_EDIT:
      g_dnfReptPeriodCache = G->ee->dnfReptPeriodMinutes();
      g_dnfReptDuratCache = G->ee->dnfReptDurat100ms();
      g_preFlshBeepDelyCache = G->ee->preFlshBeepDelySeconds();
      g_beepOnTimeCache = G->ee->beepOnTime10ms();
      g_beepOffTimeCache = G->ee->beepOffTime100ms();
      g_beepCountCache = G->ee->beepCount();
      g_pflushDirty = false;
      break;

    case LAYER_WATERTEMP_EDIT:
      // Load WaterTempSense value for editing.
      g_waterTempSenseCache = G->ee->waterTempSense(); // returns 1..2 (clamped)
      g_waterTempSenseDirty = false;
      break;

    case LAYER_COINACC_EDIT:
      g_coinAccFittedCache = G->ee->coinAcceptorFitted();
      g_coinAccFittedDirty = false;
      break;

    case LAYER_RTC_CONFIG: {
      g_rtcPresent = rtcProbeDs3231();
      g_rtcDirty = false;
      g_rtcClusterIndex = 0;
      g_rtcLastPollMs = 0;
      g_rtcLastSecond = 0xFF;
      uint16_t y = 2000;
      uint8_t mo = 1, d = 1, h = 0, mi = 0, s = 0;
      if (g_rtcPresent && rtcReadDs3231(y, mo, d, h, mi, s)) {
        rtcClampFields(y, mo, d, h, mi, s);
      } else {
        g_rtcPresent = false;
      }
      rtcDigitsFromFields(y, mo, d, h, mi, s, g_rtcDigits);
      centerText16(g_rtcLine5, "RTC Data & Time");
      if (!g_rtcPresent) {
        centerText16(g_rtcLine6, "No DS3231 RTC !");
        fillDisplayRow(g_rtcLine7, "");
        fillDisplayRow(g_rtcLine13, "");
      } else {
        rtcUpdateInfoLinesFromDigits();
        rtcUpdateEditLineFromDigits();
      }
      break;
    }

    case LAYER_NFC_PARMS_MENU:
      g_nfcParmsMenuCursor = NFC_PARMS_INIT_DELAY;
      break;

    case LAYER_NFC_PARMS_EDIT:
      // Load all NFC timing values for editing (ms).
      g_nfcInitDelayMsCache = (uint16_t)G->ee->nfcInitDelayMs();
      g_nfcReadDurationMsCache = (uint16_t)G->ee->nfcReadDurationMs();
      g_nfcInterDelayMsCache = (uint16_t)G->ee->nfcInterNfcDelayMs();
      g_nfcParmsDirty = false;
      break;

    case LAYER_DISPENSED_EDIT:
      if (g_currentDispenseIndex < KioskEeprom::DISPENSE_COUNTER_COUNT) {
        g_dispCounterCache = G->ee->dispenseCounter((KioskEeprom::DispenseCounter)g_currentDispenseIndex);
        g_dispCacheDirty = false;
      }
      break;

    case LAYER_NFC_MENU:
      g_removeAllPromptActive = false;
      g_removeAllConfirm = false;
      break;

    case LAYER_NFC_BROWSE:
      g_tokenBrowseIndex = 0;
      g_tokenState = TOKEN_STATE_BROWSE;
      g_tokenDeleteConfirm = false;
      break;

    case LAYER_NFC_ADD:
    case LAYER_NFC_DEL:
      g_nfcOpState = NFC_OP_STATE_IDLE;
      g_nfcHash1 = g_nfcHash2 = 0;
      showLcdWaiting(); // Only show “Waiting” while in NFC add/del.
      break;

    case LAYER_REINIT:
      g_reinitPromptActive = false;
      g_reinitConfirmYes = false;
      g_reinitHoldActive = false;
      break;

    default:
      break;
  }
}

// onExitLayer(): clear transient per-layer state.
static void onExitLayer(uint8_t layer) {
  switch (layer) {
    case LAYER_NFC_BROWSE:
      g_tokenState = TOKEN_STATE_BROWSE;
      g_tokenDeleteConfirm = false;
      break;

    case LAYER_NFC_ADD:
    case LAYER_NFC_DEL:
      g_nfcOpState = NFC_OP_STATE_IDLE;
      g_nfcHash1 = g_nfcHash2 = 0;
      clearLcdLine1();
      break;

    case LAYER_NFC_MENU:
      g_removeAllPromptActive = false;
      g_removeAllConfirm = false;
      break;

    case LAYER_REINIT:
      g_reinitPromptActive = false;
      g_reinitConfirmYes = false;
      g_reinitHoldActive = false;
      break;

    default:
      break;
  }
}

static void setLayer(uint8_t newLayer) {
  if (newLayer == g_currentLayer) return;
  onExitLayer(g_currentLayer);
  g_currentLayer = newLayer;
  onEnterLayer(g_currentLayer);
}

static void clearRtcCaretRow();
static bool saveIfDirtyForLayer();

// Back navigation: handles special modal prompts first (remove-all, delete prompt, reinit prompt).
static void goBackOneLayer() {
  if (g_removeAllPromptActive) { g_removeAllPromptActive = false; g_removeAllConfirm = false; return; }
  if (g_currentLayer == LAYER_NFC_BROWSE && g_tokenState == TOKEN_STATE_DELETE_PROMPT) { g_tokenState = TOKEN_STATE_BROWSE; g_tokenDeleteConfirm = false; return; }
  if (g_currentLayer == LAYER_REINIT && g_reinitPromptActive) { g_reinitPromptActive = false; g_reinitConfirmYes = false; g_reinitHoldActive = false; return; }

  switch (g_currentLayer) {
    case LAYER_PCB:
    case LAYER_DISPENSED_MENU:    case LAYER_NFC_MENU:
    case LAYER_KIOSKPARM_MENU:
    case LAYER_HYDRPARM_MENU:
    case LAYER_REINIT:
      if (g_currentLayer == LAYER_PCB) (void)saveIfDirtyForLayer();
      setLayer(LAYER_TOP);
      break;


    case LAYER_SOL_MENU:
    case LAYER_KLARAN_MENU:
    case LAYER_NFC_PARMS_MENU:
      // These parameter sections are under the \"KioskParm Edit\" category.
      setLayer(LAYER_KIOSKPARM_MENU);
      break;

    case LAYER_BACKWASH_MENU:
    case LAYER_SENSORBYP_MENU:
    case LAYER_WCIRC_MENU:
    case LAYER_PFLUSH_MENU:
      setLayer(LAYER_HYDRPARM_MENU);
      break;

    case LAYER_WD_MENU:
      setLayer(LAYER_HYDRPARM_MENU);
      break;

    case LAYER_WD_EDIT:        (void)saveIfDirtyForLayer(); setLayer(LAYER_WD_MENU); break;
    case LAYER_SOL_PARAM_EDIT: (void)saveIfDirtyForLayer(); setLayer(LAYER_SOL_MENU); break;
    case LAYER_DISPENSED_EDIT: (void)saveIfDirtyForLayer(); setLayer(LAYER_DISPENSED_MENU); break;
    case LAYER_BACKWASH_EDIT:  (void)saveIfDirtyForLayer(); setLayer(LAYER_BACKWASH_MENU); break;
    case LAYER_KLARAN_EDIT:    (void)saveIfDirtyForLayer(); setLayer(LAYER_KLARAN_MENU); break;
    case LAYER_SENSORBYP_EDIT: (void)saveIfDirtyForLayer(); setLayer(LAYER_SENSORBYP_MENU); break;
    case LAYER_WCIRC_EDIT:     (void)saveIfDirtyForLayer(); setLayer(LAYER_WCIRC_MENU); break;
    case LAYER_PFLUSH_EDIT:    (void)saveIfDirtyForLayer(); setLayer(LAYER_PFLUSH_MENU); break;
    case LAYER_WATERTEMP_EDIT: (void)saveIfDirtyForLayer(); setLayer(LAYER_KIOSKPARM_MENU); break;
    case LAYER_COINACC_EDIT:   (void)saveIfDirtyForLayer(); setLayer(LAYER_KIOSKPARM_MENU); break;
    case LAYER_RTC_CONFIG:
      (void)saveIfDirtyForLayer();
      clearRtcCaretRow();
      setLayer(LAYER_KIOSKPARM_MENU);
      break;
    case LAYER_NFC_PARMS_EDIT:  (void)saveIfDirtyForLayer(); setLayer(LAYER_NFC_PARMS_MENU); break;

    case LAYER_NFC_BROWSE:
    case LAYER_NFC_ADD:
    case LAYER_NFC_DEL:
      setLayer(LAYER_NFC_MENU);
      break;

    default:
      setLayer(LAYER_TOP);
      break;
  }
}

// -------------------- Rendering (OLED) --------------------
// oledWriteRow(): redraw a row only if content changed (reduces flicker).
static void oledWriteRow(uint8_t row, const char* text16) {
  if (!G->oled) return;
  if (row >= 16) return;
  if (memcmp(g_oledRowCache[row], text16, 16) != 0) {
    G->oled->drawString(0, row, text16);
    memcpy(g_oledRowCache[row], text16, 16);
    g_oledRowCache[row][16] = '\0';
  }
}

static void clearRtcCaretRow() {
  if (!G->oled) return;
  char row[17];
  fillDisplayRow(row, "");
  oledWriteRow(13, row);
}

// forceOledStatusLine(): write row 15 immediately regardless of cache logic.
// Used for fast “Saving/Saved/Resetting/Completed” updates.
static void forceOledStatusLine(const char* msgProgmem) {
  if (!G->oled) return;

  char row[17], msg[17];
  copyProgmemString(msg, msgProgmem, sizeof(msg));
  fillDisplayRow(row, msg);

  G->oled->drawString(0, 15, row);

  memcpy(g_oledRowCache[15], row, 16);
  g_oledRowCache[15][16] = '\0';
}

// Some edit layers use row 11 as a stable header line.
static inline bool layerUsesRow11Header(uint8_t layer) {
  return (layer == LAYER_SOL_PARAM_EDIT ||
          layer == LAYER_BACKWASH_EDIT  ||
          layer == LAYER_DISPENSED_EDIT ||
          layer == LAYER_WD_EDIT        ||
          layer == LAYER_KLARAN_EDIT    ||
          layer == LAYER_SENSORBYP_EDIT ||
          layer == LAYER_WCIRC_EDIT ||
          layer == LAYER_PFLUSH_EDIT ||
          layer == LAYER_WATERTEMP_EDIT ||
          layer == LAYER_COINACC_EDIT ||
          layer == LAYER_RTC_CONFIG ||
          layer == LAYER_NFC_PARMS_EDIT);
}

// Render one menu list row using a PROGMEM label table.
static inline void renderMenuRowFromTable(char* rowBuf,
                                          char* labelBuf,
                                          size_t labelBufSz,
                                          uint8_t idx,
                                          uint8_t cursor,
                                          const char* const* labels,
                                          uint8_t count) {
  if (idx >= count) return;
  rowBuf[0] = (idx == cursor) ? '>' : ' ';

  copyProgmemTableEntry(labelBuf, labels, idx, count, labelBufSz);
  size_t len = strlen(labelBuf);
  if (len > 15) len = 15;
  memcpy(rowBuf + 1, labelBuf, len);
}

// Render the row 11 header for edit layers that use it.
static void renderRow11Header(char* rowBuf, char* labelBuf, size_t labelBufSz) {
  switch (g_currentLayer) {
    case LAYER_SOL_PARAM_EDIT: {
      copyProgmemTableEntry(labelBuf, SOLENOID_NAMES, g_currentSolenoidIndex,
                            KioskEeprom::PWM_SOLENOID_COUNT, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;
    }

    case LAYER_WD_EDIT: {
      const char* p = STR_WD_HDR_MODE;
      if (g_wdEditField == WD_FIELD_TIME) p = STR_WD_HDR_TIME;
      else if (g_wdEditField == WD_FIELD_PULSES) p = STR_WD_HDR_PULSES;
      else if (g_wdEditField == WD_FIELD_PREDISP_CIRCT) p = STR_WD_HDR_PREDISP_CIRCT;
      else if (g_wdEditField == WD_FIELD_PREDISP_CIRCP) p = STR_WD_HDR_PREDISP_CIRCP;
      else if (g_wdEditField == WD_FIELD_PREDISP_PURGT) p = STR_WD_HDR_PREDISP_PURGT;
      else if (g_wdEditField == WD_FIELD_PREDISP_PURGP) p = STR_WD_HDR_PREDISP_PURGP;
      copyProgmemString(labelBuf, p, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;
    }

    case LAYER_BACKWASH_EDIT: {
      const char* p = nullptr;
      switch (g_currentBwIndex) {
        case BW_MENU_DUR:     p = STR_BW_HDR_DUR; break;
        case BW_MENU_NDISP:   p = STR_BW_HDR_NDISP; break;
        case BW_MENU_COUNTER: p = STR_BW_HDR_COUNTER; break;
        case BW_MENU_MAN_SHORT: p = STR_BW_HDR_MAN_SHORT; break;
        case BW_MENU_MAN_LONG:  p = STR_BW_HDR_MAN_LONG; break;
        case BW_MENU_DAILY_TIME: p = STR_BW_HDR_DAILY_TIME; break;
        case BW_MENU_DAILY_DUR:  p = STR_BW_HDR_DAILY_DUR; break;
        default: break;
      }
      if (p) {
        copyProgmemString(labelBuf, p, labelBufSz);
        fillDisplayRow(rowBuf, labelBuf);
        oledWriteRow(11, rowBuf);
      }
      break;
    }

    case LAYER_KLARAN_EDIT: {
      const char* p = (g_currentUvIndex == UV_MENU_OK_DELAY) ? STR_UV_HDR_OK_DELAY : STR_UV_HDR_MAX_ONTIME;
      copyProgmemString(labelBuf, p, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;
    }

    case LAYER_SENSORBYP_EDIT: {
      const char* p = STR_SBYP_HDR_DUR;
      if (g_currentSensorBypIndex == SBYP_MENU_PERIOD) p = STR_SBYP_HDR_PERIOD;
      else if (g_currentSensorBypIndex == SBYP_MENU_MAN_SHORT) p = STR_SBYP_HDR_MAN_SHORT;
      else if (g_currentSensorBypIndex == SBYP_MENU_MAN_LONG) p = STR_SBYP_HDR_MAN_LONG;
      copyProgmemString(labelBuf, p, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;
    }

    case LAYER_WCIRC_EDIT: {
      const char* p = STR_WCIRC_HDR_AUTODUR;
      if (g_currentWcircIndex == WCIRC_MENU_AUTOPERIOD) p = STR_WCIRC_HDR_AUTOPERIOD;
      else if (g_currentWcircIndex == WCIRC_MENU_MANSHORT) p = STR_WCIRC_HDR_MANSHORT;
      else if (g_currentWcircIndex == WCIRC_MENU_MANLONG) p = STR_WCIRC_HDR_MANLONG;
      copyProgmemString(labelBuf, p, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;
    }

    case LAYER_PFLUSH_EDIT: {
      const char* p = STR_PFLUSH_HDR_DNF_PERIOD;
      if (g_currentPflushIndex == PFLUSH_MENU_DNF_DURAT) p = STR_PFLUSH_HDR_DNF_DURAT;
      else if (g_currentPflushIndex == PFLUSH_MENU_PREFLSH_BEEP_DELY) p = STR_PFLUSH_HDR_PREFLSH_BEEP_DELY;
      else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_PERIOD) p = STR_PFLUSH_HDR_BEEP_PERIOD;
      else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_ON) p = STR_PFLUSH_HDR_BEEP_ON;
      else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_COUNT) p = STR_PFLUSH_HDR_BEEP_COUNT;
      copyProgmemString(labelBuf, p, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;
    }

    case LAYER_WATERTEMP_EDIT:
      copyProgmemString(labelBuf, STR_WTEMP_HDR, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;

    case LAYER_COINACC_EDIT:
      copyProgmemString(labelBuf, STR_COINACC_HDR, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;

    case LAYER_RTC_CONFIG:
      copyProgmemString(labelBuf, STR_KPARM_RTC, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;

    case LAYER_NFC_PARMS_EDIT: {
      const char* p = nullptr;
      switch (g_currentNfcParmIndex) {
        case NFC_PARMS_INIT_DELAY:    p = STR_NFCP_INIT_DELAY; break;
        case NFC_PARMS_READ_DURATION: p = STR_NFCP_READ_DURATION; break;
        case NFC_PARMS_INTER_DELAY:   p = STR_NFCP_INTER_DELAY; break;
        default: break;
      }
      if (p) {
        copyProgmemString(labelBuf, p, labelBufSz);
        fillDisplayRow(rowBuf, labelBuf);
        oledWriteRow(11, rowBuf);
      }
      break;
    }

    case LAYER_DISPENSED_EDIT:
      copyProgmemTableEntry(labelBuf, DISPENSE_SHORT_NAMES, g_currentDispenseIndex, 4, labelBufSz);
      fillDisplayRow(rowBuf, labelBuf);
      oledWriteRow(11, rowBuf);
      break;

    default:
      break;
  }
}

static const char* layerTitleForRow2(uint8_t layer) {
  switch (layer) {
    case LAYER_TOP: return nullptr;
    case LAYER_PCB: return STR_TOP_PCBHWID;
    case LAYER_WD_MENU:
    case LAYER_WD_EDIT: return STR_TOP_WATERDISP;
    case LAYER_SOL_MENU:
    case LAYER_SOL_PARAM_EDIT: return STR_TOP_SOLENOID;
    case LAYER_DISPENSED_MENU:
    case LAYER_DISPENSED_EDIT: return STR_TOP_DISPENSED;
    case LAYER_BACKWASH_MENU:
    case LAYER_BACKWASH_EDIT: return STR_TOP_BACKWASH;
    case LAYER_KIOSKPARM_MENU: return STR_TOP_KIOSKPARM;
    case LAYER_HYDRPARM_MENU: return STR_TOP_HYDRPARM;
    case LAYER_KLARAN_MENU:
    case LAYER_KLARAN_EDIT: return STR_TOP_KLARANUV;
    case LAYER_SENSORBYP_MENU:
    case LAYER_SENSORBYP_EDIT: return STR_HPARM_SENSORBYP;
    case LAYER_WCIRC_MENU:
    case LAYER_WCIRC_EDIT: return STR_TOP_WATERCIRC;
    case LAYER_PFLUSH_MENU:
    case LAYER_PFLUSH_EDIT: return STR_TOP_PFLUSH;
    case LAYER_WATERTEMP_EDIT: return STR_TOP_WATERTEMPSENSE;
    case LAYER_COINACC_EDIT: return STR_KPARM_COINACC;
    case LAYER_RTC_CONFIG: return STR_KPARM_RTC;
    case LAYER_NFC_PARMS_MENU:
    case LAYER_NFC_PARMS_EDIT: return STR_KPARM_NFC_PARMS;
    case LAYER_NFC_MENU:
    case LAYER_NFC_BROWSE:
    case LAYER_NFC_ADD:
    case LAYER_NFC_DEL: return STR_TOP_NFC;
    case LAYER_REINIT: return STR_TOP_REINIT;
    default: return nullptr;
  }
}

// renderOled(): redraw all dynamic OLED regions.
// Note: Row 11 is special—some edit layers use it as a stable header to avoid flicker.
static void renderOled() {
  if (!G->oled) return;

  char rowBuf[17];
  char tempBuf[48];
  char labelBuf[17];

  // Row 0: title
  copyProgmemString(labelBuf, STR_EEPROM_EDIT, sizeof(labelBuf));
  fillDisplayRow(rowBuf, labelBuf);
  oledWriteRow(0, rowBuf);

  // Row 1: separator
  copyProgmemString(labelBuf, STR_DASHES, sizeof(labelBuf));
  fillDisplayRow(rowBuf, labelBuf);
  oledWriteRow(1, rowBuf);

  if (g_currentLayer == LAYER_TOP) {
    // Top-level menu: keep HWID/SWID on rows 2 and 3.
    char hwidText[16];
    formatHwidText(g_workingPcbId, hwidText, sizeof(hwidText));
    snprintf_P(tempBuf, sizeof(tempBuf), PSTR("HWID:%s"), hwidText);
    fillDisplayRow(rowBuf, tempBuf);
    oledWriteRow(2, rowBuf);

    formatSwidText(tempBuf, sizeof(tempBuf));
    fillDisplayRow(rowBuf, tempBuf);
    oledWriteRow(3, rowBuf);
  } else {
    // Lower-level screens: row 2 shows current menu name; row 3 stays blank.
    const char* title = layerTitleForRow2(g_currentLayer);
    if (title) copyProgmemString(labelBuf, title, sizeof(labelBuf));
    else labelBuf[0] = '\0';
    fillDisplayRow(rowBuf, labelBuf);
    oledWriteRow(2, rowBuf);
    fillDisplayRow(rowBuf, "");
    oledWriteRow(3, rowBuf);
  }

  if (g_currentLayer == LAYER_RTC_CONFIG) {
    fillDisplayRow(rowBuf, "");
    oledWriteRow(5, rowBuf);
    oledWriteRow(6, g_rtcLine5);
    oledWriteRow(7, g_rtcLine6);
    oledWriteRow(8, g_rtcLine7);
    fillDisplayRow(rowBuf, "");
    oledWriteRow(9, rowBuf);
    oledWriteRow(10, rowBuf);
    copyProgmemString(labelBuf, STR_KPARM_RTC, sizeof(labelBuf));
    fillDisplayRow(rowBuf, labelBuf);
    oledWriteRow(11, rowBuf);
    oledWriteRow(12, g_rtcLine13);
    fillDisplayRow(rowBuf, "");
    if (g_rtcPresent) {
      uint8_t start = 0;
      uint8_t len = 0;
      rtcClusterSpan(g_rtcClusterIndex, start, len);
      for (uint8_t i = 0; i < len && (start + i) < 16; ++i) {
        rowBuf[start + i] = '^';
      }
    }
    oledWriteRow(13, rowBuf);
  } else {
    // Menu list region:
    // - Top menu starts at row 5 (rows 2/3 reserved for HWID/SWID)
    // - Lower-level screens start at row 4
    const uint8_t listStartRow = (g_currentLayer == LAYER_TOP) ? 5 : 4;
    // When top-level is active, keep row 4 clear.
    if (listStartRow == 5) {
      fillDisplayRow(rowBuf, "");
      oledWriteRow(4, rowBuf);
    }
    for (uint8_t r = listStartRow; r <= 11; ++r) {

      // Prevent row 11 from being overwritten in edit modes that use it as a stable header line.
      if (layerUsesRow11Header(g_currentLayer) && r == 11) continue;
      // For BW Daily Time edit, rows 5/6 are dedicated to cached stamp lines.
      if (g_currentLayer == LAYER_BACKWASH_EDIT &&
          g_currentBwIndex == BW_MENU_DAILY_TIME &&
          (r == 5 || r == 6)) continue;

      fillDisplayRow(rowBuf, "");
      uint8_t idx = (uint8_t)(r - listStartRow);

      switch (g_currentLayer) {
        case LAYER_TOP:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_topCursor, TOP_LABELS, TOP_COUNT);
          break;

        case LAYER_KIOSKPARM_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_kioskParmCursor, KPARM_LABELS, KPARM_MENU_COUNT);
          break;

        case LAYER_HYDRPARM_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_hydrParmCursor, HPARM_LABELS, HPARM_MENU_COUNT);
          break;

        case LAYER_WD_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf),
                                 (uint8_t)(g_wdMenuTop + idx),
                                 g_wdMenuCursor,
                                 WD_MENU_LABELS,
                                 WD_MENU_COUNT);
          break;

        case LAYER_SOL_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_solMenuCursor, SOLENOID_NAMES, KioskEeprom::PWM_SOLENOID_COUNT);
          break;

        case LAYER_DISPENSED_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_dispMenuCursor, DISPENSE_MENU_NAMES, DISP_MENU_COUNT);
          break;

        case LAYER_BACKWASH_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_bwMenuCursor, BW_MENU_LABELS, BW_MENU_COUNT);
          break;

        case LAYER_KLARAN_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_uvMenuCursor, UV_MENU_LABELS, UV_MENU_COUNT);
          break;

        case LAYER_SENSORBYP_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_sensorBypMenuCursor, SBYP_MENU_LABELS, SBYP_MENU_COUNT);
          break;

        case LAYER_WCIRC_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_wcircMenuCursor, WCIRC_MENU_LABELS, WCIRC_MENU_COUNT);
          break;
        case LAYER_PFLUSH_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_pflushMenuCursor, PFLUSH_MENU_LABELS, PFLUSH_MENU_COUNT);
          break;

        case LAYER_NFC_PARMS_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_nfcParmsMenuCursor, NFC_PARMS_LABELS, NFC_PARMS_COUNT);
          break;

        case LAYER_NFC_MENU:
          renderMenuRowFromTable(rowBuf, labelBuf, sizeof(labelBuf), idx, g_nfcMenuCursor, NFC_MENU_LABELS, NFC_MENU_COUNT);
          break;

        default:
          break;
      }

      oledWriteRow(r, rowBuf);
    }
  }

  // Row 11: stable header for edit layers (solenoid name / field name, etc.).
  if (g_currentLayer == LAYER_BACKWASH_EDIT && g_currentBwIndex == BW_MENU_DAILY_TIME) {
    oledWriteRow(5, g_bwDailyAutoLine);
    oledWriteRow(6, g_bwDailyTrigLine);
  }

  renderRow11Header(rowBuf, labelBuf, sizeof(labelBuf));

  // Row 12: value/details line for current layer.
  fillDisplayRow(rowBuf, "");
  char row13Value[17];
  fillDisplayRow(row13Value, "");

  switch (g_currentLayer) {

    case LAYER_PCB: {
      char hwidText[16];
      formatHwidText(g_workingPcbId, hwidText, sizeof(hwidText));
      snprintf_P(tempBuf, sizeof(tempBuf), PSTR("HWID:%s"), hwidText);
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_WD_MENU:
    case LAYER_WD_EDIT: {
      const auto p = (g_currentLayer == LAYER_WD_EDIT) ? g_wdProfileCache : G->ee->dispMeasuredProfile();
      uint8_t field = (g_currentLayer == LAYER_WD_EDIT) ? g_wdEditField : g_wdMenuCursor;

      if (field == WD_FIELD_MODESEL) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%s"), (p.modeSel == KioskEeprom::WD_MODE_TIME) ? "TIMED" : "PULSE");
      } else if (field == WD_FIELD_TIME) {
        // Stored in 100ms units, displayed as seconds with one decimal.
        const uint16_t u = p.duration100ms;
        const uint16_t whole = (uint16_t)(u / 10);
        const uint16_t frac  = (uint16_t)(u % 10);
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"), (unsigned)whole, (unsigned)frac);
      } else if (field == WD_FIELD_PULSES) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u pulse"), (unsigned)p.pulses);
      } else if (field == WD_FIELD_PREDISP_CIRCT) {
        const uint8_t v = (g_currentLayer == LAYER_WD_EDIT) ? g_preDispCircTCache : G->ee->preDispCircTSeconds();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"), (unsigned)v);
      } else if (field == WD_FIELD_PREDISP_CIRCP) {
        const uint8_t v = (g_currentLayer == LAYER_WD_EDIT) ? g_preDispCircPCache : G->ee->preDispCircP100Pulses();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u00 pulse"), (unsigned)v);
      } else if (field == WD_FIELD_PREDISP_PURGT) {
        const uint8_t v = (g_currentLayer == LAYER_WD_EDIT) ? g_preDispPurgTCache : G->ee->preDispPurgTSeconds();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"), (unsigned)v);
      } else {
        const uint8_t v = (g_currentLayer == LAYER_WD_EDIT) ? g_preDispPurgPCache : G->ee->preDispPurgP100Pulses();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u00 pulse"), (unsigned)v);
      }
  fillDisplayRow(rowBuf, tempBuf);
  break;
    }

    case LAYER_SOL_MENU:
    case LAYER_SOL_PARAM_EDIT: {
      // In edit mode, show cached values; in menu mode, show current EEPROM values for highlighted solenoid.
      uint8_t solIdx = (g_currentLayer == LAYER_SOL_PARAM_EDIT) ? g_currentSolenoidIndex : g_solMenuCursor;
      auto s = (g_currentLayer == LAYER_SOL_PARAM_EDIT)
        ? g_solProfileCache
        : G->ee->solenoidProfile((KioskEeprom::Solenoid)solIdx);

      char pname[17];
      copyProgmemTableEntry(pname, SOL_PARAM_NAMES, g_solParamCursor, SOL_PARAM_COUNT, sizeof(pname));
      int32_t val = 0;
      if (g_solParamCursor == SOL_PARAM_STARTPWM)           val = s.startPwm;
      else if (g_solParamCursor == SOL_PARAM_HOLDPWM)       val = s.holdPwm;
      else                                                 val = s.swDelaySec;

      const char* units = "";
      if (g_solParamCursor == SOL_PARAM_SWDELAY) {
        units = "secs";
      }

      snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%s=%ld%s"), pname, (long)val, units);
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_DISPENSED_MENU:
    case LAYER_DISPENSED_EDIT: {
      // Editing: show “<value> dispensed” (label goes on row 11).
      // Menu: show short label + current value (TOTAL handled separately).
      if (g_currentLayer == LAYER_DISPENSED_EDIT || g_dispMenuCursor < 4) {
        uint8_t idx = (g_currentLayer == LAYER_DISPENSED_EDIT) ? g_currentDispenseIndex : g_dispMenuCursor;

        uint32_t v = (g_currentLayer == LAYER_DISPENSED_EDIT)
          ? g_dispCounterCache
          : G->ee->dispenseCounter((KioskEeprom::DispenseCounter)idx);

        if (g_currentLayer == LAYER_DISPENSED_EDIT) {
          snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu dispensed"), (unsigned long)v);
        } else {
          char shortName[17];
          copyProgmemTableEntry(shortName, DISPENSE_SHORT_NAMES, idx, 4, sizeof(shortName));
          snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%s:%lu"), shortName, (unsigned long)v);
        }
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("TOTAL:%lu"), (unsigned long)G->ee->totalDispensedUnits());
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_BACKWASH_MENU: {
      if (g_bwMenuCursor == BW_MENU_DUR) {
        const unsigned long secs = (unsigned long)G->ee->backwashDuration() * 5UL;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu s"), secs);
      } else if (g_bwMenuCursor == BW_MENU_NDISP) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u pulse"), (unsigned)G->ee->backwashAfterNDispenses());
      } else if (g_bwMenuCursor == BW_MENU_COUNTER) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u pulse"), (unsigned)G->ee->backwashDispenseCounter());
      } else if (g_bwMenuCursor == BW_MENU_DAILY_DUR) {
        const unsigned long totalSecs = (unsigned long)G->ee->dailyBackwashDuration10s() * 10UL;
        if (totalSecs == 0UL) snprintf_P(tempBuf, sizeof(tempBuf), PSTR("0 mins"));
        else if ((totalSecs % 60UL) == 0UL) snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu mins"), totalSecs / 60UL);
        else snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu s"), totalSecs);
      } else if (g_bwMenuCursor == BW_MENU_DAILY_TIME) {
        uint16_t minutes = G->ee->dailyBackwashTimeMinutes();
        if (minutes > 1439U) minutes = 0U;
        const uint8_t hh = (uint8_t)(minutes / 60U);
        const uint8_t mm = (uint8_t)(minutes % 60U);
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%02u:%02u"), (unsigned)hh, (unsigned)mm);
      } else if (g_bwMenuCursor == BW_MENU_MAN_SHORT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"), (unsigned)G->ee->backwashManualShortSeconds());
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"), (unsigned)G->ee->backwashManualLong2Minutes() * 2U);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_KLARAN_MENU: {
      if (g_uvMenuCursor == UV_MENU_OK_DELAY) {
        const unsigned long ms = (unsigned long)G->ee->klaranUvOkDelay10ms() * 10UL;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"), ms);
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"), (unsigned)G->ee->klaranUvMaxOntimeMinutes());
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_SENSORBYP_MENU: {
      if (g_sensorBypMenuCursor == SBYP_MENU_DUR) {
        const uint8_t units = G->ee->sensorBypassDuration100ms();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"),
                   (unsigned)(units / 10U), (unsigned)(units % 10U));
      } else if (g_sensorBypMenuCursor == SBYP_MENU_PERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)G->ee->sensorBypassPeriodMinutes());
      } else if (g_sensorBypMenuCursor == SBYP_MENU_MAN_SHORT) {
        const uint8_t units = G->ee->sensorBypassManualShort100ms();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"),
                   (unsigned)(units / 10U), (unsigned)(units % 10U));
      } else {
        const unsigned secs = (unsigned)G->ee->sensorBypassManualLong10s() * 10U;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"), secs);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_WCIRC_MENU: {
      if (g_wcircMenuCursor == WCIRC_MENU_AUTODUR) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)G->ee->autoCircDurationMinutes());
      } else if (g_wcircMenuCursor == WCIRC_MENU_AUTOPERIOD) {
        const unsigned mins = (unsigned)G->ee->autoCircPeriod10Minutes() * 10U;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"), mins);
      } else if (g_wcircMenuCursor == WCIRC_MENU_MANSHORT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)G->ee->manCircDurationShortMinutes());
      } else {
        const unsigned mins = (unsigned)G->ee->manCircDurationLong10Minutes() * 10U;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"), mins);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_PFLUSH_MENU: {
      if (g_pflushMenuCursor == PFLUSH_MENU_DNF_PERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)G->ee->dnfReptPeriodMinutes());
      } else if (g_pflushMenuCursor == PFLUSH_MENU_DNF_DURAT) {
        const uint8_t u = G->ee->dnfReptDurat100ms();
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"),
                   (unsigned)(u / 10U), (unsigned)(u % 10U));
      } else if (g_pflushMenuCursor == PFLUSH_MENU_PREFLSH_BEEP_DELY) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"),
                   (unsigned)G->ee->preFlshBeepDelySeconds());
      } else if (g_pflushMenuCursor == PFLUSH_MENU_BEEP_PERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"),
                   (unsigned long)G->ee->beepOffTime100ms() * 100UL);
      } else if (g_pflushMenuCursor == PFLUSH_MENU_BEEP_ON) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"),
                   (unsigned long)G->ee->beepOnTime10ms() * 10UL);
      } else if (g_pflushMenuCursor == PFLUSH_MENU_BEEP_COUNT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u"),
                   (unsigned)G->ee->beepCount());
      } else {
        tempBuf[0] = '\0';
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_BACKWASH_EDIT: {
      // Backwash auto duration units: 1 step = 5 seconds.
      if (g_currentBwIndex == BW_MENU_DUR) {
        unsigned long secs = (unsigned long)g_backwashDurCache * 5UL;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu s"), secs);
      } else if (g_currentBwIndex == BW_MENU_NDISP) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u pulse"), (unsigned)g_backwashNDispCache);
      } else if (g_currentBwIndex == BW_MENU_MAN_SHORT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"), (unsigned)g_backwashManShortCache);
      } else if (g_currentBwIndex == BW_MENU_MAN_LONG) {
        const unsigned mins = (unsigned)g_backwashManLongCache * 2U;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"), mins);
      } else if (g_currentBwIndex == BW_MENU_DAILY_TIME) {
        uint16_t minutes = g_backwashDailyTimeCache;
        if (minutes > 1439) minutes = 0;
        const uint8_t hh = (uint8_t)(minutes / 60U);
        const uint8_t mm = (uint8_t)(minutes % 60U);
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%02u:%02u"), (unsigned)hh, (unsigned)mm);
      } else if (g_currentBwIndex == BW_MENU_DAILY_DUR) {
        const unsigned long totalSecs = (unsigned long)g_backwashDailyDurCache * 10UL;
        if (totalSecs == 0UL) snprintf_P(tempBuf, sizeof(tempBuf), PSTR("0 mins"));
        else if ((totalSecs % 60UL) == 0UL) snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu mins"), totalSecs / 60UL);
        else snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu s"), totalSecs);
      } else { // BW_MENU_COUNTER
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u pulse"), (unsigned)g_backwashCounterCache);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }


    case LAYER_KLARAN_EDIT: {
      // KlaranUV config values are stored as compact uint8_t values in EEPROM:
      //  - UV OK Delay: 10ms increments, displayed as milliseconds here.
      //  - UV MAX ON time: stored in minutes, displayed as minutes here.
      if (g_currentUvIndex == UV_MENU_OK_DELAY) {
        const unsigned long ms = (unsigned long)g_uvOkDelayCache * 10UL;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"), ms);
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"), (unsigned)g_uvMaxOntimeCache);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_SENSORBYP_EDIT: {
      if (g_currentSensorBypIndex == SBYP_MENU_DUR) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"),
                   (unsigned)(g_sensorBypDurCache / 10U),
                   (unsigned)(g_sensorBypDurCache % 10U));
      } else if (g_currentSensorBypIndex == SBYP_MENU_PERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)g_sensorBypPeriodCache);
      } else if (g_currentSensorBypIndex == SBYP_MENU_MAN_SHORT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"),
                   (unsigned)(g_sensorBypManShortCache / 10U),
                   (unsigned)(g_sensorBypManShortCache % 10U));
      } else {
        const unsigned secs = (unsigned)g_sensorBypManLongCache * 10U;
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"), secs);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_WCIRC_EDIT: {
      if (g_currentWcircIndex == WCIRC_MENU_AUTODUR) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)g_autoCircDurCache);
      } else if (g_currentWcircIndex == WCIRC_MENU_AUTOPERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)g_autoCircPeriodCache * 10U);
      } else if (g_currentWcircIndex == WCIRC_MENU_MANSHORT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)g_manCircShortCache);
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)g_manCircLongCache * 10U);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_PFLUSH_EDIT: {
      if (g_currentPflushIndex == PFLUSH_MENU_DNF_PERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u mins"),
                   (unsigned)g_dnfReptPeriodCache);
      } else if (g_currentPflushIndex == PFLUSH_MENU_DNF_DURAT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u.%u s"),
                   (unsigned)(g_dnfReptDuratCache / 10U),
                   (unsigned)(g_dnfReptDuratCache % 10U));
      } else if (g_currentPflushIndex == PFLUSH_MENU_PREFLSH_BEEP_DELY) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u s"),
                   (unsigned)g_preFlshBeepDelyCache);
      } else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_PERIOD) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"),
                   (unsigned long)g_beepOffTimeCache * 100UL);
      } else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_ON) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"),
                   (unsigned long)g_beepOnTimeCache * 10UL);
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%u"),
                   (unsigned)g_beepCountCache);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_WATERTEMP_EDIT: {
      // WaterTempSense is stored as 1 (T1) or 2 (T2).
      const char* t = (g_waterTempSenseCache == 2) ? "T2" : "T1";
      snprintf_P(tempBuf, sizeof(tempBuf), PSTR("Use %s"), t);
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_COINACC_EDIT: {
      const char* v = g_coinAccFittedCache ? "YES" : "NO";
      snprintf_P(tempBuf, sizeof(tempBuf), PSTR("Fitted ? %s"), v);
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_RTC_CONFIG:
      // RTC display is rendered from cached values to avoid flicker.
      fillDisplayRow(rowBuf, "");
      break;

    case LAYER_NFC_PARMS_MENU: {
      // Preview the current value of the selected NFC parameter.
      unsigned long ms = 0;
      if (g_nfcParmsMenuCursor == NFC_PARMS_INIT_DELAY) ms = (unsigned long)G->ee->nfcInitDelayMs();
      else if (g_nfcParmsMenuCursor == NFC_PARMS_READ_DURATION) ms = (unsigned long)G->ee->nfcReadDurationMs();
      else ms = (unsigned long)G->ee->nfcInterNfcDelayMs();

      snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"), ms);

      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_NFC_PARMS_EDIT: {
      // NFC timing values are edited in milliseconds.
      unsigned long ms = 0;
      if (g_currentNfcParmIndex == NFC_PARMS_INIT_DELAY) ms = (unsigned long)g_nfcInitDelayMsCache;
      else if (g_currentNfcParmIndex == NFC_PARMS_READ_DURATION) ms = (unsigned long)g_nfcReadDurationMsCache;
      else ms = (unsigned long)g_nfcInterDelayMsCache;

      snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%lu ms"), ms);
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_NFC_BROWSE: {
      // Browse shows index + hash; delete prompt overlays YES/NO.
      uint32_t h = G->ee->tokenHashAt(g_tokenBrowseIndex);
      char hs[9]; formatHash32(h, hs);
      if (g_tokenState == TOKEN_STATE_DELETE_PROMPT) {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("Del? %s %s"), hs, g_tokenDeleteConfirm ? "YES" : "NO");
      } else {
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("I %03u %s"), (unsigned)g_tokenBrowseIndex, hs);
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_NFC_ADD:
    case LAYER_NFC_DEL: {
      // NFC add/del: show current scanned hash if present, else prompt.
      uint32_t h = (g_nfcHash2 != 0) ? g_nfcHash2 : g_nfcHash1;
      if (h) {
        char hs[9]; formatHash32(h, hs);
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%s %s"), (g_currentLayer == LAYER_NFC_ADD) ? "ADD" : "DEL", hs);
      } else {
        strcpy(tempBuf, "Tap tag...");
      }
      fillDisplayRow(rowBuf, tempBuf);
      break;
    }

    case LAYER_REINIT:
      // Reinit screen has a two-step safety:
      // 1) short-select enters “prompt active”
      // 2) user must toggle confirm to YES and hold SELECT >= 10s
      if (!g_reinitPromptActive) {
        copyProgmemString(labelBuf, STR_REINIT_PROMPT, sizeof(labelBuf));
        fillDisplayRow(rowBuf, labelBuf);
      } else {
        if (g_reinitConfirmYes) {
          if (g_reinitHoldActive) {
            unsigned long t = millis();
            unsigned long held = (t >= g_reinitHoldStartMs) ? (t - g_reinitHoldStartMs) : 0;
            unsigned long remain = (held >= REINIT_START_HOLD_MS) ? 0 : (REINIT_START_HOLD_MS - held);
            snprintf_P(tempBuf, sizeof(tempBuf), PSTR("YES Hold %lus"),
                     (unsigned long)((remain + 999UL)/1000UL));
          } else {
            strcpy(tempBuf, "YES Hold 10s");
          }
        } else {
          strcpy(tempBuf, "Confirm:NO");
        }
        fillDisplayRow(rowBuf, tempBuf);
      }
      break;

    case LAYER_NFC_MENU:
      // Remove-all prompt is a modal overlay within NFC menu.
      if (g_removeAllPromptActive) {
        copyProgmemString(labelBuf, STR_REMOVEALL_PROMPT, sizeof(labelBuf));
        snprintf_P(tempBuf, sizeof(tempBuf), PSTR("%s %s"), labelBuf, g_removeAllConfirm ? "YES" : "NO");
        fillDisplayRow(rowBuf, tempBuf);
      }
      break;

    default:
      break;
  }
  const bool row13CenteredValue =
    (g_currentLayer == LAYER_KLARAN_MENU ||
     g_currentLayer == LAYER_KLARAN_EDIT ||
     g_currentLayer == LAYER_NFC_PARMS_MENU ||
     g_currentLayer == LAYER_NFC_PARMS_EDIT ||
     g_currentLayer == LAYER_BACKWASH_MENU ||
     g_currentLayer == LAYER_BACKWASH_EDIT ||
     g_currentLayer == LAYER_SENSORBYP_MENU ||
     g_currentLayer == LAYER_SENSORBYP_EDIT ||
     g_currentLayer == LAYER_WD_MENU ||
     g_currentLayer == LAYER_WD_EDIT ||
     g_currentLayer == LAYER_WCIRC_MENU ||
     g_currentLayer == LAYER_WCIRC_EDIT ||
     g_currentLayer == LAYER_PFLUSH_MENU ||
     g_currentLayer == LAYER_PFLUSH_EDIT ||
     g_currentLayer == LAYER_DISPENSED_MENU ||
     g_currentLayer == LAYER_DISPENSED_EDIT);
  if (row13CenteredValue) {
    memcpy(row13Value, rowBuf, sizeof(row13Value));
    fillDisplayRow(rowBuf, "");
  }
  if (g_currentLayer != LAYER_RTC_CONFIG) {
    oledWriteRow(12, rowBuf);
    if (row13CenteredValue) {
      char centered[17];
      centerText16(centered, row13Value);
      memcpy(rowBuf, centered, sizeof(centered));
    } else {
      fillDisplayRow(rowBuf, "");
    }
    oledWriteRow(13, rowBuf);
  }

  // Row 14 remains blank for editor layers.
  fillDisplayRow(rowBuf, "");
  oledWriteRow(14, rowBuf);

  // Row 15: hint/status line.
  if (g_showSavingMessage) {
    copyProgmemString(labelBuf, STR_SAVING, sizeof(labelBuf));
    fillDisplayRow(rowBuf, labelBuf);
  } else if (g_showSavedMessage) {
    copyProgmemString(labelBuf, STR_SAVED, sizeof(labelBuf));
    fillDisplayRow(rowBuf, labelBuf);
  } else {
    copyProgmemString(labelBuf, STR_HINT, sizeof(labelBuf));
    fillDisplayRow(rowBuf, labelBuf);
  }
  oledWriteRow(15, rowBuf);
}

// -------------------- Saving indicators --------------------
// beginSaving/finishSaving are called around EEPROM writes.
// They update both OLED row 15 and LCD line1 and arm the timers that keep text visible.
static void beginSaving() {
  g_showSavingMessage = true;
  g_showSavedMessage = false;

   // LCD status disabled: do not show "Saving"
   // lcdWriteFixed(1, "Saving");
   lcdWriteFixed(1, "");
  g_lcdStatusActive = true;
  g_lcdStatusStartMs = millis();

  forceOledStatusLine(STR_SAVING);
}

static void finishSaving() {
  g_showSavingMessage = false;
  g_showSavedMessage = true;
  g_savedStartMs = millis();

   // LCD status disabled: do not show "Saved !"
   // lcdWriteFixed(1, "Saved !");
   lcdWriteFixed(1, "");
  g_lcdStatusActive = true;
  g_lcdStatusStartMs = millis();

  forceOledStatusLine(STR_SAVED);
}

// saveIfDirtyForLayer(): commits cached values based on current edit layer.
// Note: This intentionally does not “save everything”; it saves only the object being edited.
static bool saveIfDirtyForLayer() {
  bool didWrite = false;

  switch (g_currentLayer) {
    case LAYER_WD_EDIT:
      if (g_wdCacheDirty) {
        if (!didWrite) beginSaving();
        G->ee->storeMeasuredProfile(g_wdProfileCache);
        g_wdCacheDirty = false;
        didWrite = true;
      }
      if (g_preDispDirty) {
        if (!didWrite) beginSaving();
        G->ee->setPreDispCircTSeconds(g_preDispCircTCache);
        G->ee->setPreDispCircP100Pulses(g_preDispCircPCache);
        G->ee->setPreDispPurgTSeconds(g_preDispPurgTCache);
        G->ee->setPreDispPurgP100Pulses(g_preDispPurgPCache);
        g_preDispDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_SOL_PARAM_EDIT:
      if (g_solCacheDirty) {
        if (!didWrite) beginSaving();
        G->ee->storeSolenoidProfile((KioskEeprom::Solenoid)g_currentSolenoidIndex, g_solProfileCache);
        g_solCacheDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_DISPENSED_EDIT:
      if (g_dispCacheDirty) {
        if (!didWrite) beginSaving();
        G->ee->setDispenseCounter((KioskEeprom::DispenseCounter)g_currentDispenseIndex, g_dispCounterCache);
        g_dispCacheDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_BACKWASH_EDIT:
      if (g_bwCacheDirty) {
        if (!didWrite) beginSaving();
        if (g_currentBwIndex == BW_MENU_DUR)     G->ee->setBackwashDuration(g_backwashDurCache);
        if (g_currentBwIndex == BW_MENU_NDISP)   G->ee->setBackwashAfterNDispenses(g_backwashNDispCache);
        if (g_currentBwIndex == BW_MENU_COUNTER) G->ee->setBackwashDispenseCounter(g_backwashCounterCache);
        if (g_currentBwIndex == BW_MENU_MAN_SHORT) G->ee->setBackwashManualShortSeconds(g_backwashManShortCache);
        if (g_currentBwIndex == BW_MENU_MAN_LONG)  G->ee->setBackwashManualLong2Minutes(g_backwashManLongCache);
        if (g_currentBwIndex == BW_MENU_DAILY_TIME) G->ee->setDailyBackwashTimeMinutes(g_backwashDailyTimeCache);
        if (g_currentBwIndex == BW_MENU_DAILY_DUR)  G->ee->setDailyBackwashDuration10s(g_backwashDailyDurCache);
        g_bwCacheDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_KLARAN_EDIT:
      if (g_uvCacheDirty) {
        if (!didWrite) beginSaving();
        // Save both KlaranUV parameters together (both are cached).
        G->ee->setKlaranUvOkDelay10ms(g_uvOkDelayCache);
        G->ee->setKlaranUvMaxOntimeMinutes(g_uvMaxOntimeCache);
        g_uvCacheDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_SENSORBYP_EDIT:
      if (g_sensorBypDirty) {
        if (!didWrite) beginSaving();
        G->ee->setSensorBypassDuration100ms(g_sensorBypDurCache);
        G->ee->setSensorBypassPeriodMinutes(g_sensorBypPeriodCache);
        G->ee->setSensorBypassManualShort100ms(g_sensorBypManShortCache);
        G->ee->setSensorBypassManualLong10s(g_sensorBypManLongCache);
        g_sensorBypDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_WCIRC_EDIT:
      if (g_wcircDirty) {
        if (!didWrite) beginSaving();
        G->ee->setAutoCircDurationMinutes(g_autoCircDurCache);
        G->ee->setAutoCircPeriod10Minutes(g_autoCircPeriodCache);
        G->ee->setManCircDurationShortMinutes(g_manCircShortCache);
        G->ee->setManCircDurationLong10Minutes(g_manCircLongCache);
        g_wcircDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_PFLUSH_EDIT:
      if (g_pflushDirty) {
        if (!didWrite) beginSaving();
        G->ee->setDnfReptPeriodMinutes(g_dnfReptPeriodCache);
        G->ee->setDnfReptDurat100ms(g_dnfReptDuratCache);
        G->ee->setPreFlshBeepDelySeconds(g_preFlshBeepDelyCache);
        G->ee->setBeepOnTime10ms(g_beepOnTimeCache);
        G->ee->setBeepOffTime100ms(g_beepOffTimeCache);
        G->ee->setBeepCount(g_beepCountCache);
        g_pflushDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_WATERTEMP_EDIT:
      if (g_waterTempSenseDirty) {
        if (!didWrite) beginSaving();
        G->ee->setWaterTempSense(g_waterTempSenseCache); // clamps to 1..2
        g_waterTempSenseDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_COINACC_EDIT:
      if (g_coinAccFittedDirty) {
        if (!didWrite) beginSaving();
        G->ee->setCoinAcceptorFitted(g_coinAccFittedCache);
        g_coinAccFittedDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_NFC_PARMS_EDIT:
      if (g_nfcParmsDirty) {
        if (!didWrite) beginSaving();
        G->ee->setNfcInitDelayMs(g_nfcInitDelayMsCache);
        G->ee->setNfcReadDurationMs(g_nfcReadDurationMsCache);
        G->ee->setNfcInterNfcDelayMs(g_nfcInterDelayMsCache);
        g_nfcParmsDirty = false;
        didWrite = true;
      }
      break;

    case LAYER_PCB:
      if (g_pcbIdDirty) {
        if (!didWrite) beginSaving();
        G->ee->setHwId(g_workingPcbId);
        g_pcbIdDirty = false;
        renderLcdBase(); // HWID line2 depends on this.
        didWrite = true;
      }
      break;

    case LAYER_RTC_CONFIG:
      if (g_rtcDirty && g_rtcPresent) {
        uint16_t y = 2000;
        uint8_t mo = 1, d = 1, h = 0, mi = 0, s = 0;
        rtcFieldsFromDigits(g_rtcDigits, y, mo, d, h, mi, s);
        rtcClampFields(y, mo, d, h, mi, s);
        if (!didWrite) beginSaving();
        (void)rtcWriteDs3231(y, mo, d, h, mi, s);
        rtcUpdateInfoLinesFromDigits();
        g_rtcDirty = false;
        didWrite = true;
      }
      break;

    default:
      break;
  }

  if (didWrite) finishSaving();
  return didWrite;
}

// -------------------- Full reset helpers --------------------
// restartEditorUiAfterReset(): returns to a known “fresh” UI state after reset.
static void restartEditorUiAfterReset() {
  g_reinitPostWaitActive = false;
  g_reinitPromptActive = false;
  g_reinitConfirmYes = false;
  g_reinitHoldActive = false;

  g_removeAllPromptActive = false;
  g_removeAllConfirm = false;

  g_tokenState = TOKEN_STATE_BROWSE;
  g_tokenDeleteConfirm = false;

  g_workingPcbId = G->ee->hwId();

  renderLcdBase();
  clearLcdLine1();
  g_lcdStatusActive = false;

  g_showSavingMessage = false;
  g_showSavedMessage  = false;

  setLayer(LAYER_TOP);
}

// startEepromFullResetNow(): executes the full EEPROM default re-initialisation.
static void startEepromFullResetNow() {
  g_showSavingMessage = false;
  g_showSavedMessage  = false;
  g_lcdStatusActive   = true;
  g_lcdStatusStartMs  = millis();

  forceOledStatusLine(STR_RESETTING);
   // LCD status disabled: do not show "Resetting"
   // lcdWriteFixed(1, "Resetting");
   lcdWriteFixed(1, "");

  // Preserve HWID across operator-triggered resets only when EEPROM magic is
  // already valid (same layout). If magic changed/invalid, allow HWID reset.
  const bool preserveHwid = G->ee->magicValid();
  const uint8_t preservedHwid = preserveHwid ? G->ee->hwId() : 0U;

  // Re-initialise EEPROM contents. This is expected to be a blocking operation.
  G->ee->reinitializeToDefaults(true);
  if (preserveHwid) {
    G->ee->setHwId(preservedHwid);
  }

  forceOledStatusLine(STR_COMPLETED);
   // LCD status disabled: do not show "Completed"
   // lcdWriteFixed(1, "Completed");
   lcdWriteFixed(1, "");

  // Post-wait state ensures operator sees “Completed”.
  g_reinitPostWaitActive = true;
  g_reinitPostWaitStartMs = millis();
}

// startNfcTablePackNow(): executes the NFC token-table pack/compact operation.
// This is expected to be a blocking operation (EEPROM reads/writes).
static void startNfcTablePackNow() {
  // Use the standard Saving/Saved messaging used for all EEPROM write actions.
  // Note: packTokenHashTable() is a blocking EEPROM read/write operation.
  beginSaving();

  // Pack + de-duplicate token table.
  G->ee->packTokenHashTable();

  finishSaving();
}

// -------------------- Value editing (UP/DOWN) --------------------
// handleUpDown(): unified handler for menu navigation and numeric editing.
//
// NOTE on direction:
// - g_btnUp.direction  = -1 (menu previous)
// - g_btnDown.direction= +1 (menu next)
// - For numeric edit layers we invert so UP increments and DOWN decrements,
//   which is more intuitive for operators.
static inline bool layerInvertsUpDown(uint8_t layer) {
  return (layer == LAYER_PCB ||
          layer == LAYER_WD_EDIT ||
          layer == LAYER_SOL_PARAM_EDIT ||
          layer == LAYER_DISPENSED_EDIT ||
          layer == LAYER_BACKWASH_EDIT ||
          layer == LAYER_KLARAN_EDIT ||
          layer == LAYER_SENSORBYP_EDIT ||
          layer == LAYER_WCIRC_EDIT ||
          layer == LAYER_PFLUSH_EDIT ||
          layer == LAYER_WATERTEMP_EDIT ||
          layer == LAYER_COINACC_EDIT ||
          layer == LAYER_RTC_CONFIG ||
          layer == LAYER_NFC_PARMS_EDIT);
}

static inline bool isTimedDispenseEdit() {
  return (g_currentLayer == LAYER_WD_EDIT && g_wdEditField == WD_FIELD_TIME);
}

static void applyTimedDispenseStep(int8_t dir, uint16_t step100ms) {
  int32_t v = (int32_t)g_wdProfileCache.duration100ms + (int32_t)dir * (int32_t)step100ms;
  // Stored in 100ms units; clamp to 10 minutes => 6000 * 100ms.
  g_wdProfileCache.duration100ms = (uint16_t)clamp32(v, 0, 6000);
  g_wdCacheDirty = true;
}

static void alignTimedDispenseForStep(uint16_t step100ms) {
  uint16_t v = g_wdProfileCache.duration100ms;
  if (step100ms == 10) {
    // Zero the 100ms digit (round down to nearest 1s).
    v = (uint16_t)(v - (v % 10));
  } else if (step100ms == 100) {
    // Zero both 1s and 100ms digits (round down to nearest 10s).
    v = (uint16_t)(v - (v % 100));
  }
  g_wdProfileCache.duration100ms = v;
}

static inline bool handleMenuNavigation(int8_t dir) {
  if (g_currentLayer == LAYER_WD_MENU) {
    moveCursorWrapped(g_wdMenuCursor, WD_MENU_COUNT, dir);
    if (g_wdMenuCursor < g_wdMenuTop) g_wdMenuTop = g_wdMenuCursor;
    const uint8_t windowRows = 8; // OLED rows 4..11 for lower-level menus
    if (g_wdMenuCursor >= (uint8_t)(g_wdMenuTop + windowRows)) {
      g_wdMenuTop = (uint8_t)(g_wdMenuCursor - (windowRows - 1));
    }
    const uint8_t maxTop = (WD_MENU_COUNT > windowRows) ? (uint8_t)(WD_MENU_COUNT - windowRows) : 0;
    if (g_wdMenuTop > maxTop) g_wdMenuTop = maxTop;
    return true;
  }

  uint8_t* cursor = nullptr;
  uint8_t count = 0;

  switch (g_currentLayer) {
    case LAYER_TOP:            cursor = &g_topCursor;        count = TOP_COUNT; break;
    case LAYER_KIOSKPARM_MENU: cursor = &g_kioskParmCursor;  count = KPARM_MENU_COUNT; break;
    case LAYER_HYDRPARM_MENU:  cursor = &g_hydrParmCursor;   count = HPARM_MENU_COUNT; break;
    case LAYER_WD_MENU:        cursor = &g_wdMenuCursor;     count = WD_MENU_COUNT; break;
    case LAYER_SOL_MENU:       cursor = &g_solMenuCursor;    count = KioskEeprom::PWM_SOLENOID_COUNT; break;
    case LAYER_DISPENSED_MENU: cursor = &g_dispMenuCursor;   count = DISP_MENU_COUNT; break;
    case LAYER_BACKWASH_MENU:  cursor = &g_bwMenuCursor;     count = BW_MENU_COUNT; break;
    case LAYER_KLARAN_MENU:    cursor = &g_uvMenuCursor;     count = UV_MENU_COUNT; break;
    case LAYER_SENSORBYP_MENU: cursor = &g_sensorBypMenuCursor; count = SBYP_MENU_COUNT; break;
    case LAYER_WCIRC_MENU:     cursor = &g_wcircMenuCursor;  count = WCIRC_MENU_COUNT; break;
    case LAYER_PFLUSH_MENU:    cursor = &g_pflushMenuCursor; count = PFLUSH_MENU_COUNT; break;
    case LAYER_NFC_PARMS_MENU: cursor = &g_nfcParmsMenuCursor; count = NFC_PARMS_COUNT; break;
    case LAYER_NFC_MENU:       cursor = &g_nfcMenuCursor;    count = NFC_MENU_COUNT; break;
    default:
      return false;
  }

  moveCursorWrapped(*cursor, count, dir);
  return true;
}

static void handleUpDown(int8_t dir) {
  if (layerInvertsUpDown(g_currentLayer)) dir = -dir;

  // Modal prompts override normal navigation.
  if (g_removeAllPromptActive) { g_removeAllConfirm = !g_removeAllConfirm; return; }

  if (g_currentLayer == LAYER_NFC_BROWSE && g_tokenState == TOKEN_STATE_DELETE_PROMPT) {
    g_tokenDeleteConfirm = !g_tokenDeleteConfirm;
    return;
  }

  if (g_currentLayer == LAYER_REINIT && g_reinitPromptActive) {
    g_reinitConfirmYes = !g_reinitConfirmYes;
    g_reinitHoldActive = false;
    return;
  }

  // Menu navigation.
  if (handleMenuNavigation(dir)) return;

  // Numeric editing.
  switch (g_currentLayer) {
    case LAYER_PCB:
      g_workingPcbId = (uint8_t)(g_workingPcbId + dir);
      g_pcbIdDirty = true;
      renderLcdBase();
      break;

    case LAYER_WD_EDIT:
      if (g_wdEditField == WD_FIELD_MODESEL) {
        g_wdProfileCache.modeSel =
          (g_wdProfileCache.modeSel == KioskEeprom::WD_MODE_TIME)
            ? KioskEeprom::WD_MODE_PULSES
            : KioskEeprom::WD_MODE_TIME;
        g_wdCacheDirty = true;
      } else if (g_wdEditField == WD_FIELD_TIME) {
        applyTimedDispenseStep(dir, 1);
      } else if (g_wdEditField == WD_FIELD_PULSES) {
        int32_t v = (int32_t)g_wdProfileCache.pulses + (int32_t)dir * 1;
        g_wdProfileCache.pulses = (uint16_t)clamp32(v, 0, 65535);
        g_wdCacheDirty = true;
      } else if (g_wdEditField == WD_FIELD_PREDISP_CIRCT) {
        int32_t v = (int32_t)g_preDispCircTCache + (int32_t)dir;
        g_preDispCircTCache = (uint8_t)clamp32(v, 0, 255);
        g_preDispDirty = true;
      } else if (g_wdEditField == WD_FIELD_PREDISP_CIRCP) {
        int32_t v = (int32_t)g_preDispCircPCache + (int32_t)dir;
        g_preDispCircPCache = (uint8_t)clamp32(v, 0, 255);
        g_preDispDirty = true;
      } else if (g_wdEditField == WD_FIELD_PREDISP_PURGT) {
        int32_t v = (int32_t)g_preDispPurgTCache + (int32_t)dir;
        g_preDispPurgTCache = (uint8_t)clamp32(v, 0, 255);
        g_preDispDirty = true;
      } else {
        int32_t v = (int32_t)g_preDispPurgPCache + (int32_t)dir;
        g_preDispPurgPCache = (uint8_t)clamp32(v, 0, 255);
        g_preDispDirty = true;
      }
      break;

    case LAYER_SOL_PARAM_EDIT:
      switch (g_solParamCursor) {
        case SOL_PARAM_STARTPWM:
          g_solProfileCache.startPwm = (uint8_t)clamp32((int32_t)g_solProfileCache.startPwm + dir, 0, 255);
          g_solCacheDirty = true;
          break;
        case SOL_PARAM_HOLDPWM:
          g_solProfileCache.holdPwm  = (uint8_t)clamp32((int32_t)g_solProfileCache.holdPwm + dir, 0, 255);
          g_solCacheDirty = true;
          break;
        case SOL_PARAM_SWDELAY:
          g_solProfileCache.swDelaySec = (uint8_t)clamp32((int32_t)g_solProfileCache.swDelaySec + dir, 0, 60);
          g_solCacheDirty = true;
          break;
      }
      break;

    case LAYER_DISPENSED_EDIT: {
      int32_t v = (int32_t)g_dispCounterCache + (int32_t)dir * 1;
      g_dispCounterCache = (uint32_t)clamp32(v, 0, 2000000000L);
      g_dispCacheDirty = true;
      break;
    }

    case LAYER_BACKWASH_EDIT:
      if (g_currentBwIndex == BW_MENU_DUR) {
        g_backwashDurCache = (uint8_t)clamp32((int32_t)g_backwashDurCache + dir, 0, 255);
        g_bwCacheDirty = true;
      } else if (g_currentBwIndex == BW_MENU_NDISP) {
        g_backwashNDispCache = (uint8_t)clamp32((int32_t)g_backwashNDispCache + dir, 0, 255);
        g_bwCacheDirty = true;
      } else if (g_currentBwIndex == BW_MENU_MAN_SHORT) {
        g_backwashManShortCache = (uint8_t)clamp32((int32_t)g_backwashManShortCache + dir, 0, 255);
        g_bwCacheDirty = true;
      } else if (g_currentBwIndex == BW_MENU_MAN_LONG) {
        g_backwashManLongCache = (uint8_t)clamp32((int32_t)g_backwashManLongCache + dir, 0, 255);
        g_bwCacheDirty = true;
      } else if (g_currentBwIndex == BW_MENU_DAILY_TIME) {
        int32_t v = (int32_t)g_backwashDailyTimeCache + dir;
        g_backwashDailyTimeCache = (uint16_t)clamp32(v, 0, 1439);
        g_bwCacheDirty = true;
      } else if (g_currentBwIndex == BW_MENU_DAILY_DUR) {
        g_backwashDailyDurCache = (uint8_t)clamp32((int32_t)g_backwashDailyDurCache + dir, 0, 255);
        g_bwCacheDirty = true;
      } else {
        if (dir > 0) {
          if (g_backwashCounterCache < 0xFFFFU) g_backwashCounterCache++;
        } else if (dir < 0) {
          if (g_backwashCounterCache > 0) g_backwashCounterCache--;
        }
        g_bwCacheDirty = true;
      }
      break;

    case LAYER_KLARAN_EDIT:
      if (g_currentUvIndex == UV_MENU_OK_DELAY) {
        g_uvOkDelayCache = (uint8_t)clamp32((int32_t)g_uvOkDelayCache + dir, 0, 255);
        g_uvCacheDirty = true;
      } else {
        g_uvMaxOntimeCache = (uint8_t)clamp32((int32_t)g_uvMaxOntimeCache + dir, 0, 255);
        g_uvCacheDirty = true;
      }
      break;

    case LAYER_SENSORBYP_EDIT:
      if (g_currentSensorBypIndex == SBYP_MENU_DUR) {
        g_sensorBypDurCache = (uint8_t)clamp32((int32_t)g_sensorBypDurCache + dir, 0, 255);
      } else if (g_currentSensorBypIndex == SBYP_MENU_PERIOD) {
        g_sensorBypPeriodCache = (uint8_t)clamp32((int32_t)g_sensorBypPeriodCache + dir, 0, 255);
      } else if (g_currentSensorBypIndex == SBYP_MENU_MAN_SHORT) {
        g_sensorBypManShortCache = (uint8_t)clamp32((int32_t)g_sensorBypManShortCache + dir, 0, 255);
      } else {
        g_sensorBypManLongCache = (uint8_t)clamp32((int32_t)g_sensorBypManLongCache + dir, 0, 255);
      }
      g_sensorBypDirty = true;
      break;

    case LAYER_WCIRC_EDIT:
      if (g_currentWcircIndex == WCIRC_MENU_AUTODUR) {
        g_autoCircDurCache = (uint8_t)clamp32((int32_t)g_autoCircDurCache + dir, 0, 255);
      } else if (g_currentWcircIndex == WCIRC_MENU_AUTOPERIOD) {
        g_autoCircPeriodCache = (uint8_t)clamp32((int32_t)g_autoCircPeriodCache + dir, 0, 255);
      } else if (g_currentWcircIndex == WCIRC_MENU_MANSHORT) {
        g_manCircShortCache = (uint8_t)clamp32((int32_t)g_manCircShortCache + dir, 0, 255);
      } else {
        g_manCircLongCache = (uint8_t)clamp32((int32_t)g_manCircLongCache + dir, 0, 255);
      }
      g_wcircDirty = true;
      break;

    case LAYER_PFLUSH_EDIT:
      if (g_currentPflushIndex == PFLUSH_MENU_DNF_PERIOD) {
        g_dnfReptPeriodCache = (uint8_t)clamp32((int32_t)g_dnfReptPeriodCache + dir, 0, 255);
      } else if (g_currentPflushIndex == PFLUSH_MENU_DNF_DURAT) {
        g_dnfReptDuratCache = (uint8_t)clamp32((int32_t)g_dnfReptDuratCache + dir, 0, 255);
      } else if (g_currentPflushIndex == PFLUSH_MENU_PREFLSH_BEEP_DELY) {
        g_preFlshBeepDelyCache = (uint8_t)clamp32((int32_t)g_preFlshBeepDelyCache + dir, 0, 255);
      } else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_PERIOD) {
        g_beepOffTimeCache = (uint8_t)clamp32((int32_t)g_beepOffTimeCache + dir, 0, 255);
      } else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_ON) {
        g_beepOnTimeCache = (uint8_t)clamp32((int32_t)g_beepOnTimeCache + dir, 0, 255);
      } else if (g_currentPflushIndex == PFLUSH_MENU_BEEP_COUNT) {
        g_beepCountCache = (uint8_t)clamp32((int32_t)g_beepCountCache + dir, 0, 255);
      }
      g_pflushDirty = true;
      break;

    case LAYER_WATERTEMP_EDIT: {
      // WaterTempSense: 1=T1, 2=T2 (wrap-around).
      uint8_t v = g_waterTempSenseCache;
      if (v < 1 || v > 2) v = 1;
      int32_t nv = (int32_t)v + (int32_t)dir;
      if (nv < 1) nv = 2;
      if (nv > 2) nv = 1;
      g_waterTempSenseCache = (uint8_t)nv;
      g_waterTempSenseDirty = true;
      break;
    }

    case LAYER_COINACC_EDIT:
      g_coinAccFittedCache = !g_coinAccFittedCache;
      g_coinAccFittedDirty = true;
      break;

    case LAYER_RTC_CONFIG:
      if (!g_rtcPresent) break;
      rtcSetClusterValue(g_rtcClusterIndex, dir);
      break;

    case LAYER_NFC_PARMS_EDIT: {
      // NFC timing values are edited in milliseconds.
      if (g_currentNfcParmIndex == NFC_PARMS_INIT_DELAY) {
        int32_t v = (int32_t)g_nfcInitDelayMsCache + (int32_t)dir * (int32_t)KioskEeprom::NFC_INIT_DELAY_STEP_MS;
        g_nfcInitDelayMsCache = (uint16_t)clamp32(
          v,
          (int32_t)KioskEeprom::NFC_INIT_DELAY_MIN_MS,
          (int32_t)KioskEeprom::NFC_INIT_DELAY_MAX_MS
        );
      } else if (g_currentNfcParmIndex == NFC_PARMS_READ_DURATION) {
        int32_t v = (int32_t)g_nfcReadDurationMsCache + (int32_t)dir * (int32_t)KioskEeprom::NFC_SCAN_DURATION_STEP_MS;
        g_nfcReadDurationMsCache = (uint16_t)clamp32(
          v,
          (int32_t)KioskEeprom::NFC_SCAN_DURATION_MIN_MS,
          (int32_t)KioskEeprom::NFC_SCAN_DURATION_MAX_MS
        );
      } else {
        int32_t v = (int32_t)g_nfcInterDelayMsCache + (int32_t)dir * (int32_t)KioskEeprom::NFC_INTER_DELAY_STEP_MS;
        g_nfcInterDelayMsCache = (uint16_t)clamp32(
          v,
          (int32_t)KioskEeprom::NFC_INTER_DELAY_MIN_MS,
          (int32_t)KioskEeprom::NFC_INTER_DELAY_MAX_MS
        );
      }
      g_nfcParmsDirty = true;
      break;
    }

    // NFC browse index:
    // - Up increments index
    // - Down decrements index
    // - Wrap at the table ends
    case LAYER_NFC_BROWSE: {
      const uint16_t maxIndex = (uint16_t)(KioskEeprom::EEPROM_TOKEN_MAX - 1);
      if (dir < 0) {
        g_tokenBrowseIndex = (g_tokenBrowseIndex >= maxIndex) ? 0 : (uint16_t)(g_tokenBrowseIndex + 1U);
      } else if (dir > 0) {
        g_tokenBrowseIndex = (g_tokenBrowseIndex == 0U) ? maxIndex : (uint16_t)(g_tokenBrowseIndex - 1U);
      }
      break;
    }

    default:
      break;
  }
}

// -------------------- Button repeat / long-press processing --------------------
// Repeat logic is intentionally simple and driven from loop() polling.
// With RC debounce on the button inputs, this is sufficient for stable UI feel.
static void processRepeatButton(RepeatButton& b, unsigned long now) {
  const bool cur = digitalRead(b.pin);
  const bool pressed = (cur == LOW);

  if (g_currentLayer == LAYER_RTC_CONFIG && g_rtcPresent) {
    if (b.lastState == HIGH && cur == LOW) {
      b.pressStartMs = now;
      b.lastRepeatMs = now;
      b.isRepeating = false;
      b.step100msApplied = 0;
      handleUpDown(b.direction);
    }

    if (pressed) {
      const unsigned long held = now - b.pressStartMs;
      if (held >= 2000) {
        const unsigned long rate = (held >= 10000) ? 50 : 500;
        if (!b.isRepeating) {
          b.isRepeating = true;
          b.lastRepeatMs = now;
          handleUpDown(b.direction);
        } else if ((now - b.lastRepeatMs) >= rate) {
          b.lastRepeatMs = now;
          handleUpDown(b.direction);
        }
      }
    } else {
      b.isRepeating = false;
      b.step100msApplied = 0;
    }

    b.lastState = cur;
    return;
  }

  if (b.lastState == HIGH && cur == LOW) {
    b.pressStartMs = now;
    b.lastRepeatMs = now;
    b.isRepeating = false;
    b.step100msApplied = 0;
    handleUpDown(b.direction);
  }

  if (pressed) {
    const unsigned long held = now - b.pressStartMs;

    if (isTimedDispenseEdit()) {
      unsigned long rate = 0;
      uint16_t step = 0;
      if (held >= 10000UL) {
        step = 100;   // 10s
        rate = 100UL;
      } else if (held >= 6000UL) {
        step = 10;    // 1s
        rate = 100UL;
      } else if (held >= 2000UL) {
        step = 1;     // 0.1s
        rate = 50UL;
      }

      if (step == 0) {
        b.isRepeating = false;
        b.step100msApplied = 0;
      } else if (!b.isRepeating) {
        if (step > b.step100msApplied) {
          alignTimedDispenseForStep(step);
        }
        b.isRepeating = true;
        b.step100msApplied = step;
        b.lastRepeatMs = now;
        applyTimedDispenseStep((int8_t)-b.direction, step);
      } else if ((now - b.lastRepeatMs) >= rate) {
        if (step > b.step100msApplied) {
          alignTimedDispenseForStep(step);
          b.step100msApplied = step;
        }
        b.lastRepeatMs = now;
        applyTimedDispenseStep((int8_t)-b.direction, step);
      }
    } else {
      const unsigned long rate = (held >= BTN_FAST_THRESHOLD) ? BTN_REPEAT_FAST_MS : BTN_REPEAT_RATE_MS;

      if (!b.isRepeating) {
        if (held >= BTN_REPEAT_DELAY_MS) {
          b.isRepeating = true;
          b.lastRepeatMs = now;
          handleUpDown(b.direction);
        }
      } else {
        if ((now - b.lastRepeatMs) >= rate) {
          b.lastRepeatMs = now;
          handleUpDown(b.direction);
        }
      }
    }
  } else {
    b.isRepeating = false;
    b.step100msApplied = 0;
  }

  b.lastState = cur;
}

static void shortSelect();
static void longSelect();

static void processSelectButton(unsigned long now) {
  const bool cur = digitalRead(g_btnSelect.pin);

  if (g_btnSelect.lastState == HIGH && cur == LOW) {
    g_btnSelect.pressStartMs = now;
    g_btnSelect.longHandled = false;
  }

  if (cur == LOW) {
    if (!g_btnSelect.longHandled && (now - g_btnSelect.pressStartMs) >= BTN_SELECT_LONG_MS) {
      g_btnSelect.longHandled = true;
      longSelect();
    }
  } else {
    if (g_btnSelect.lastState == LOW) {
      if (!g_btnSelect.longHandled) shortSelect();
    }
  }

  g_btnSelect.lastState = cur;
}

static void processBackButton(unsigned long now) {
  const bool cur = digitalRead(g_btnBack.pin);

  if (g_btnBack.lastState == HIGH && cur == LOW) {
    g_btnBack.pressStartMs = now;
    g_btnBack.longHandled = false;

    if (g_currentLayer == LAYER_RTC_CONFIG && g_rtcPresent) {
      if (g_rtcClusterIndex > 0) {
        g_rtcClusterIndex--;
      } else {
        goBackOneLayer();
      }
      consumeBackPress(now);
      g_btnBack.lastState = cur;
      return;
    }

    // In NFC Add/Del, BACK should exit immediately (no long-press reset).
    if (isNfcAddDelLayer(g_currentLayer)) {
      goBackOneLayer();
      consumeBackPress(now);
      g_btnBack.lastState = cur;
      return;
    }
  }

  if (cur == LOW) {
    if (!g_btnBack.longHandled && (now - g_btnBack.pressStartMs) >= BTN_BACK_LONG_MS) {
      g_btnBack.longHandled = true;
      resetNow();
    }
  } else {
    if (g_btnBack.lastState == LOW) {
      if (!g_btnBack.longHandled) goBackOneLayer();
    }
  }

  g_btnBack.lastState = cur;
}

// Reset chord: hold SELECT + guard pin >= 10s triggers full reset immediately.
// This provides an “emergency” reset mechanism independent of the menu flow.
static void processResetChord(unsigned long now) {
  if (G->pinReinitGuard == 255) return;

  const bool sel = readPressed(G->pinSelect);
  const bool guard = readPressed(G->pinReinitGuard);

  static bool chordActive = false;
  static unsigned long chordStartMs = 0;

  if (sel && guard) {
    if (!chordActive) {
      chordActive = true;
      chordStartMs = now;
    } else if ((now - chordStartMs) >= REINIT_START_HOLD_MS) {
      chordActive = false;
      startEepromFullResetNow();
    }
  } else {
    chordActive = false;
  }
}

// -------------------- NFC operations --------------------
// performNfcAddDel(): executes the NFC two-scan confirm workflow.
// It updates LCD line1 with “Waiting” and then a result prefix+hash.
// On completion, processNfcOpTimeout() will re-arm for the next token after a short display window.
static void performNfcAddDel(unsigned long now) {
  if (!G->nfc) return;
  if (!G->allowTokenWrites) return;
  if (g_currentLayer != LAYER_NFC_ADD && g_currentLayer != LAYER_NFC_DEL) return;

  if (g_nfcOpState == NFC_OP_STATE_IDLE) {
    g_nfcOpState = NFC_OP_STATE_SCAN1;
    g_nfcHash1 = g_nfcHash2 = 0;
    showLcdWaiting();
  }

  if (g_nfcOpState != NFC_OP_STATE_SCAN1 && g_nfcOpState != NFC_OP_STATE_SCAN2) return;

  uint32_t h = readNfcHash(NFC_SCAN_TIMEOUT_SEC);
  if (h == NFC_HASH_ABORT) {
    // User pressed BACK while scanning: exit NFC add/del immediately.
    setLayer(LAYER_NFC_MENU);
    consumeBackPress(now);
    return;
  }
  if (h == 0) {
    g_nfcOpState = NFC_OP_STATE_TIMEOUT;
    g_nfcOpMessageMs = now;
    showLcdNfcResult(STR_LCD_TOKN_INV, 0, now);
    return;
  }

  if (g_nfcOpState == NFC_OP_STATE_SCAN1) {
    g_nfcHash1 = h;
    g_nfcOpState = NFC_OP_STATE_SCAN2;
    return;
  }

  g_nfcHash2 = h;
  if (g_nfcHash1 != g_nfcHash2) {
    g_nfcOpState = NFC_OP_STATE_MISMATCH;
    g_nfcOpMessageMs = now;
    showLcdNfcResult(STR_LCD_TOKN_INV, 0, now);
    return;
  }

  if (g_currentLayer == LAYER_NFC_ADD) {
    bool ok = G->ee->addTokenHash(g_nfcHash1);
    if (ok) {
      g_nfcOpState = NFC_OP_STATE_SUCCESS;
      showLcdNfcResult(STR_LCD_TOKN_ADD, g_nfcHash1, now);
    } else if (G->ee->tokenHashExists(g_nfcHash1)) {
      g_nfcOpState = NFC_OP_STATE_DUPLICATE;
      showLcdNfcResult(STR_LCD_TOKN_DUP, g_nfcHash1, now);
    } else {
      g_nfcOpState = NFC_OP_STATE_FULL;
      showLcdNfcResult(STR_LCD_TOKN_FULL, 0, now);
    }
  } else {
    bool ok = G->ee->deleteTokenHash(g_nfcHash1);
    if (ok) {
      g_nfcOpState = NFC_OP_STATE_SUCCESS;
      showLcdNfcResult(STR_LCD_TOKN_DEL, g_nfcHash1, now);
    } else {
      g_nfcOpState = NFC_OP_STATE_NOT_FOUND;
      showLcdNfcResult(STR_LCD_TOKN_NOT, g_nfcHash1, now);
    }
  }

  g_nfcOpMessageMs = now;
}

// processNfcOpTimeout(): handles post-result delays and re-arms for the next token.
static void processNfcOpTimeout(unsigned long now) {
  if (g_currentLayer != LAYER_NFC_ADD && g_currentLayer != LAYER_NFC_DEL) return;

  if (g_nfcOpState == NFC_OP_STATE_SUCCESS ||
      g_nfcOpState == NFC_OP_STATE_MISMATCH ||
      g_nfcOpState == NFC_OP_STATE_TIMEOUT ||
      g_nfcOpState == NFC_OP_STATE_DUPLICATE ||
      g_nfcOpState == NFC_OP_STATE_NOT_FOUND ||
      g_nfcOpState == NFC_OP_STATE_FULL) {
    if ((now - g_nfcOpMessageMs) >= NFC_MESSAGE_DISPLAY_MS) {
      g_nfcOpState = NFC_OP_STATE_IDLE;
      g_nfcHash1 = g_nfcHash2 = 0;
      showLcdWaiting();
      return;
    }
  }

  if (g_lcdNfcMessageActive && (now - g_lcdNfcMessageMs) >= NFC_LCD_MESSAGE_HOLD_MS) {
    clearLcdLine1();
  }
}

// -------------------- Select actions --------------------
// shortSelect(): enter/advance within menus.
// longSelect(): save (for edit layers) or confirm destructive operations (remove-all/delete).
static void shortSelect() {
  if (g_removeAllPromptActive) {
    g_removeAllConfirm = !g_removeAllConfirm;
    return;
  }

  switch (g_currentLayer) {
    case LAYER_TOP:
  switch (g_topCursor) {
    case TOP_PCBHWID:     setLayer(LAYER_PCB); break;
    case TOP_DISPENSED:   setLayer(LAYER_DISPENSED_MENU); break;
    case TOP_KIOSKPARM:   setLayer(LAYER_KIOSKPARM_MENU); break;
    case TOP_HYDRPARM:    setLayer(LAYER_HYDRPARM_MENU); break;
    case TOP_NFC_MANAGE:  setLayer(LAYER_NFC_MENU); break;
    case TOP_REINIT:      setLayer(LAYER_REINIT); break;
    default:              setLayer(LAYER_TOP); break;
  }
  break;

case LAYER_KIOSKPARM_MENU:
  switch (g_kioskParmCursor) {
    case KPARM_MENU_SOLENOID: setLayer(LAYER_SOL_MENU); break;
    case KPARM_MENU_KLARANUV: setLayer(LAYER_KLARAN_MENU); break;
    case KPARM_MENU_WATERTEMP: setLayer(LAYER_WATERTEMP_EDIT); break;
    case KPARM_MENU_COINACC: setLayer(LAYER_COINACC_EDIT); break;
    case KPARM_MENU_RTC: setLayer(LAYER_RTC_CONFIG); break;
    case KPARM_MENU_NFC_PARMS: setLayer(LAYER_NFC_PARMS_MENU); break;
    default:                  setLayer(LAYER_KIOSKPARM_MENU); break;
  }
  break;

    case LAYER_HYDRPARM_MENU:
      switch (g_hydrParmCursor) {
        case HPARM_MENU_BACKWASH: setLayer(LAYER_BACKWASH_MENU); break;
        case HPARM_MENU_SENSORBYP: setLayer(LAYER_SENSORBYP_MENU); break;
        case HPARM_MENU_WATERDISP: setLayer(LAYER_WD_MENU); break;
        case HPARM_MENU_WATERCIRC: setLayer(LAYER_WCIRC_MENU); break;
        case HPARM_MENU_PFLUSH: setLayer(LAYER_PFLUSH_MENU); break;
        default: setLayer(LAYER_HYDRPARM_MENU); break;
      }
      break;


    case LAYER_WD_MENU:
      // Enter edit mode for selected WD field.
      g_wdEditField = g_wdMenuCursor;
      setLayer(LAYER_WD_EDIT);
      break;

    case LAYER_WD_EDIT:
      // Cycle edit field.
      g_wdEditField = (uint8_t)((g_wdEditField + 1) % WD_FIELD_COUNT);
      break;

    case LAYER_SOL_MENU:
      g_currentSolenoidIndex = g_solMenuCursor;
      setLayer(LAYER_SOL_PARAM_EDIT);
      break;

    case LAYER_SOL_PARAM_EDIT:
      g_solParamCursor = (uint8_t)((g_solParamCursor + 1) % SOL_PARAM_COUNT);
      break;

    case LAYER_DISPENSED_MENU:
      if (g_dispMenuCursor < 4) {
        g_currentDispenseIndex = g_dispMenuCursor;
        setLayer(LAYER_DISPENSED_EDIT);
      }
      break;

    case LAYER_BACKWASH_MENU:
      g_currentBwIndex = g_bwMenuCursor;
      setLayer(LAYER_BACKWASH_EDIT);
      break;

    case LAYER_KLARAN_MENU:
      g_currentUvIndex = g_uvMenuCursor;
      setLayer(LAYER_KLARAN_EDIT);
      break;

    case LAYER_SENSORBYP_MENU:
      g_currentSensorBypIndex = g_sensorBypMenuCursor;
      setLayer(LAYER_SENSORBYP_EDIT);
      break;

    case LAYER_SENSORBYP_EDIT:
      g_currentSensorBypIndex = (uint8_t)((g_currentSensorBypIndex + 1U) % SBYP_MENU_COUNT);
      break;

    case LAYER_WCIRC_MENU:
      g_currentWcircIndex = g_wcircMenuCursor;
      setLayer(LAYER_WCIRC_EDIT);
      break;

    case LAYER_WCIRC_EDIT:
      g_currentWcircIndex = (uint8_t)((g_currentWcircIndex + 1U) % WCIRC_MENU_COUNT);
      break;

    case LAYER_PFLUSH_MENU:
      g_currentPflushIndex = g_pflushMenuCursor;
      setLayer(LAYER_PFLUSH_EDIT);
      break;

    case LAYER_PFLUSH_EDIT:
      g_currentPflushIndex = (uint8_t)((g_currentPflushIndex + 1U) % PFLUSH_MENU_COUNT);
      break;

    case LAYER_NFC_PARMS_MENU:
      g_currentNfcParmIndex = g_nfcParmsMenuCursor;
      setLayer(LAYER_NFC_PARMS_EDIT);
      break;

    case LAYER_WATERTEMP_EDIT:
      // Short select toggles between T1 and T2.
      g_waterTempSenseCache = (g_waterTempSenseCache == 2) ? 1 : 2;
      g_waterTempSenseDirty = true;
      break;

    case LAYER_COINACC_EDIT:
      g_coinAccFittedCache = !g_coinAccFittedCache;
      g_coinAccFittedDirty = true;
      break;

    case LAYER_RTC_CONFIG: {
      if (!g_rtcPresent) break;
      if (g_rtcClusterIndex < 5) {
        g_rtcClusterIndex++;
      } else {
        g_rtcClusterIndex = 0;
      }
      break;
    }

    case LAYER_NFC_MENU:
      if (g_nfcMenuCursor == NFC_MENU_BROWSE) setLayer(LAYER_NFC_BROWSE);
      else if (g_nfcMenuCursor == NFC_MENU_ADD) setLayer(LAYER_NFC_ADD);
      else if (g_nfcMenuCursor == NFC_MENU_DEL) setLayer(LAYER_NFC_DEL);
      else if (g_nfcMenuCursor == NFC_MENU_PACK) {
        if (!G->allowTokenWrites) break;
        startNfcTablePackNow();
      }
      else if (g_nfcMenuCursor == NFC_MENU_REMOVE_ALL) {
        if (!G->allowTokenWrites) break;
        g_removeAllPromptActive = true;
        g_removeAllConfirm = false;
      }
      break;

    case LAYER_NFC_BROWSE:
      // Short select toggles delete prompt.
      if (g_tokenState == TOKEN_STATE_BROWSE) {
        g_tokenState = TOKEN_STATE_DELETE_PROMPT;
        g_tokenDeleteConfirm = false;
      } else {
        g_tokenState = TOKEN_STATE_BROWSE;
        g_tokenDeleteConfirm = false;
      }
      break;

    case LAYER_REINIT:
      // Enter prompt mode (user must still confirm YES and hold >= 10s).
      g_reinitPromptActive = true;
      g_reinitConfirmYes = false;
      g_reinitHoldActive = false;
      break;

    default:
      break;
  }
}

static void longSelect() {
  if (g_removeAllPromptActive) {
    if (G->allowTokenWrites && g_removeAllConfirm) {
      beginSaving();
      G->ee->removeAllTokens();
      finishSaving();
    }
    g_removeAllPromptActive = false;
    g_removeAllConfirm = false;
    return;
  }

  switch (g_currentLayer) {
    case LAYER_WD_EDIT:
    case LAYER_SOL_PARAM_EDIT:
    case LAYER_DISPENSED_EDIT:
    case LAYER_BACKWASH_EDIT:
    case LAYER_KLARAN_EDIT:
    case LAYER_SENSORBYP_EDIT:
    case LAYER_WCIRC_EDIT:
    case LAYER_PFLUSH_EDIT:
    case LAYER_WATERTEMP_EDIT:
    case LAYER_COINACC_EDIT:
    case LAYER_NFC_PARMS_EDIT:
    case LAYER_RTC_CONFIG:
    case LAYER_PCB:
      // Commit edits.
      (void)saveIfDirtyForLayer();
      break;

    case LAYER_NFC_BROWSE:
      if (g_tokenState == TOKEN_STATE_DELETE_PROMPT) {
        if (G->allowTokenWrites && g_tokenDeleteConfirm) {
          uint32_t h = G->ee->tokenHashAt(g_tokenBrowseIndex);
          if (h != KioskEeprom::EEPROM_TOKEN_EMPTY) {
            beginSaving();
            G->ee->deleteTokenHash(h);
            finishSaving();
          }
        }
        g_tokenState = TOKEN_STATE_BROWSE;
        g_tokenDeleteConfirm = false;
      }
      break;

    case LAYER_REINIT:
      // Intentionally no action: reset begins only when SELECT is held >= 10s after YES confirm.
      break;

    default:
      break;
  }
}

// -------------------- Main entrypoint --------------------
// run(): blocking call; never returns.
[[noreturn]] void run(const Config& cfg) {
  EditorState st;
  st.cfg = &cfg;
  S = &st;

  // Minimal sanity checks: this module is display-driven.
  if (!G->ee || !G->oled || !G->lcd) resetNow();

  // Ensure EEPROM module is ready.
  if (!G->ee->isReady()) {
    if (!G->ee->begin()) resetNow();
  }

  // Optional: initialise only our input pins (useful for standalone test harness).
  if (G->initPins) {
    pinMode(G->pinUp, INPUT_PULLUP);
    pinMode(G->pinDown, INPUT_PULLUP);
    pinMode(G->pinSelect, INPUT_PULLUP);
    pinMode(G->pinBack, INPUT_PULLUP);
    if (G->pinReinitGuard != 255) pinMode(G->pinReinitGuard, INPUT_PULLUP);
  }

  // Menu navigation direction:
  // - UP moves cursor “up” (previous) => -1
  // - DOWN moves cursor “down” (next) => +1
  g_btnUp     = { G->pinUp,   HIGH, false, 0, 0, -1 };
  g_btnDown   = { G->pinDown, HIGH, false, 0, 0, +1 };
  g_btnSelect = { G->pinSelect, HIGH, false, 0 };
  g_btnBack   = { G->pinBack,   HIGH, false, 0 };

  // Keep PN532 in reset unless scanning.
  if (G->nfc && G->pinPn532Reset != 255) {
    pinMode(G->pinPn532Reset, OUTPUT);
    digitalWrite(G->pinPn532Reset, LOW);
  }

  // Ensure EEPROM has valid “magic” or initialise to defaults.
  G->ee->ensureMagicOrInitDefaults();
  g_workingPcbId = G->ee->hwId();

  if (G->startInPcbHwid) {
    g_currentLayer = LAYER_PCB;
    g_topCursor = TOP_PCBHWID;
  }

  // Initialise base LCD layout for editor mode.
  renderLcdBase();
  clearLcdLine1();
  g_lcdStatusActive = false;

  // Clear OLED cache so first render draws all rows.
  for (int r = 0; r < 16; ++r) memset(g_oledRowCache[r], 0, 17);

  // Enter initial layer.
  onEnterLayer(g_currentLayer);

  // Main loop: event polling + render.
  while (1) {
    unsigned long now = millis();

    // After a reset operation: wait for SELECT release OR >= 3s, then restart UI.
    if (g_reinitPostWaitActive) {
      unsigned long t2 = millis();
      bool selPressed = readPressed(G->pinSelect);

      if (!selPressed || (t2 - g_reinitPostWaitStartMs >= REINIT_COMPLETE_WAIT_MS)) {
        g_reinitPostWaitActive = false;
        g_lcdStatusActive = false;
        restartEditorUiAfterReset();
      }

      renderOled();
      delay(5);
      continue;
    }

    // Re-init screen hold-to-start logic:
    // only active when prompt is active AND confirm is YES.
    if (g_currentLayer == LAYER_REINIT && g_reinitPromptActive && g_reinitConfirmYes) {
      bool selPressed = readPressed(G->pinSelect);
      unsigned long t2 = millis();

      if (selPressed) {
        if (!g_reinitHoldActive) {
          g_reinitHoldActive = true;
          g_reinitHoldStartMs = t2;
        } else if ((t2 - g_reinitHoldStartMs) >= REINIT_START_HOLD_MS) {
          g_reinitHoldActive = false;
          startEepromFullResetNow();
        }
      } else {
        g_reinitHoldActive = false;
      }
    } else {
      g_reinitHoldActive = false;
    }

    // NFC workflow
    performNfcAddDel(now);
    processNfcOpTimeout(now);

    // Buttons
    processRepeatButton(g_btnUp, now);
    processRepeatButton(g_btnDown, now);
    processSelectButton(now);
    processBackButton(now);

    // Emergency reset chord (SELECT + guard)
    processResetChord(now);

    if (g_currentLayer == LAYER_RTC_CONFIG && g_rtcPresent) {
      if ((now - g_rtcLastPollMs) >= 100) {
        g_rtcLastPollMs = now;
        uint16_t y = 2000;
        uint8_t mo = 1, d = 1, h = 0, mi = 0, s = 0;
        if (rtcReadDs3231(y, mo, d, h, mi, s)) {
          rtcClampFields(y, mo, d, h, mi, s);
          if (s != g_rtcLastSecond) {
            g_rtcLastSecond = s;
            rtcUpdateInfoLinesFromFields(y, mo, d, h, mi, s);
            if (g_rtcClusterIndex < 3) {
              rtcUpdateTimeDigitsFromFields(y, mo, d, h, mi, s);
            }
          }
        }
      }
    }

    // Manage Saved overlay timing (OLED + LCD).
    unsigned long t = millis();

    if (g_showSavedMessage && (t - g_savedStartMs >= SAVED_DISPLAY_MS)) {
      g_showSavedMessage = false;

      if (g_lcdStatusActive) {
        g_lcdStatusActive = false;
        if (g_currentLayer == LAYER_NFC_ADD || g_currentLayer == LAYER_NFC_DEL) showLcdWaiting();
        else clearLcdLine1();
      }
    }

    if (g_lcdStatusActive && (t - g_lcdStatusStartMs >= SAVED_DISPLAY_MS)) {
      g_lcdStatusActive = false;
      if (g_currentLayer == LAYER_NFC_ADD || g_currentLayer == LAYER_NFC_DEL) showLcdWaiting();
      else clearLcdLine1();
    }

    // Render the current UI.
    renderOled();

    // Small delay to keep loop stable and reduce CPU usage.
    delay(5);
  }
}

} // namespace KioskEepromEditor
} // namespace Kiosk
