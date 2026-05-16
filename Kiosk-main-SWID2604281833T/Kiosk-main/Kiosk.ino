// Kiosk.ino
//
// Main sketch: wires inputs/outputs, UI state machine, and hydraulic behavior.
// Most button actions are edge-triggered on HIGH->LOW transitions (active-low wiring);
// some controls intentionally use press/release duration semantics.
//
// Serial output is intentionally disabled in this build.
//
// Behavior notes:
// - Backwash/sensor bypass/circulation/inlet policy is centralized in KioskBehavior.
// - KioskHydraulics executes low-level dispense/backwash/inlet actuator timing.

#include <Arduino.h>
#include <Wire.h>  // must be included before hd44780_I2Cexp.h
#include <string.h>
#include <stdarg.h>
#include <U8x8lib.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_PN532.h>
#include <avr/wdt.h>
#include "KioskIOpins.h"
#include "KioskBuildInfo.h"
#include "KioskIO.h"
#include "KioskHydraulics.h"
#include "KioskBehavior.h"
#include "KioskEeprom.h"
#include "KioskSeqTest.h"

struct StatusSnap;

static char g_swidLine[17];

extern "C" void KioskFormatSwidText(char* out, size_t outLen)
{
  if (!out || outLen == 0) return;
  snprintf(out, outLen, "%s", KIOSK_SWID);
}

extern "C" void KioskFormatHwIdText(uint8_t hwid, char* out, size_t outLen)
{
  switch (hwid) {
    case 0: snprintf(out, outLen, "Update ID!"); break;
    case 1: snprintf(out, outLen, "v2.4 202508"); break;
    case 2: snprintf(out, outLen, "v2.5 202508"); break;
    case 3: snprintf(out, outLen, "v2.6 260205"); break;
    case 4: snprintf(out, outLen, "v2.7 260301"); break;
    default: snprintf(out, outLen, "UNDEFINED"); break;
  }
}

#if __has_include("KioskEepromEditor.h")
#include "KioskEepromEditor.h"
#define KIOSK_HAS_EEPROM_EDITOR 1
#else
#define KIOSK_HAS_EEPROM_EDITOR 0
#endif

static constexpr uint32_t DEBOUNCE_MS = 50;
static constexpr uint16_t PRESS_DURATION_MAX_MS = 65534U;
static constexpr uint16_t PRESS_DURATION_OVERFLOW = 65535U;

struct FallingEdgeBtn {
  uint8_t  pin;
  uint8_t  lastRaw;       // HIGH/LOW
  uint32_t lastChangeMs;
  bool   (*readRaw)();    // raw pin reader

  void begin() {
    lastRaw      = (uint8_t)(readRaw ? readRaw() : digitalRead(pin));
    lastChangeMs = millis();
  }

  // Returns true on HIGH->LOW transition (debounced)
  bool fell() {
    const uint8_t raw = (uint8_t)(readRaw ? readRaw() : digitalRead(pin));
    const uint32_t now = millis();

    if (raw != lastRaw && (uint32_t)(now - lastChangeMs) >= DEBOUNCE_MS) {
      lastChangeMs = now;
      const bool isFalling = (lastRaw == HIGH && raw == LOW);
      lastRaw = raw;
      return isFalling;
    }
    return false;
  }
};

struct DebouncedButtonTracker {
  uint8_t  pin;
  uint8_t  lastRaw;      // HIGH/LOW stable state
  uint32_t lastChangeMs;
  bool   (*readRaw)();   // raw pin reader

  bool pressedFlag;
  bool releasedFlag;
  bool down;
  unsigned long pressStartMs;
  uint16_t lastPressDurationMs;

  void begin() {
    lastRaw = (uint8_t)(readRaw ? readRaw() : digitalRead(pin));
    lastChangeMs = millis();
    pressedFlag = false;
    releasedFlag = false;
    down = (lastRaw == LOW);
    pressStartMs = 0UL;
    lastPressDurationMs = 0U;
  }

  void update(uint32_t now) {
    const uint8_t raw = (uint8_t)(readRaw ? readRaw() : digitalRead(pin));
    if (raw != lastRaw && (uint32_t)(now - lastChangeMs) >= DEBOUNCE_MS) {
      const bool wasDown = (lastRaw == LOW);
      lastRaw = raw;
      lastChangeMs = now;
      down = (raw == LOW);
      if (!wasDown && down) {
        pressedFlag = true;
        pressStartMs = (unsigned long)now;
      } else if (wasDown && !down) {
        releasedFlag = true;
        const uint32_t duration = (uint32_t)(now - (uint32_t)pressStartMs);
        lastPressDurationMs =
          (duration > (uint32_t)PRESS_DURATION_MAX_MS) ? PRESS_DURATION_OVERFLOW : (uint16_t)duration;
      }
    }
  }

  bool takePressed() {
    const bool v = pressedFlag;
    pressedFlag = false;
    return v;
  }

  bool takeReleased() {
    const bool v = releasedFlag;
    releasedFlag = false;
    return v;
  }

  bool isDown() const { return down; }
  uint16_t lastDurationMs() const { return lastPressDurationMs; }
};

// Button trackers used by the runtime loop:
// - FallingEdgeBtn: one-shot pulse on debounced HIGH->LOW.
// - DebouncedButtonTracker: press/release events + held duration.
// Backwash/sensor-bypass depend on release duration, so they use DebouncedButtonTracker.
static DebouncedButtonTracker btnBackwash { KioskPins::BTN_BACKWASH_CTL, 0, 0, KioskIO::btnBackwashCtl, false, false, false, 0UL, 0U };
static FallingEdgeBtn btnInlet       { KioskPins::BTN_WATER_INLET,  0, 0, KioskIO::btnWaterInlet     };
static FallingEdgeBtn btnDispense    { KioskPins::BTN_SINGLE_DISP,  0, 0, KioskIO::btnSingleDisp     };
static DebouncedButtonTracker btnSensorBypass { KioskPins::BTN_SENSOR_BYPASS, 0, 0, KioskIO::btnSensorBypass, false, false, false, 0UL, 0U };
static DebouncedButtonTracker btnFrontDisp { KioskPins::BTN_FRONT_DISPENSE, 0, 0, KioskIO::btnFrontDisp, false, false, false, 0UL, 0U };
static DebouncedButtonTracker btnContDisp  { KioskPins::BTN_CONT_DISP,    0, 0, KioskIO::btnContDisp,  false, false, false, 0UL, 0U };

// Displays (I2C): OLED + 16x4 LCD
U8X8_SH1107_SEEED_128X128_HW_I2C oled(U8X8_PIN_NONE);
hd44780_I2Cexp lcd;
// NFC reader (PN532) for EEPROM editor NFC add/del.
static Adafruit_PN532 g_editorNfc(KioskPins::PN532_IRQ, KioskPins::PN532_RESET);

// EEPROM access (shared with the editor UI).
static Kiosk::KioskEeprom g_eeprom;


static bool g_backlightOn = false;
static uint32_t g_lastLcdWriteMs = 0;
static bool g_lcdPending = false;
static bool g_lcdDispensingFrameWritten = false;
static char g_lcdPendingLine0[17] = {0};
static char g_lcdPendingLine1[17] = {0};
static char g_lcdPendingLine2[17] = {0};
static char g_lcdPendingLine3[17] = {0};

// UI state machine context + output buffer
static KioskBehavior::Ui::UiContext g_ui;
static KioskBehavior::Ui::UiOutputs g_uiOut;

// Snapshot used to detect status changes
struct StatusSnap {
  uint8_t wl;
  bool    bw;
  uint8_t hydSt;
  uint8_t dispCnt;
  uint8_t inletPh;
};

static bool      g_haveLast = false;
static StatusSnap g_last;
static bool g_prevSensorBypassOn = false;
static bool g_prevBackwashOn = false;
static bool g_prevWaterInOn = false;
static bool g_prevDispenseOn = false;
static uint32_t g_sensorBypassOnStartMs = 0;
static uint32_t g_backwashOnStartMs = 0;
static uint32_t g_waterInOnStartMs = 0;
static uint32_t g_dispenseOnStartMs = 0;
static uint32_t g_dispensePendingUntilMs = 0;
static bool g_prevContDispDown = false;
static uint32_t g_contDispPressStartMs = 0;
enum DayEndChordState : uint8_t {
  DAYEND_CHORD_IDLE = 0,
  DAYEND_CHORD_BW_HELD = 1,
  DAYEND_CHORD_BOTH_HELD = 2,
  DAYEND_CHORD_BW_RELEASED_WAIT_CIRC_RELEASE = 3
};
static DayEndChordState g_dayEndChordState = DAYEND_CHORD_IDLE;
static char g_oledRow2Cache[17] = {0};
static char g_oledRow3Cache[17] = {0};
static char g_oledRow4Cache[17] = {0};
static char g_oledRow5Cache[17] = {0};
static char g_oledRow7Cache[17] = {0};
static char g_oledRow8Cache[17] = {0};
static char g_oledRow9Cache[17] = {0};
static char g_oledRow10Cache[17] = {0};
static char g_oledRow11Cache[17] = {0};

static void oledClearRow(uint8_t row)
{
  // OLED uses a 16-character row with the 8x8 font.
  oled.drawString(0, row, "                ");
}
static const char* waterLevelLabel(uint8_t wl)
{
  switch (wl) {
    case 0: return " EMPTY";
    case 1: return " LOW ";
    case 2: return " MID ";
    case 3: return " FULL";
    default: return " ????";
  }
}

static void pad16(char* s)
{
  // Make the string exactly 16 chars for the LCD (space padded, NUL‑terminated).
  size_t n = strlen(s);
  if (n > 16) {
    s[16] = 0;
    return;
  }
  while (n < 16) s[n++] = ' ';
  s[16] = 0;
}

// Strict formatter for Kiosk Operation rows: any >16-char result is replaced
// with a visible marker instead of silent truncation.
static void formatStatusRow16(char* out, size_t outSize, const char* tag, const char* fmt, ...)
{
  if (!out || outSize == 0 || !fmt) return;
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(out, outSize, fmt, ap);
  va_end(ap);
  if (n < 0 || n > 16) {
    snprintf(out, outSize, "%s FMT OVR", (tag && tag[0]) ? tag : "ROW");
  }
  pad16(out);
}

// Format a countdown as seconds with one decimal place (ss.s style), rounded up
// to the next 100ms so visible countdowns do not stall on boundaries.
static void formatSecondsTenths(uint32_t remainMs, char* out, size_t outSize)
{
  if (!out || outSize == 0) return;
  const uint32_t deci = (remainMs + 99UL) / 100UL;
  uint32_t secWhole = deci / 10UL;
  uint32_t secFrac = deci % 10UL;
  if (secWhole > 99UL) {
    secWhole = 99UL;
    secFrac = 9UL;
  }
  snprintf(out, outSize, "%lu.%lu", (unsigned long)secWhole, (unsigned long)secFrac);
}

// Dispense countdown format:
// - >= 100.0s: integer seconds (nnnn)
// - < 100.0s : one decimal second (nn.n)
static void formatDispenseCountdown(uint32_t remainMs, char* out, size_t outSize)
{
  if (!out || outSize == 0) return;
  if (remainMs >= 100000UL) {
    uint32_t sec = (remainMs + 999UL) / 1000UL;
    if (sec > 9999UL) sec = 9999UL;
    snprintf(out, outSize, "%lu", (unsigned long)sec);
    return;
  }
  formatSecondsTenths(remainMs, out, outSize);
}

static void lcdPrintLinePadded(uint8_t row, const char* text)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%s", text);
  pad16(buf);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

static void oledClearRows(uint8_t startRow, uint8_t count)
{
  for (uint8_t i = 0; i < count; ++i) {
    oledClearRow((uint8_t)(startRow + i));
  }
}

static void oledDrawCenteredRow(uint8_t row, const char* text)
{
  oledClearRow(row);
  if (!text) return;
  const size_t len = strlen(text);
  const uint8_t start = (len >= 16) ? 0 : (uint8_t)((16 - len) / 2);
  oled.drawString(start, row, text);
}

static void oledDrawCenteredBlock(uint8_t startRow,
                                  uint8_t endRow,
                                  const char* const* lines,
                                  uint8_t lineCount)
{
  if (endRow < startRow) return;
  for (uint8_t r = startRow; r <= endRow; ++r) {
    oledClearRow(r);
  }
  if (!lines || lineCount == 0) return;
  const uint8_t regionRows = (uint8_t)(endRow - startRow + 1);
  const uint8_t firstRow =
    (lineCount >= regionRows) ? startRow : (uint8_t)(startRow + (regionRows - lineCount) / 2);
  for (uint8_t i = 0; i < lineCount && (uint8_t)(firstRow + i) <= endRow; ++i) {
    oledDrawCenteredRow((uint8_t)(firstRow + i), lines[i]);
  }
}

static void formatWaterLine(const StatusSnap& s, char* buf, size_t bufSize)
{
  // "WaterLvl <n><label>" where label is one of: " EMPTY", " LOW ", " MID ", " FULL".
  formatStatusRow16(buf, bufSize, "R2", "WaterLvl %u%s", (unsigned)s.wl, waterLevelLabel(s.wl));
}

static void formatBackwashLine(const StatusSnap& s, char* buf, size_t bufSize)
{
  // Backwash eligibility depends on mode:
  // - Normal: water level > 1
  // - Override: water level > 0
  const bool overrideActive = KioskIO::overrideActive();
  const bool bwOk = overrideActive ? (s.wl > 0) : (s.wl > 1);
  formatStatusRow16(buf, bufSize, "R3", "%s", bwOk ? "BackWash is OK  " : "Backwash not OK ");
}

static void formatDispenseLine(const StatusSnap& s, char* buf, size_t bufSize)
{
  // "Dispense is OK  " / "Dispense not OK ".
  const bool dispOk = (s.wl > 0) && (s.hydSt == 0);
  formatStatusRow16(buf, bufSize, "R4", "%s", dispOk ? "Dispense is OK  " : "Dispense not OK ");
}

static bool dispenseOkNow()
{
  return (KioskIO::tankWaterLevel() > 0)
      && (KioskHydraulics::state() == 0);
}

static uint32_t dispenseDurationMs()
{
  if (g_eeprom.isReady()) {
    const uint32_t d = g_eeprom.dispMeasuredDurationMs();
    if (d > 0) return d;
  }
  return 5000UL;
}

static void lcdPrintStatus(const StatusSnap& s)
{
  char buf[17];

  formatWaterLine(s, buf, sizeof(buf));
  lcd.setCursor(0, 1);
  lcd.print(buf);

  formatBackwashLine(s, buf, sizeof(buf));
  lcd.setCursor(0, 2);
  lcd.print(buf);

  formatDispenseLine(s, buf, sizeof(buf));
  lcd.setCursor(0, 3);
  lcd.print(buf);
}

static void lcdInitProgressChars()
{
  uint8_t blocks[5][8];
  for (uint8_t i = 0; i < 5; ++i) {
    const uint8_t cols = (uint8_t)(i + 1);
    const uint8_t mask = (uint8_t)(((1U << cols) - 1U) << (5 - cols)); // left columns
    for (uint8_t r = 0; r < 8; ++r) blocks[i][r] = mask;
    lcd.createChar((uint8_t)(i + 1), blocks[i]); // slots 1..5
  }
}

static void applyDispenseLed(KioskBehavior::Ui::LedMode mode)
{
  switch (mode) {
    case KioskBehavior::Ui::LED_ON:
      KioskIO::setBtnFrontDispLEDMode(KioskIO::LED_ON);
      break;
    case KioskBehavior::Ui::LED_PULSE:
      KioskIO::setBtnFrontDispLEDMode(KioskIO::LED_BEACON);
      break;
    case KioskBehavior::Ui::LED_OFF:
    default:
      KioskIO::setBtnFrontDispLEDMode(KioskIO::LED_OFF);
      break;
  }
}

static void applyBacklight(KioskBehavior::Ui::BacklightMode mode, uint32_t stateAgeMs)
{
  // UI intro flicker is implemented as an explicit one-shot pulse to avoid loop-time
  // quantization stretching a nominal 50ms blip into ~100ms.
  if (mode == KioskBehavior::Ui::BL_FLICK_ON_INTRO && stateAgeMs < 5U) {
    lcd.noBacklight();
    delay(50);
    lcd.backlight();
    g_backlightOn = true;
    return;
  }

  bool wantOn = false;
  switch (mode) {
    case KioskBehavior::Ui::BL_ON: wantOn = true; break;
    case KioskBehavior::Ui::BL_FLICK_ON_INTRO:
      // After the one-shot pulse at state entry, stay ON.
      wantOn = true;
      break;
    case KioskBehavior::Ui::BL_OFF:
    default:
      wantOn = false;
      break;
  }
  if (wantOn != g_backlightOn) {
    if (wantOn) {
      lcd.backlight();
    } else {
      lcd.noBacklight();
    }
    g_backlightOn = wantOn;
  }
}

static void oledPrintStatus(const StatusSnap& s, uint32_t now)
{
  char buf[17];

  // Row 2: Water level + label
  formatWaterLine(s, buf, sizeof(buf));
  if (strcmp(buf, g_oledRow2Cache) != 0) {
    oled.drawString(0, 2, buf);
    strncpy(g_oledRow2Cache, buf, sizeof(g_oledRow2Cache));
    g_oledRow2Cache[16] = 0;
  }

  // Row 3: Backwash OK / Not OK
  formatBackwashLine(s, buf, sizeof(buf));
  if (strcmp(buf, g_oledRow3Cache) != 0) {
    oled.drawString(0, 3, buf);
    strncpy(g_oledRow3Cache, buf, sizeof(g_oledRow3Cache));
    g_oledRow3Cache[16] = 0;
  }

  // Row 4: Dispense OK / Not OK
  formatDispenseLine(s, buf, sizeof(buf));
  if (strcmp(buf, g_oledRow4Cache) != 0) {
    oled.drawString(0, 4, buf);
    strncpy(g_oledRow4Cache, buf, sizeof(g_oledRow4Cache));
    g_oledRow4Cache[16] = 0;
  }

  // Row 5: inlet PWM phase (kept from earlier demo)
  const char* ph = "PH0";
  if (s.inletPh == 1) ph = "PH1";
  else if (s.inletPh == 2) ph = "PH2";
  formatStatusRow16(buf, sizeof(buf), "R5", "InletSol PWM %s", ph);
  if (strcmp(buf, g_oledRow5Cache) != 0) {
    oled.drawString(0, 5, buf);
    strncpy(g_oledRow5Cache, buf, sizeof(g_oledRow5Cache));
    g_oledRow5Cache[16] = 0;
  }

  // Row 8: SensorBypass OFF/AUT/ON.
  // - ON countdown:
  //   >=100.0s -> nnnn seconds
  //   <100.0s  -> nn.n seconds
  // - AUT: periodic timed event with countdown to next event
  const bool sensorOn = KioskBehavior::sensorBypassActive();
  const bool sensorPeriodic = KioskBehavior::sensorBypassPeriodicActive();
  if (sensorOn) {
    const uint32_t remain = KioskBehavior::sensorBypassRemainingMs(now);
    char t[8];
    formatDispenseCountdown(remain, t, sizeof(t));
    formatStatusRow16(buf, sizeof(buf), "R8", "SenseByp ON %4s", t);
  } else if (sensorPeriodic) {
      const uint32_t remainMs = KioskBehavior::sensorBypassPeriodicRemainingMs(now);
      const uint32_t sec = (remainMs + 999UL) / 1000UL;
      const uint32_t mins = (sec + 59UL) / 60UL;
      const uint32_t val = (sec > 9999UL) ? mins : sec;
      formatStatusRow16(buf, sizeof(buf), "R8", "SenseByp AUT%4lu", (unsigned long)val);
  } else {
    formatStatusRow16(buf, sizeof(buf), "R8", "SenseByp OFF");
  }
  if (strcmp(buf, g_oledRow8Cache) != 0) {
    oled.drawString(0, 8, buf);
    strncpy(g_oledRow8Cache, buf, sizeof(g_oledRow8Cache));
    g_oledRow8Cache[16] = 0;
  }

  // Row 7: Backwash OFF/AUT/ON with countdown while ON.
  // - ON: seconds by default; minutes only once remaining duration gets large
  //   (per current threshold rule)
  // Values are right-justified without leading zeros.
  const bool dayEndModeActive = KioskBehavior::dayEndModeActive();
  const bool backwashEod = KioskBehavior::dayEndBackwashActive();
  const bool backwashEodHld = KioskBehavior::dayEndBackwashWaiting();
  const bool backwashOn = s.bw;
  const bool backwashPend = KioskBehavior::backwashPending();
  if (dayEndModeActive) {
    if (backwashEod) {
      if (backwashEodHld) {
        formatStatusRow16(buf, sizeof(buf), "R7", "Backwash EoD HLD");
      } else {
        const uint32_t remain = KioskBehavior::dayEndBackwashRemainingMs(now);
        const uint32_t sec = (remain + 999UL) / 1000UL;
        const uint32_t mins = (sec + 59UL) / 60UL;
        const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
        formatStatusRow16(buf, sizeof(buf), "R7", "Backwash EoD%4lu", (unsigned long)val);
      }
    } else {
      formatStatusRow16(buf, sizeof(buf), "R7", "Backwash OFF");
    }
  } else if (backwashEod) {
    if (backwashEodHld) {
      formatStatusRow16(buf, sizeof(buf), "R7", "Backwash EoD HLD");
    } else {
      const uint32_t remain = KioskBehavior::dayEndBackwashRemainingMs(now);
      const uint32_t sec = (remain + 999UL) / 1000UL;
      const uint32_t mins = (sec + 59UL) / 60UL;
      const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
      formatStatusRow16(buf, sizeof(buf), "R7", "Backwash EoD%4lu", (unsigned long)val);
    }
  } else if (backwashOn) {
    const uint32_t remain = KioskHydraulics::backwashRemainingMs(now);
    const uint32_t sec = (remain + 999UL) / 1000UL;
    const uint32_t mins = (sec + 59UL) / 60UL;
    const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
    formatStatusRow16(buf, sizeof(buf), "R7", "Backwash ON %4lu", (unsigned long)val);
  } else if (backwashPend) {
    formatStatusRow16(buf, sizeof(buf), "R7", "Backwash AUT");
  } else {
    if (g_eeprom.isReady()) {
      const uint8_t autoAfter = g_eeprom.backwashAfterNDispenses();
      if (autoAfter > 0U) {
        const uint16_t bwCount = g_eeprom.backwashDispenseCounter();
        uint32_t remain = (bwCount >= (uint32_t)autoAfter) ? 0UL : ((uint32_t)autoAfter - bwCount);
        if (remain > 999UL) remain = 999UL;
        formatStatusRow16(buf, sizeof(buf), "R7", "Backwash AUT%4lu", (unsigned long)remain);
      } else {
        formatStatusRow16(buf, sizeof(buf), "R7", "Backwash OFF");
      }
    } else {
      formatStatusRow16(buf, sizeof(buf), "R7", "Backwash OFF");
    }
  }
  if (strcmp(buf, g_oledRow7Cache) != 0) {
    oled.drawString(0, 7, buf);
    strncpy(g_oledRow7Cache, buf, sizeof(g_oledRow7Cache));
    g_oledRow7Cache[16] = 0;
  }

  // Row 9: WaterInp OFF/AUT/ON.
  // - ON/AUT: seconds by default; minutes only once elapsed value gets large
  //   (per current threshold rule)
  const bool waterInOn = (KioskIO::waterInletPhase() != 0);
  const bool waterInHold = KioskBehavior::waterInletHoldActive();
  if (waterInOn) {
    const uint32_t elapsedSec = (uint32_t)(now - g_waterInOnStartMs + 999UL) / 1000UL;
    const uint32_t mins = (elapsedSec + 59UL) / 60UL;
    const uint32_t val = (elapsedSec > 999UL && mins >= 16UL) ? mins : elapsedSec;
    formatStatusRow16(buf, sizeof(buf), "R9", "WaterInp ON %4lu", (unsigned long)val);
  } else if (waterInHold) {
    const uint32_t holdMs = KioskBehavior::waterInletHoldElapsedMs(now);
    const uint32_t sec = (holdMs + 999UL) / 1000UL;
    const uint32_t mins = (sec + 59UL) / 60UL;
    const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
    formatStatusRow16(buf, sizeof(buf), "R9", "WaterInp AUT%4lu", (unsigned long)val);
  } else {
    formatStatusRow16(buf, sizeof(buf), "R9", "WaterInp OFF");
  }
  if (strcmp(buf, g_oledRow9Cache) != 0) {
    oled.drawString(0, 9, buf);
    strncpy(g_oledRow9Cache, buf, sizeof(g_oledRow9Cache));
    g_oledRow9Cache[16] = 0;
  }

  // Row 10: TankCirc OFF/ON/AUT.
  // - ON/AUT: seconds by default; minutes only once remaining value gets large
  //   (per current threshold rule)
  const bool circEod = KioskBehavior::dayEndRecircActive();
  const bool circEodHld = KioskBehavior::dayEndRecircWaiting();
  const KioskBehavior::WaterCircStatus circSt = KioskBehavior::waterCircStatus();
  const uint32_t circRemainMs = circEod
      ? KioskBehavior::dayEndRecircRemainingMs(now)
      : KioskBehavior::waterCircDisplayRemainingMs(now);
  if (dayEndModeActive) {
    if (circEod) {
      if (circEodHld) {
        formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc EoD HLD");
      } else {
        const uint32_t sec = (circRemainMs + 999UL) / 1000UL;
        const uint32_t mins = (sec + 59UL) / 60UL;
        const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
        formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc EoD%4lu", (unsigned long)val);
      }
    } else {
      formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc OFF");
    }
  } else if (circEod) {
    if (circEodHld) {
      formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc EoD HLD");
    } else {
      const uint32_t sec = (circRemainMs + 999UL) / 1000UL;
      const uint32_t mins = (sec + 59UL) / 60UL;
      const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
      formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc EoD%4lu", (unsigned long)val);
    }
  } else if (circSt == KioskBehavior::WaterCircOn) {
    const uint32_t sec = (circRemainMs + 999UL) / 1000UL;
    const uint32_t mins = (sec + 59UL) / 60UL;
    const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
    formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc ON %4lu", (unsigned long)val);
  } else if (circSt == KioskBehavior::WaterCircAuto) {
    const uint32_t sec = (circRemainMs + 999UL) / 1000UL;
    const uint32_t mins = (sec + 59UL) / 60UL;
    const uint32_t val = (sec > 999UL && mins >= 16UL) ? mins : sec;
    formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc AUT%4lu", (unsigned long)val);
  } else {
    formatStatusRow16(buf, sizeof(buf), "R10", "TankCirc OFF");
  }
  if (strcmp(buf, g_oledRow10Cache) != 0) {
    oled.drawString(0, 10, buf);
    strncpy(g_oledRow10Cache, buf, sizeof(g_oledRow10Cache));
    g_oledRow10Cache[16] = 0;
  }

  // Row 11: Dispense OFF/AUT/ON with countdown while ON.
  // - >=100.0s: nnnn seconds
  // - <100.0s : nn.n seconds
  const bool dispOn = (s.hydSt != 0);
  const bool dispPend = (!dispOn && ((int32_t)(g_dispensePendingUntilMs - now) > 0));
  const bool dnfPending = KioskBehavior::periodicNozzleFlushPending();
  const bool dnfDispensing = KioskBehavior::periodicNozzleFlushDispensing();
  if (dnfDispensing) {
    const uint32_t remainMs = KioskBehavior::periodicNozzleFlushDispenseRemainingMs(now);
    char t[8];
    formatDispenseCountdown(remainMs, t, sizeof(t));
    formatStatusRow16(buf, sizeof(buf), "R11", "Dispense ON %4s", t);
  } else if (dnfPending) {
    const uint32_t remainMs = KioskBehavior::periodicNozzleFlushPendingRemainingMs(now);
    const uint32_t remainSec = (remainMs + 999UL) / 1000UL;
    formatStatusRow16(buf, sizeof(buf), "R11", "Dispense DNF%4lu", (unsigned long)remainSec);
  } else if (dispOn) {
    const uint32_t durMs = dispenseDurationMs();
    const uint32_t elapsed = (uint32_t)(now - g_dispenseOnStartMs);
    const uint32_t remain = (durMs > elapsed) ? (durMs - elapsed) : 0UL;
    char t[8];
    formatDispenseCountdown(remain, t, sizeof(t));
    formatStatusRow16(buf, sizeof(buf), "R11", "Dispense ON %4s", t);
  } else if (dispPend) {
    formatStatusRow16(buf, sizeof(buf), "R11", "Dispense AUT");
  } else {
    formatStatusRow16(buf, sizeof(buf), "R11", "Dispense OFF");
  }
  if (strcmp(buf, g_oledRow11Cache) != 0) {
    oled.drawString(0, 11, buf);
    strncpy(g_oledRow11Cache, buf, sizeof(g_oledRow11Cache));
    g_oledRow11Cache[16] = 0;
  }
}

static void oledPrintUiState(uint32_t now)
{
  static KioskBehavior::Ui::UiState s_lastUi =
    (KioskBehavior::Ui::UiState)255;
  static int32_t s_lastRemain = -2;
  static int8_t s_lastOverride = -1;
  static int8_t s_lastDayEnd = -1;
  static char s_lastRow15[17] = {0};

  int32_t remainSec = -1;
  if (g_ui.state == KioskBehavior::Ui::UI_DISP_RDY ||
      g_ui.state == KioskBehavior::Ui::UI_BOOT) {
    const uint32_t elapsed = (uint32_t)(now - g_ui.stateStartMs);
    remainSec = (elapsed >= 60000UL) ? 0 : (int32_t)((60000UL - elapsed + 999UL) / 1000UL);
  } else if (g_ui.state == KioskBehavior::Ui::UI_DISPENSING) {
    const uint32_t elapsed = (uint32_t)(now - g_ui.dispenseStartMs);
    const uint32_t duration = dispenseDurationMs();
    remainSec = (elapsed >= duration) ? 0 : (int32_t)((duration - elapsed + 999UL) / 1000UL);
  } else if (g_ui.state == KioskBehavior::Ui::UI_PAY_PROMPT) {
    const uint32_t base = g_ui.paymentStartMs ? g_ui.paymentStartMs : g_ui.stateStartMs;
    const uint32_t elapsed = (uint32_t)(now - base);
    remainSec = (elapsed >= 15000UL) ? 0 : (int32_t)((15000UL - elapsed + 999UL) / 1000UL);
  } else if (g_ui.state == KioskBehavior::Ui::UI_PAY_FAILED ||
             g_ui.state == KioskBehavior::Ui::UI_PAY_TOKEN_BAD ||
             g_ui.state == KioskBehavior::Ui::UI_DISP_DONE) {
    const uint32_t elapsed = (uint32_t)(now - g_ui.stateStartMs);
    remainSec = (elapsed >= 15000UL) ? 0 : (int32_t)((15000UL - elapsed + 999UL) / 1000UL);
  } else if (g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_IN) {
    const uint32_t elapsed = (uint32_t)(now - g_ui.stateStartMs);
    const uint32_t longDelayMs = 60000UL;
    const uint32_t shortDelayMs = 2000UL;
    if (elapsed >= (longDelayMs + shortDelayMs)) {
      // Distinct marker for "actually dark" phase to force OLED row-14 refresh.
      remainSec = -2;
    } else if (elapsed >= longDelayMs) {
      remainSec = 0;
    } else {
      remainSec = (int32_t)((longDelayMs - elapsed + 999UL) / 1000UL);
    }
  } else if (g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_OUT) {
    const uint32_t elapsed = (uint32_t)(now - g_ui.stateStartMs);
    remainSec = (elapsed >= 15000UL) ? 0 : (int32_t)((15000UL - elapsed + 999UL) / 1000UL);
  } else if (g_ui.state == KioskBehavior::Ui::UI_TIMEOUT) {
    const uint32_t elapsed = (uint32_t)(now - g_ui.stateStartMs);
    remainSec = (elapsed >= 3000UL) ? 0 : (int32_t)((3000UL - elapsed + 999UL) / 1000UL);
  }

  const bool overrideActive = KioskIO::overrideActive();
  const int8_t overrideFlag = overrideActive ? 1 : 0;
  const bool dayEndActive = KioskBehavior::dayEndModeActive();
  const int8_t dayEndFlag = dayEndActive ? 1 : 0;

  // Row 15: left-justified mode + right-justified YYMMDDHHMM from RTC.
  char row15[17];
  for (size_t i = 0; i < 16; ++i) row15[i] = ' ';
  row15[16] = 0;
  const char* mode4 = overrideActive ? "OVRD" : "NORM";
  for (size_t i = 0; i < 4; ++i) row15[i] = mode4[i];

  char dt[11];
  KioskIO::RtcTime rtcNow{};
  if (KioskIO::rtcRead(rtcNow)) {
    snprintf(dt, sizeof(dt), "%02u%02u%02u%02u%02u",
             (unsigned)(rtcNow.year % 100U),
             (unsigned)rtcNow.month,
             (unsigned)rtcNow.day,
             (unsigned)rtcNow.hour,
             (unsigned)rtcNow.minute);
  } else {
    snprintf(dt, sizeof(dt), "??????????");
  }
  for (size_t i = 0; i < 10; ++i) row15[6 + i] = dt[i];
  if (strcmp(row15, s_lastRow15) != 0) {
    oled.drawString(0, 15, row15);
    strncpy(s_lastRow15, row15, sizeof(s_lastRow15));
    s_lastRow15[16] = 0;
  }

  if (g_ui.state == s_lastUi && remainSec == s_lastRemain &&
      overrideFlag == s_lastOverride && dayEndFlag == s_lastDayEnd) return;
  s_lastUi = g_ui.state;
  s_lastRemain = remainSec;
  s_lastOverride = overrideFlag;
  s_lastDayEnd = dayEndFlag;

  char buf[17];
  for (size_t i = 0; i < 16; ++i) buf[i] = ' ';
  buf[16] = 0;
  oledClearRow(13);

  if (dayEndActive) {
    oledDrawCenteredRow(13, "DayEnd Circ+BW");
    oledClearRow(14);
    return;
  }

  if (g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_IN) {
    const uint32_t ageMs = (uint32_t)(now - g_ui.stateStartMs);
    const uint32_t longDelayMs = 60000UL;
    const uint32_t shortDelayMs = 2000UL;
    if (ageMs >= (longDelayMs + shortDelayMs)) {
      // Screen is in actual dark phase.
      oledDrawCenteredRow(14, "IN DARK MODE");
    } else {
      const char* uiId = KioskBehavior::Ui::uiStateId(g_ui.state);
      for (size_t i = 0; i < 16 && uiId[i] != 0; ++i) {
        buf[i] = uiId[i];
      }

      if (ageMs < longDelayMs && remainSec >= 0) {
        char tbuf[7];
        snprintf(tbuf, sizeof(tbuf), "%ld", (long)remainSec);
        const size_t len = strlen(tbuf);
        const size_t start = (len >= 16) ? 0 : (16 - len);
        for (size_t i = 0; i < len && (start + i) < 16; ++i) {
          buf[start + i] = tbuf[i];
        }
      }
      oled.drawString(0, 14, buf);
    }
  } else if (g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_OUT) {
    const char* uiId = KioskBehavior::Ui::uiStateId(g_ui.state);
    for (size_t i = 0; i < 16 && uiId[i] != 0; ++i) {
      buf[i] = uiId[i];
    }
    if (remainSec >= 0) {
      char tbuf[7];
      snprintf(tbuf, sizeof(tbuf), "%ld", (long)remainSec);
      const size_t len = strlen(tbuf);
      const size_t start = (len >= 16) ? 0 : (16 - len);
      for (size_t i = 0; i < len && (start + i) < 16; ++i) {
        buf[start + i] = tbuf[i];
      }
    }
    oled.drawString(0, 14, buf);
  } else {
    const char* uiId = KioskBehavior::Ui::uiStateId(g_ui.state);
    for (size_t i = 0; i < 16 && uiId[i] != 0; ++i) {
      buf[i] = uiId[i];
    }

    if (remainSec >= 0) {
      char tbuf[7];
      snprintf(tbuf, sizeof(tbuf), "%ld", (long)remainSec);
      const size_t len = strlen(tbuf);
      const size_t start = (len >= 16) ? 0 : (16 - len);
      for (size_t i = 0; i < len && (start + i) < 16; ++i) {
        buf[start + i] = tbuf[i];
      }
    }

    oled.drawString(0, 14, buf);
  }

}

static void oledPrintError(uint16_t code)
{
  char msg[17];
  if (code == KioskBehavior::ERR_UV_NOT_OK) {
    snprintf(msg, sizeof(msg), "ERROR:%04u (UV)", (unsigned)code);
  } else {
    snprintf(msg, sizeof(msg), "ERROR:%04u", (unsigned)code);
  }
  const size_t len = strlen(msg);
  char buf[17];
  for (size_t i = 0; i < 16; ++i) buf[i] = ' ';
  buf[16] = 0;
  const size_t start = (len >= 16) ? 0 : ((16 - len) / 2);
  for (size_t i = 0; i < len && (start + i) < 16; ++i) {
    buf[start + i] = msg[i];
  }
  oled.drawString(0, 1, buf);
}

static bool statusChanged(const StatusSnap& a, const StatusSnap& b)
{
  return a.wl      != b.wl
      || a.bw      != b.bw
      || a.hydSt   != b.hydSt
      || a.dispCnt != b.dispCnt
      || a.inletPh != b.inletPh;
}

#if KIOSK_HAS_EEPROM_EDITOR
static bool eepromEditorChordPressedNow()
{
  // EEPROM editor boot chord: SELECT + BACK (active-low).
  return !KioskIO::btnSensorBypass() && !KioskIO::btnBackwashCtl();
}

static void formatHwidLine(uint8_t hwid, char* out, size_t outLen)
{
  char hwidText[16];
  KioskFormatHwIdText(hwid, hwidText, sizeof(hwidText));
  snprintf(out, outLen, "HWID:%s", hwidText);
  pad16(out);
}

static void updateSwidLine()
{
  KioskFormatSwidText(g_swidLine, sizeof(g_swidLine));
  pad16(g_swidLine);
}

[[noreturn]] static void resetNow()
{
  wdt_enable(WDTO_15MS);
  while (1) { }
}

static void showEepromEditorReleasePrompt(uint8_t hwid, const char* swid)
{
  static const char* kReleaseLines[] = {
    "Release the",
    "invocation",
    "button(s)",
    "when ready",
    "to proceed"
  };

  oled.clearDisplay();
  oled.drawString(0, 0, "EEPROM Edit Mode");
  oled.drawString(0, 1, "----------------");
  oledDrawCenteredBlock(2, 15, kReleaseLines, 5);

  lcd.clear();
  lcdPrintLinePadded(0, "EEPROM Edit Mode");
  lcdPrintLinePadded(1, "----------------");
  char hwidLine[17];
  formatHwidLine(hwid, hwidLine, sizeof(hwidLine));
  lcdPrintLinePadded(2, hwidLine);
  if (swid) {
    lcdPrintLinePadded(3, swid);
  }
}

static void clearEepromEditorReleasePrompt()
{
  oled.clearDisplay();
}

static void waitForEepromEditorChordRelease(uint8_t hwid, const char* swid)
{
  showEepromEditorReleasePrompt(hwid, swid);
  while (eepromEditorChordPressedNow()) {
    delay(10);
  }
  clearEepromEditorReleasePrompt();
  delay(30);
}

[[noreturn]] static void runEepromEditor(bool startInPcbHwid)
{
  // NOTE: The EEPROM editor exits only via reset/watchdog; it never returns.
  Kiosk::KioskEepromEditor::Config cfg;
  cfg.ee = &g_eeprom;
  cfg.oled = &oled;
  cfg.lcd = &lcd;
  cfg.nfc = &g_editorNfc;
  cfg.allowTokenWrites = true;
  cfg.initPins = false; // KioskIO::begin already configured pins
  cfg.startInPcbHwid = startInPcbHwid;
  Kiosk::KioskEepromEditor::run(cfg);
}
#endif

static bool sequentialTestChordPressedNow()
{
  // Sequential test boot chord: Ozone OR Continuous Circulation button held (active-low).
  return !KioskIO::btnOzoneCtl() || !KioskIO::btnContCirc();
}

static bool hydraulicTestChordPressedNow()
{
  // Dispense calibration boot chord: Single Dispense (active-low).
  return !KioskIO::btnSingleDisp();
}

static void showDispenseCalReleasePrompt()
{
  static const char* kReleaseLines[] = {
    "Release the",
    "invocation",
    "button(s)",
    "when ready",
    "to proceed"
  };

  oled.clearDisplay();
  oled.drawString(0, 0, "DispenseCAL Mode");
  oled.drawString(0, 1, "----------------");
  oledDrawCenteredBlock(2, 15, kReleaseLines, 5);

  lcd.clear();
  lcdPrintLinePadded(0, "DispenseCAL Mode");
  lcdPrintLinePadded(1, "----------------");
  char hwidLine[17];
  formatHwidLine(g_eeprom.hwId(), hwidLine, sizeof(hwidLine));
  lcdPrintLinePadded(2, hwidLine);
  lcdPrintLinePadded(3, g_swidLine);
}

static void clearDispenseCalReleasePrompt()
{
  oled.clearDisplay();
}

static void waitForDispenseCalChordRelease()
{
  showDispenseCalReleasePrompt();
  while (hydraulicTestChordPressedNow()) {
    delay(10);
  }
  clearDispenseCalReleasePrompt();
  delay(30);
}

static void formatCalSecondsTenths(uint32_t ms, char* out, size_t outLen)
{
  const uint32_t tenths = (ms + 50UL) / 100UL;
  const uint32_t s = tenths / 10UL;
  const uint32_t t = tenths % 10UL;
  snprintf(out, outLen, "%lu.%1lus", (unsigned long)s, (unsigned long)t);
}

static void formatCalSecondsHundredths(uint32_t ms, char* out, size_t outLen)
{
  const uint32_t hundredths = (ms + 5UL) / 10UL;
  const uint32_t s = hundredths / 100UL;
  const uint32_t h = hundredths % 100UL;
  snprintf(out, outLen, "%lu.%02lus", (unsigned long)s, (unsigned long)h);
}

[[noreturn]] static void runDispenseCalMode()
{
  static constexpr uint16_t BTN_LONG_MS = 1500U;
  static constexpr uint16_t LONG_BLINK_MS = 50U;

  enum CalState : uint8_t { CAL_IDLE = 0, CAL_MEASURING = 1, CAL_REVIEW = 2 };
  CalState st = CAL_IDLE;

  DebouncedButtonTracker frontBtn{
    KioskPins::BTN_FRONT_DISPENSE, 0, 0, KioskIO::btnFrontDisp, false, false, false, 0UL, 0U
  };
  DebouncedButtonTracker contBtn{
    KioskPins::BTN_CONT_DISP, 0, 0, KioskIO::btnContDisp, false, false, false, 0UL, 0U
  };
  DebouncedButtonTracker rearBtn{
    KioskPins::BTN_SINGLE_DISP, 0, 0, KioskIO::btnSingleDisp, false, false, false, 0UL, 0U
  };
  frontBtn.begin();
  contBtn.begin();
  rearBtn.begin();

  // Ensure dispense path starts OFF in calibration mode.
  KioskIO::setWaterDispense(false);
  KioskIO::setWaterCirculation(false);
  KioskIO::setBackwash(false);
  KioskIO::setSensorBypass(false);

  uint32_t sampleStartMs = 0;
  uint32_t sampleMs = 0;
  uint32_t aggSumMs = 0;
  uint8_t  aggCount = 0;
  uint32_t lastUiMs = 0;
  bool frontLongBlinkDone = false;
  bool contLongBlinkDone = false;
  bool rearLongBlinkDone = false;

  auto blinkLongThreshold = [&]() {
    lcd.noBacklight();
    delay(LONG_BLINK_MS);
    lcd.backlight();
    g_backlightOn = true;
  };

  auto drawUi = [&](uint32_t now, const char* line1 = nullptr) {
    char l0[17], l1[17], l2[17], l3[17];
    snprintf(l0, sizeof(l0), "DispenseCAL Mode");
    if (line1) snprintf(l1, sizeof(l1), "%s", line1);
    else if (st == CAL_MEASURING) snprintf(l1, sizeof(l1), "Release=Stop");
    else if (st == CAL_REVIEW) snprintf(l1, sizeof(l1), "S=Rej L=Keep");
    else snprintf(l1, sizeof(l1), "PressHold=Sample");

    if (st == CAL_MEASURING) {
      const uint32_t liveMs = (uint32_t)(now - sampleStartMs);
      char t[12];
      formatCalSecondsTenths(liveMs, t, sizeof(t));
      snprintf(l2, sizeof(l2), "Live: %s", t);
    } else {
      char t[12];
      formatCalSecondsTenths(sampleMs, t, sizeof(t));
      snprintf(l2, sizeof(l2), "Sample: %s", t);
    }

    {
      char avgTok[16];
      if (aggCount > 0) {
        const uint32_t avgMs = (aggSumMs + (uint32_t)aggCount / 2U) / (uint32_t)aggCount;
        char t[12];
        formatCalSecondsHundredths(avgMs, t, sizeof(t));
        snprintf(avgTok, sizeof(avgTok), "AVG=%s", t);
      } else {
        snprintf(avgTok, sizeof(avgTok), "AVG=#.##s");
      }

      // Row 3 format:
      // - Left:  sample count (no leading zeros/spaces)
      // - Right: AVG token right-aligned to col 15.
      for (uint8_t i = 0; i < 16; ++i) l3[i] = ' ';
      l3[16] = 0;
      char nTok[8];
      snprintf(nTok, sizeof(nTok), "N=%u", (unsigned)aggCount);
      const size_t nLen = strlen(nTok);
      const size_t aLen = strlen(avgTok);
      for (size_t i = 0; i < nLen && i < 16; ++i) l3[i] = nTok[i];
      const size_t aStart = (aLen >= 16) ? 0 : (16 - aLen);
      for (size_t i = 0; i < aLen && (aStart + i) < 16; ++i) l3[aStart + i] = avgTok[i];
    }

    lcdPrintLinePadded(0, l0);
    lcdPrintLinePadded(1, l1);
    lcdPrintLinePadded(2, l2);
    lcdPrintLinePadded(3, l3);
  };

  oled.clearDisplay();
  oled.drawString(0, 0, "DispenseCAL Mode");
  oled.drawString(0, 1, "----------------");
  oled.drawString(0, 2, "Use FRONT button");
  oled.drawString(0, 3, "----------------");
  oled.drawString(0, 4, "Press+Hold disp");
  oled.drawString(0, 5, "button to start");
  oled.drawString(0, 6, "Release to stop");
  oled.drawString(0, 7, "----------------");
  oled.drawString(0, 8, " Short press to");
  oled.drawString(0, 9, " reject sample");
  oled.drawString(0, 10, " Long press to");
  oled.drawString(0, 11, " accept sample");
  oled.drawString(0, 12, "----------------");
  oled.drawString(0, 13, "Press+Hold rear");
  oled.drawString(0, 14, "  disp button");
  oled.drawString(0, 15, "saves to EEPROM");

  lcd.clear();
  lcd.backlight();
  g_backlightOn = true;
  drawUi(millis());

  while (1) {
    const uint32_t now = millis();
    frontBtn.update(now);
    contBtn.update(now);
    rearBtn.update(now);

    bool uiDirty = false;

    // One-shot blink feedback when a held press first crosses long-press threshold.
    // Front long-press is meaningful only in CAL_REVIEW (keep sample).
    if (st == CAL_REVIEW && frontBtn.isDown()) {
      if (!frontLongBlinkDone &&
          (uint32_t)(now - (uint32_t)frontBtn.pressStartMs) >= (uint32_t)BTN_LONG_MS) {
        blinkLongThreshold();
        frontLongBlinkDone = true;
        uiDirty = true;
      }
    } else {
      frontLongBlinkDone = false;
    }
    // Cont/rear long-press is meaningful when at least one sample exists and
    // no active measurement is running (save + exit).
    if (st != CAL_MEASURING && aggCount > 0U && contBtn.isDown()) {
      if (!contLongBlinkDone &&
          (uint32_t)(now - (uint32_t)contBtn.pressStartMs) >= (uint32_t)BTN_LONG_MS) {
        blinkLongThreshold();
        contLongBlinkDone = true;
        uiDirty = true;
      }
    } else {
      contLongBlinkDone = false;
    }
    if (st != CAL_MEASURING && aggCount > 0U && rearBtn.isDown()) {
      if (!rearLongBlinkDone &&
          (uint32_t)(now - (uint32_t)rearBtn.pressStartMs) >= (uint32_t)BTN_LONG_MS) {
        blinkLongThreshold();
        rearLongBlinkDone = true;
        uiDirty = true;
      }
    } else {
      rearLongBlinkDone = false;
    }

    // Front button controls sample start/stop and accept/reject review.
    // - In CAL_IDLE: press starts measuring immediately (press-and-hold sample).
    // - In CAL_MEASURING: release stops and captures sample.
    // - In CAL_REVIEW: release short=reject, long=keep.
    if (frontBtn.takePressed()) {
      if (st == CAL_IDLE && KioskIO::tankWaterLevel() > 0U) {
        KioskIO::setWaterDispense(true);
        sampleStartMs = now;
        st = CAL_MEASURING;
        uiDirty = true;
      }
    }
    if (frontBtn.takeReleased()) {
      const uint16_t heldMs = frontBtn.lastDurationMs();
      if (st == CAL_MEASURING) {
        KioskIO::setWaterDispense(false);
        sampleMs = (uint32_t)(now - sampleStartMs);
        st = CAL_REVIEW;
        uiDirty = true;
      } else { // CAL_REVIEW
        if (heldMs >= BTN_LONG_MS) {
          // Keep sample.
          if (aggCount < 255U) aggCount++;
          aggSumMs += sampleMs;
          st = CAL_IDLE;
        } else {
          // Reject sample.
          st = CAL_IDLE;
        }
        uiDirty = true;
      }
    }

    // CONT_DISP controls reject in review and save+exit.
    if (contBtn.takeReleased()) {
      const uint16_t heldMs = contBtn.lastDurationMs();
      if (st == CAL_REVIEW && heldMs < BTN_LONG_MS) {
        st = CAL_IDLE; // reject sample
        uiDirty = true;
      } else if (st != CAL_MEASURING && heldMs >= BTN_LONG_MS) {
        if (aggCount > 0U && g_eeprom.isReady()) {
          const uint32_t avgMs = (aggSumMs + (uint32_t)aggCount / 2U) / (uint32_t)aggCount;
          g_eeprom.setMeasuredDurationMs(avgMs);
          drawUi(now, "Saved. Resetting");
          delay(1000);
          wdt_enable(WDTO_15MS);
          while (1) { }
        } else {
          drawUi(now, "No samples to save");
          delay(700);
          uiDirty = true;
        }
      }
    }

    // BTN_SINGLE_DISP (rear) long-press save+exit when samples exist.
    if (rearBtn.takeReleased()) {
      const uint16_t heldMs = rearBtn.lastDurationMs();
      if (st != CAL_MEASURING && heldMs >= BTN_LONG_MS) {
        if (aggCount > 0U && g_eeprom.isReady()) {
          const uint32_t avgMs = (aggSumMs + (uint32_t)aggCount / 2U) / (uint32_t)aggCount;
          g_eeprom.setMeasuredDurationMs(avgMs);
          drawUi(now, "Saved. Resetting");
          delay(1000);
          wdt_enable(WDTO_15MS);
          while (1) { }
        } else {
          drawUi(now, "No samples to save");
          delay(700);
          uiDirty = true;
        }
      }
    }

    // Safety stop while measuring if tank goes empty.
    if (st == CAL_MEASURING && KioskIO::tankWaterLevel() == 0U) {
      KioskIO::setWaterDispense(false);
      sampleMs = (uint32_t)(now - sampleStartMs);
      st = CAL_REVIEW;
      uiDirty = true;
    }

    if (st == CAL_MEASURING) {
      // Refresh live timer at 10 Hz for xx.xs rendering.
      if ((uint32_t)(now - lastUiMs) >= 100UL) {
        uiDirty = true;
      }
    }

    if (uiDirty) {
      drawUi(now);
      lastUiMs = now;
    }
    delay(10);
  }
}

void setup()
{
  Wire.begin();
  // Serial logging disabled for demo.

  KioskIO::begin();
  updateSwidLine();

  bool bootChordActive = false;
#if KIOSK_HAS_EEPROM_EDITOR
  bootChordActive = bootChordActive || eepromEditorChordPressedNow();
#endif
  bootChordActive =
    bootChordActive || sequentialTestChordPressedNow() || hydraulicTestChordPressedNow();

  // Initialize displays
  oled.begin();
  oled.setFont(u8x8_font_chroma48medium8_r);
  oled.clearDisplay();
  if (!bootChordActive) {
    oled.drawString(0, 0, "Kiosk Operations");
  }

  lcd.begin(16, 4);
  lcd.clear();
  // No default LCD title screen.
  delay(200);

  const bool eepromReady = g_eeprom.begin();
  const bool magicOk = eepromReady && g_eeprom.magicValid();
  g_eeprom.ensureMagicOrInitDefaults();
  const uint8_t hwid = g_eeprom.hwId();

#if KIOSK_HAS_EEPROM_EDITOR
  if (!magicOk || hwid == 0) {
    runEepromEditor(true);
  }
#endif

#if KIOSK_HAS_EEPROM_EDITOR
  if (eepromEditorChordPressedNow()) {
    waitForEepromEditorChordRelease(hwid, g_swidLine);
    runEepromEditor(false);
  }
#endif

  if (sequentialTestChordPressedNow()) {
    KioskSeqTest::Config seqCfg;
    seqCfg.oled = &oled;
    seqCfg.lcd = &lcd;
    seqCfg.hwid = hwid;
    seqCfg.swid = g_swidLine;
    KioskSeqTest::run(seqCfg);
  }

  if (hydraulicTestChordPressedNow()) {
    waitForDispenseCalChordRelease();
    runDispenseCalMode();
  }

  lcdInitProgressChars();

  KioskHydraulics::begin();

  // Start with LED ring OFF
  analogWrite(KioskPins::PWM_DISP_BTTN_LED, 0);
  KioskIO::setBtnFrontDispLEDMode(KioskIO::LED_OFF);

  // Inlet solenoid profile:
  // load from EEPROM when available, otherwise use safe defaults.
  if (g_eeprom.isReady()) {
    const auto inlet = g_eeprom.solenoidProfile(Kiosk::KioskEeprom::SOL_INLET);
    const uint32_t kickMs = (inlet.swDelaySec > 0U) ? ((uint32_t)inlet.swDelaySec * 1000UL) : 5000UL;
    KioskIO::setWaterInletKickPwm(inlet.startPwm);
    KioskIO::setWaterInletHoldPwm(inlet.holdPwm);
    KioskIO::setWaterInletKickMs(kickMs);
  } else {
    KioskIO::setWaterInletKickPwm(255);
    KioskIO::setWaterInletHoldPwm(130);
    KioskIO::setWaterInletKickMs(5000UL);
  }

  btnBackwash.begin();
  btnInlet.begin();
  btnDispense.begin();
  btnSensorBypass.begin();
  btnFrontDisp.begin();
  btnContDisp.begin();

  if (!g_eeprom.isReady()) g_eeprom.begin();
  KioskBehavior::Ui::uiInit(g_ui, g_eeprom.isReady() ? &g_eeprom : nullptr, millis());

  if (g_eeprom.isReady()) {
    const auto profile = g_eeprom.dispMeasuredProfile();
    KioskHydraulics::setDispenseProfile(g_eeprom.dispMeasuredDurationMs(), profile.pulses, profile.modeSel);
  } else {
    KioskHydraulics::setDispenseProfile(5000UL, 0, 0);
  }

}

void loop()
{
  const uint32_t now = (uint32_t)millis();

  // One-shot request pulses for this loop() iteration.
  // These are consumed by KioskBehavior and are not carried between loop cycles.
  bool backwashPressPulse = false;
  bool backwashReqPulse = false;
  uint16_t backwashLastPressMs = 0U;
  bool dayEndModeReqPulse = false;
  bool inletTogglePulse = false;
  bool btnFrontDispPulse = false;
  bool btnFrontDispDown = false;
  bool rearDispensePulse = false;
  bool contDispPressPulse = false;
  bool contDispDown = false;
  bool contDispReleasePulse = false;
  uint16_t contDispLastPressMs = 0U;
  bool sensorBypassTogglePulse = false;
  uint16_t sensorBypassLastPressMs = 0U;

  btnFrontDisp.update(now);
  btnBackwash.update(now);
  const bool backwashDown = btnBackwash.isDown();
  btnSensorBypass.update(now);
  btnFrontDispDown = btnFrontDisp.isDown();
  // Circulation trigger uses direct raw BTN_CONT_CIRC state (active-low)
  // to avoid missed release events under heavy loop work.
  const bool contDispDownRaw = (!KioskIO::btnContCirc());
  contDispDown = contDispDownRaw;
  contDispPressPulse = (contDispDownRaw && !g_prevContDispDown);
  contDispReleasePulse = (!contDispDownRaw && g_prevContDispDown);
  if (contDispPressPulse) {
    g_contDispPressStartMs = now;
  }
  if (contDispReleasePulse) {
    const uint32_t held = (uint32_t)(now - g_contDispPressStartMs);
    contDispLastPressMs =
      (held > (uint32_t)PRESS_DURATION_MAX_MS) ? PRESS_DURATION_OVERFLOW : (uint16_t)held;
  }
  g_prevContDispDown = contDispDownRaw;

  // Backwash request pulse uses press->release duration (handled by KioskBehavior).
  if (btnBackwash.takePressed()) {
    backwashPressPulse = true;
  }
  if (btnBackwash.takeReleased()) {
    backwashReqPulse = true;
    backwashLastPressMs = btnBackwash.lastDurationMs();
  }
  // Water inlet request pulse (hydraulics will accept/ignore per its rules)
  if (btnInlet.fell()) inletTogglePulse = true;
  // Dispense request pulses (eligibility handled in KioskBehavior).
  if (btnDispense.fell()) {
    rearDispensePulse = true;
  }
  if (btnFrontDisp.takePressed()) btnFrontDispPulse = true;
  if (btnSensorBypass.takeReleased()) {
    sensorBypassTogglePulse = true;
    sensorBypassLastPressMs = btnSensorBypass.lastDurationMs();
  }

  // DayEnd Circ+BW chord sequence:
  // 1) Hold BTN_BACKWASH_CTL
  // 2) Hold BTN_CONT_CIRC
  // 3) Release BTN_BACKWASH_CTL
  // 4) Release BTN_CONT_CIRC
  // Request is honored only when override switch is active.
  switch (g_dayEndChordState) {
    case DAYEND_CHORD_IDLE:
      if (backwashPressPulse && !contDispDownRaw) {
        g_dayEndChordState = DAYEND_CHORD_BW_HELD;
      }
      break;
    case DAYEND_CHORD_BW_HELD:
      if (!backwashDown) {
        g_dayEndChordState = DAYEND_CHORD_IDLE;
      } else if (contDispPressPulse || (contDispDownRaw && backwashDown)) {
        g_dayEndChordState = DAYEND_CHORD_BOTH_HELD;
      }
      break;
    case DAYEND_CHORD_BOTH_HELD:
      if (!contDispDownRaw && backwashDown) {
        // Wrong release order; abort chord.
        g_dayEndChordState = DAYEND_CHORD_BW_HELD;
      } else if ((backwashReqPulse || !backwashDown) && contDispDownRaw) {
        // Consume this release as part of the chord, not as a manual backwash request.
        backwashReqPulse = false;
        g_dayEndChordState = DAYEND_CHORD_BW_RELEASED_WAIT_CIRC_RELEASE;
      }
      break;
    case DAYEND_CHORD_BW_RELEASED_WAIT_CIRC_RELEASE:
      if (backwashDown) {
        g_dayEndChordState = DAYEND_CHORD_BOTH_HELD;
      } else if (contDispReleasePulse || !contDispDownRaw) {
        dayEndModeReqPulse = true;
        g_dayEndChordState = DAYEND_CHORD_IDLE;
      }
      break;
    default:
      g_dayEndChordState = DAYEND_CHORD_IDLE;
      break;
  }

  if (g_dayEndChordState != DAYEND_CHORD_IDLE || dayEndModeReqPulse) {
    // While the chord is in progress, suppress normal BTN_CONT_CIRC actions.
    contDispPressPulse = false;
    contDispReleasePulse = false;
  }
  if (dayEndModeReqPulse && KioskIO::overrideActive()) {
    KioskBehavior::requestDayEndCircBwMode();
  }

  // UI state machine (non-blocking)
  KioskBehavior::Ui::UiInputs uiIn{};
  uiIn.btnFrontDispPressed = btnFrontDispPulse;
  uiIn.btnFrontDispDown = btnFrontDispDown;
  uiIn.btnRearDispPressed = rearDispensePulse;
  uiIn.dispenseNotOk = !dispenseOkNow();
  uiIn.filterBackwashActive = KioskHydraulics::backwashActive();
  uiIn.hasCoinAcceptor = g_eeprom.isReady() ? g_eeprom.coinAcceptorFitted() : false;
  uiIn.hasNfc = true;

  KioskBehavior::Ui::uiUpdate(g_ui, uiIn, g_uiOut, now);

  // Apply UI outputs (LCD + backlight + LED ring).
  const KioskBehavior::Ui::LedMode ledMode =
    (g_uiOut.btnFrontDispLED == KioskBehavior::Ui::LED_ON || g_uiOut.dispenseButtonLed == KioskBehavior::Ui::LED_ON)
      ? KioskBehavior::Ui::LED_ON
      : ((g_uiOut.btnFrontDispLED == KioskBehavior::Ui::LED_PULSE || g_uiOut.dispenseButtonLed == KioskBehavior::Ui::LED_PULSE)
         ? KioskBehavior::Ui::LED_PULSE
         : KioskBehavior::Ui::LED_OFF);
  applyDispenseLed(ledMode);
  applyBacklight(g_uiOut.backlight, (uint32_t)(now - g_ui.stateStartMs));
  if (g_ui.state != KioskBehavior::Ui::UI_DISPENSING) {
    g_lcdDispensingFrameWritten = false;
  }
  if (g_uiOut.lcdDirty) {
    snprintf(g_lcdPendingLine0, sizeof(g_lcdPendingLine0), "%s", g_uiOut.line0);
    snprintf(g_lcdPendingLine1, sizeof(g_lcdPendingLine1), "%s", g_uiOut.line1);
    snprintf(g_lcdPendingLine2, sizeof(g_lcdPendingLine2), "%s", g_uiOut.line2);
    snprintf(g_lcdPendingLine3, sizeof(g_lcdPendingLine3), "%s", g_uiOut.line3);
    g_lcdPending = true;
  }
  if (g_lcdPending) {
    const bool dispensingUi = (g_ui.state == KioskBehavior::Ui::UI_DISPENSING);
    if (!dispensingUi) {
      lcd.setCursor(0, 0); lcd.print(g_lcdPendingLine0);
      lcd.setCursor(0, 1); lcd.print(g_lcdPendingLine1);
      lcd.setCursor(0, 2); lcd.print(g_lcdPendingLine2);
      lcd.setCursor(0, 3); lcd.print(g_lcdPendingLine3);
      g_lastLcdWriteMs = now;
      g_lcdPending = false;
    } else if (!g_lcdDispensingFrameWritten) {
      lcd.setCursor(0, 0); lcd.print(g_lcdPendingLine0);
      lcd.setCursor(0, 1); lcd.print(g_lcdPendingLine1);
      lcd.setCursor(0, 2); lcd.print(g_lcdPendingLine2);
      lcd.setCursor(0, 3); lcd.print(g_lcdPendingLine3);
      g_lastLcdWriteMs = now;
      g_lcdPending = false;
      g_lcdDispensingFrameWritten = true;
    } else {
      const bool lcdWriteAllowed = (g_lastLcdWriteMs == 0) ||
                                   ((uint32_t)(now - g_lastLcdWriteMs) >= 200UL);
      if (lcdWriteAllowed) {
        lcd.setCursor(0, 3); lcd.print(g_lcdPendingLine3);
        g_lastLcdWriteMs = now;
        g_lcdPending = false;
      }
    }
  }
  oledPrintUiState(now);
  if (g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_IN ||
      g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_OUT) {
    char dmLine[17];
    snprintf(dmLine, sizeof(dmLine), "%s", KioskBehavior::Ui::uiStateId(g_ui.state));
    pad16(dmLine);
    oled.drawString(0, 0, dmLine);
  }

  if (g_uiOut.requestReset) {
    resetNow();
  }

  // Run hydraulics policy update
  const bool dayEndQueuedOrActive = KioskBehavior::dayEndModeActive();
  const bool rearDispenseAllowed =
    (g_ui.state == KioskBehavior::Ui::UI_DISP_RDY);
  const bool dispenseRequestedThisLoop =
    !dayEndQueuedOrActive &&
    ((rearDispenseAllowed && rearDispensePulse) || g_uiOut.requestDispensePulse);
  const KioskBehavior::BackwashEvent bwEvt =
    KioskBehavior::updateHydraulics(now,
                                    inletTogglePulse,
                                    false, // btnFrontDisp handled by UI state machine
                                    dispenseRequestedThisLoop,
                                    contDispPressPulse,
                                    contDispDown,
                                    contDispReleasePulse,
                                    contDispLastPressMs,
                                    sensorBypassTogglePulse,
                                    sensorBypassLastPressMs,
                                    backwashReqPulse,
                                    backwashLastPressMs,
                                    (g_ui.state == KioskBehavior::Ui::UI_WELCOME),
                                    (g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_IN ||
                                     g_ui.state == KioskBehavior::Ui::UI_DARK_MODE_OUT),
                                    g_eeprom.isReady() ? &g_eeprom : nullptr);
  if (KioskBehavior::errorActive()) {
    oledPrintError(KioskBehavior::errorCode());
  } else {
    oledClearRow(1);
  }
  // Status reporting only on change
  StatusSnap cur;
  cur.wl      = KioskIO::tankWaterLevel();
  cur.bw      = KioskHydraulics::backwashActive();
  cur.hydSt   = KioskHydraulics::state();
  cur.dispCnt = KioskHydraulics::dispenseCount();
  cur.inletPh = KioskIO::waterInletPhase();
  const bool sensorOnNow = KioskBehavior::sensorBypassActive();
  const bool backwashOnNow = cur.bw;
  const bool waterInOnNow = (cur.inletPh != 0);
  const bool dispenseOnNow = (cur.hydSt != 0);
  const uint8_t prevWl = g_haveLast ? g_last.wl : cur.wl;
  const bool waterLevelChanged = (cur.wl != prevWl);

  if (sensorOnNow && !g_prevSensorBypassOn) g_sensorBypassOnStartMs = now;
  if (backwashOnNow && !g_prevBackwashOn) g_backwashOnStartMs = now;
  // Reset WaterInp ON-duration only when inlet turns ON because water level changed.
  if (waterInOnNow && !g_prevWaterInOn && waterLevelChanged) g_waterInOnStartMs = now;
  if (dispenseOnNow && !g_prevDispenseOn) g_dispenseOnStartMs = now;

  if (dispenseOnNow) g_dispensePendingUntilMs = 0;
  else if (dispenseRequestedThisLoop) g_dispensePendingUntilMs = now + 1000UL;

  g_prevSensorBypassOn = sensorOnNow;
  g_prevBackwashOn = backwashOnNow;
  g_prevWaterInOn = waterInOnNow;
  g_prevDispenseOn = dispenseOnNow;

  oledPrintStatus(cur, now);
  g_last = cur;
  g_haveLast = true;
}
