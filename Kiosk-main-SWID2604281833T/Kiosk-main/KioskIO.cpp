// KioskIO.cpp
// -----------------------------------------------------------------------------
// Centralized hardware facade for kiosk firmware.
// Responsibilities:
// - pin-level reads/writes and boot-safe pin initialization
// - ISR-backed edge/pulse capture (buttons, flow sensors)
// - actuator drive helpers (including inlet kick->hold profile)
// - RTC and NFC device access

#include "KioskIO.h"
#include "KioskIOpins.h"
#include "KioskEepromLayout.h"

#include <EEPROM.h>
#include <Adafruit_PN532.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

namespace KioskIO {

   // -------------------- Basic helpers --------------------
   bool readPin(uint8_t pin) {
      return (digitalRead(pin) == HIGH);
   }

   void writePin(uint8_t pin, bool high) {
      digitalWrite(pin, high ? HIGH : LOW);
   }

   void safeMakeOutputLow(uint8_t pin) {
      digitalWrite(pin, LOW);
      pinMode(pin, OUTPUT);
   }

   bool overrideActive() {
      return readPin(KioskPins::OVERRIDE_SWITCH);
   }

   bool btnBackwashCtl() {
      return readPin(KioskPins::BTN_BACKWASH_CTL);
   }

   bool btnSensorBypass() {
      return readPin(KioskPins::BTN_SENSOR_BYPASS);
   }

   bool btnContCirc() {
      return readPin(KioskPins::BTN_CONT_CIRC);
   }

   bool btnWaterInlet() {
      return readPin(KioskPins::BTN_WATER_INLET);
   }

   bool btnContDisp() {
      return readPin(KioskPins::BTN_CONT_DISP);
   }

   bool btnSingleDisp() {
      return readPin(KioskPins::BTN_SINGLE_DISP);
   }

   bool btnOzoneCtl() {
      return readPin(KioskPins::BTN_OZONE_CTL);
   }

   bool btnFrontDisp() {
      return readPin(KioskPins::BTN_FRONT_DISPENSE);
   }

   // -------------------- Front Button LED Ring (Timer5 ISR waveform) --------------------
   static constexpr uint8_t LED_WAVE_SAMPLES = 50;

   const uint8_t ledWaveforms[LED_MODE_COUNT][LED_WAVE_SAMPLES] PROGMEM = {
      // LED_OFF
      {
         0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0,
         0,0,0,0,0,0,0,0,0,0
      },
      // LED_BEACON
      {
           0,  18,  36,  55,  73,  91, 109, 128, 146, 164,
         182, 200, 219, 237, 255,
         255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
         255, 237, 219, 200, 182, 164, 146, 128, 109,  91,
          73,  55,  36,  18,   0,
           0,   0,   0,   0,   0,   0,   0,   0,   0,   0
      },
      // LED_FLASH: 200 ms on, 300 ms off.
      {
         255,255,255,255,255,255,255,255,255,255,
         255,255,255,255,255,255,255,255,255,255,
           0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
           0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
           0,  0,  0,  0,  0,  0,  0,  0,  0,  0
      },
      // LED_ON: steady.
      {
         255,255,255,255,255,255,255,255,255,255,
         255,255,255,255,255,255,255,255,255,255,
         255,255,255,255,255,255,255,255,255,255,
         255,255,255,255,255,255,255,255,255,255,
         255,255,255,255,255,255,255,255,255,255
      }
   };

   static volatile LedMode s_frontLedMode = LED_OFF;
   static volatile bool s_frontLedIsrEnabled = false;

   static void setupTimer5_10ms() {
      cli();
      TCCR5A = 0;
      TCCR5B = 0;
      TCNT5  = 0;

      TCCR5B |= _BV(WGM52);  // CTC
      OCR5A = 2499;          // 10 ms @ 16 MHz, prescaler 64
      TIMSK5 |= _BV(OCIE5A);
      TCCR5B |= _BV(CS50) | _BV(CS51);
      sei();
   }

   void enableBtnFrontDispLEDIsr(bool enable) {
      s_frontLedIsrEnabled = enable;
   }

   void setBtnFrontDispLEDMode(LedMode mode) {
      if (mode >= LED_MODE_COUNT) mode = LED_OFF;
      s_frontLedMode = mode;
      s_frontLedIsrEnabled = true;
   }

   LedMode btnFrontDispLEDMode() {
      return s_frontLedMode;
   }

   // -------------------- RTC (I2C) --------------------
   namespace {
      constexpr uint8_t RTC_DS3231_ADDR = 0x68;
      bool s_rtcPresent = false;

      uint8_t bcdToDec(uint8_t v) {
         return (uint8_t)((v >> 4) * 10 + (v & 0x0F));
      }

      uint8_t decToBcd(uint8_t v) {
         return (uint8_t)(((v / 10) << 4) | (v % 10));
      }

      bool rtcProbeDs3231() {
         // Minimal-power probe: I2C address ACK only.
         Wire.beginTransmission(RTC_DS3231_ADDR);
         return (Wire.endTransmission() == 0);
      }

      bool rtcReadDs3231(RtcTime &out) {
         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write((uint8_t)0x00); // seconds register
         if (Wire.endTransmission() != 0) return false;

         if (Wire.requestFrom(RTC_DS3231_ADDR, (uint8_t)7) != 7) return false;
         uint8_t rawSec = Wire.read();
         uint8_t rawMin = Wire.read();
         uint8_t rawHour = Wire.read();
         uint8_t rawDow = Wire.read();
         uint8_t rawDay = Wire.read();
         uint8_t rawMonth = Wire.read();
         uint8_t rawYear = Wire.read();

         out.second = bcdToDec(rawSec & 0x7F);
         out.minute = bcdToDec(rawMin & 0x7F);

         if (rawHour & 0x40) {
            // 12-hour mode
            uint8_t hr12 = bcdToDec(rawHour & 0x1F);
            bool pm = (rawHour & 0x20) != 0;
            if (hr12 == 12) out.hour = pm ? 12 : 0;
            else out.hour = pm ? (uint8_t)(hr12 + 12) : hr12;
         } else {
            // 24-hour mode
            out.hour = bcdToDec(rawHour & 0x3F);
         }

         out.day = bcdToDec(rawDay & 0x3F);
         out.dow = bcdToDec(rawDow & 0x07);
         if (out.dow < 1 || out.dow > 7) out.dow = 1;
         out.month = bcdToDec(rawMonth & 0x1F);
         out.year = (uint16_t)(2000 + bcdToDec(rawYear));
         return true;
      }

      bool rtcSetDs3231(const RtcTime &in) {
         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write((uint8_t)0x00); // seconds register
         Wire.write(decToBcd(in.second));
         Wire.write(decToBcd(in.minute));
         Wire.write(decToBcd(in.hour)); // force 24-hour mode
         const uint8_t dow = (in.dow >= 1 && in.dow <= 7) ? in.dow : 1;
         Wire.write((uint8_t)dow); // day-of-week
         Wire.write(decToBcd(in.day));
         Wire.write(decToBcd(in.month)); // century bit cleared
         Wire.write(decToBcd((uint8_t)(in.year % 100)));
         return (Wire.endTransmission() == 0);
      }

      bool rtcReadReg(uint8_t reg, uint8_t &out) {
         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write(reg);
         if (Wire.endTransmission() != 0) return false;
         if (Wire.requestFrom(RTC_DS3231_ADDR, (uint8_t)1) != 1) return false;
         out = Wire.read();
         return true;
      }

      bool rtcWriteReg(uint8_t reg, uint8_t value) {
         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write(reg);
         Wire.write(value);
         return (Wire.endTransmission() == 0);
      }

      // Alarm interrupts must remain disabled; Alarm1 registers are used as
      // non-volatile storage for the last daily backwash timestamp.
      void rtcDisableAlarmInterrupts() {
         uint8_t ctrl = 0;
         if (!rtcReadReg(0x0E, ctrl)) return;
         ctrl = (uint8_t)(ctrl & ~0x03U); // clear A1IE/A2IE
         (void)rtcWriteReg(0x0E, ctrl);
      }

      static constexpr uint16_t BW_STAMP_BASE_YEAR = 2000U;
      static constexpr uint8_t BW_STAMP_MAX_YEAR_OFFSET = 225U; // 2225 - 2000

      uint8_t clampYearOffset(uint16_t year) {
         if (year < BW_STAMP_BASE_YEAR) return 0U;
         uint16_t off = (uint16_t)(year - BW_STAMP_BASE_YEAR);
         if (off > (uint16_t)BW_STAMP_MAX_YEAR_OFFSET) off = BW_STAMP_MAX_YEAR_OFFSET;
         return (uint8_t)off;
      }

      uint8_t clampMonth(uint8_t month) {
         if (month < 1U || month > 12U) return 1U;
         return month;
      }

      uint8_t clampDay(uint8_t day) {
         if (day < 1U || day > 31U) return 1U;
         return day;
      }

      void writeDailyStampYearMonth(const RtcTime &in) {
         EEPROM.update(Kiosk::EepromLayout::ADDR_DAILY_BW_YEAR_OFFSET, clampYearOffset(in.year));
         EEPROM.update(Kiosk::EepromLayout::ADDR_DAILY_BW_MONTH, clampMonth(in.month));
      }

      void writeTrigStampYearMonth(const RtcTime &in) {
         EEPROM.update(Kiosk::EepromLayout::ADDR_TRIG_BW_YEAR_OFFSET, clampYearOffset(in.year));
         EEPROM.update(Kiosk::EepromLayout::ADDR_TRIG_BW_MONTH, clampMonth(in.month));
      }

      void readDailyStampYearMonth(RtcTime &out) {
         const uint8_t y = EEPROM.read(Kiosk::EepromLayout::ADDR_DAILY_BW_YEAR_OFFSET);
         const uint8_t m = EEPROM.read(Kiosk::EepromLayout::ADDR_DAILY_BW_MONTH);
         const uint8_t yOff = (y <= BW_STAMP_MAX_YEAR_OFFSET) ? y : 0U;
         out.year = (uint16_t)(BW_STAMP_BASE_YEAR + yOff);
         out.month = clampMonth(m);
      }

      void readTrigStampYearMonth(RtcTime &out) {
         const uint8_t y = EEPROM.read(Kiosk::EepromLayout::ADDR_TRIG_BW_YEAR_OFFSET);
         const uint8_t m = EEPROM.read(Kiosk::EepromLayout::ADDR_TRIG_BW_MONTH);
         const uint8_t yOff = (y <= BW_STAMP_MAX_YEAR_OFFSET) ? y : 0U;
         out.year = (uint16_t)(BW_STAMP_BASE_YEAR + yOff);
         out.month = clampMonth(m);
      }

      // Sakamoto algorithm: returns 0..6 => Sun..Sat.
      uint8_t calcDowSun0(uint16_t year, uint8_t month, uint8_t day) {
         static const uint8_t t[12] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
         uint16_t y = year;
         if (month < 3) y--;
         return (uint8_t)((y + y/4 - y/100 + y/400 + t[month - 1] + day) % 7U);
      }

      // DS3231 convention used here: 1..7 => Sun..Sat.
      uint8_t calcDowDs3231(const RtcTime &in) {
         if (in.month < 1 || in.month > 12 || in.day < 1 || in.day > 31 || in.year < 1900) return 1;
         return (uint8_t)(calcDowSun0(in.year, in.month, in.day) + 1U);
      }

      bool rtcWriteAlarm1Stamp(const RtcTime &in) {
         // Alarm1 registers: 0x07 sec, 0x08 min, 0x09 hour, 0x0A day/date.
         // Use date mode (DY/DT=0), with alarm mask bits cleared.
         const uint8_t sec  = (uint8_t)(decToBcd(in.second % 60U) & 0x7FU);
         const uint8_t min  = (uint8_t)(decToBcd(in.minute % 60U) & 0x7FU);
         const uint8_t hour = (uint8_t)(decToBcd(in.hour % 24U) & 0x3FU);
         const uint8_t dateReg = (uint8_t)(decToBcd(clampDay(in.day)) & 0x3FU); // DY/DT=0

         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write((uint8_t)0x07);
         Wire.write(sec);
         Wire.write(min);
         Wire.write(hour);
         Wire.write(dateReg);
         if (Wire.endTransmission() != 0) return false;

         writeDailyStampYearMonth(in);
         rtcDisableAlarmInterrupts();
         return true;
      }

      bool rtcReadAlarm1Stamp(RtcTime &out) {
         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write((uint8_t)0x07);
         if (Wire.endTransmission() != 0) return false;
         if (Wire.requestFrom(RTC_DS3231_ADDR, (uint8_t)4) != 4) return false;
         const uint8_t rawSec = Wire.read();
         const uint8_t rawMin = Wire.read();
         const uint8_t rawHour = Wire.read();
         const uint8_t rawDayDate = Wire.read();

         out.second = bcdToDec(rawSec & 0x7F);
         out.minute = bcdToDec(rawMin & 0x7F);
         if (rawHour & 0x40) {
            uint8_t hr12 = bcdToDec(rawHour & 0x1F);
            bool pm = (rawHour & 0x20) != 0;
            if (hr12 == 12) out.hour = pm ? 12 : 0;
            else out.hour = pm ? (uint8_t)(hr12 + 12) : hr12;
         } else {
            out.hour = bcdToDec(rawHour & 0x3F);
         }

         readDailyStampYearMonth(out);
         out.day = bcdToDec(rawDayDate & 0x3F);
         out.day = clampDay(out.day);
         out.dow = calcDowDs3231(out);
         return true;
      }

      bool rtcWriteAlarm2Stamp(const RtcTime &in) {
         // Alarm2 registers: 0x0B min, 0x0C hour, 0x0D day/date.
         // Use date mode (DY/DT=0), with alarm mask bits cleared.
         const uint8_t min  = (uint8_t)(decToBcd(in.minute % 60U) & 0x7FU);
         const uint8_t hour = (uint8_t)(decToBcd(in.hour % 24U) & 0x3FU);
         const uint8_t dateReg = (uint8_t)(decToBcd(clampDay(in.day)) & 0x3FU); // DY/DT=0

         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write((uint8_t)0x0B);
         Wire.write(min);
         Wire.write(hour);
         Wire.write(dateReg);
         if (Wire.endTransmission() != 0) return false;

         writeTrigStampYearMonth(in);
         rtcDisableAlarmInterrupts();
         return true;
      }

      bool rtcReadAlarm2Stamp(RtcTime &out) {
         Wire.beginTransmission(RTC_DS3231_ADDR);
         Wire.write((uint8_t)0x0B);
         if (Wire.endTransmission() != 0) return false;
         if (Wire.requestFrom(RTC_DS3231_ADDR, (uint8_t)3) != 3) return false;
         const uint8_t rawMin = Wire.read();
         const uint8_t rawHour = Wire.read();
         const uint8_t rawDayDate = Wire.read();

         out.second = 0;
         out.minute = bcdToDec(rawMin & 0x7F);
         if (rawHour & 0x40) {
            uint8_t hr12 = bcdToDec(rawHour & 0x1F);
            bool pm = (rawHour & 0x20) != 0;
            if (hr12 == 12) out.hour = pm ? 12 : 0;
            else out.hour = pm ? (uint8_t)(hr12 + 12) : hr12;
         } else {
            out.hour = bcdToDec(rawHour & 0x3F);
         }

         readTrigStampYearMonth(out);
         out.day = bcdToDec(rawDayDate & 0x3F);
         out.day = clampDay(out.day);
         out.dow = calcDowDs3231(out);
         return true;
      }
   }

   uint8_t rtcBegin() {
      s_rtcPresent = rtcProbeDs3231();
      if (s_rtcPresent) rtcDisableAlarmInterrupts();
      return s_rtcPresent ? 1 : 0;
   }

   bool rtcPresent() {
      return s_rtcPresent;
   }

   bool rtcRead(RtcTime &out) {
      if (!s_rtcPresent) return false;
      return rtcReadDs3231(out);
   }

   bool rtcSet(const RtcTime &in) {
      if (!s_rtcPresent) return false;
      return rtcSetDs3231(in);
   }

   bool rtcReadDailyBackwashStamp(RtcTime &out) {
      if (!s_rtcPresent) return false;
      return rtcReadAlarm1Stamp(out);
   }

   bool rtcWriteDailyBackwashStamp(const RtcTime &in) {
      if (!s_rtcPresent) return false;
      return rtcWriteAlarm1Stamp(in);
   }

   bool rtcResetDailyBackwashStamp() {
      if (!s_rtcPresent) return false;
      RtcTime t{};
      t.year = 2000;
      t.month = 1;
      t.day = 1;
      t.dow = calcDowDs3231(t);
      t.hour = 0;
      t.minute = 0;
      t.second = 0;
      return rtcWriteAlarm1Stamp(t);
   }

   bool rtcReadTriggeredBackwashStamp(RtcTime &out) {
      if (!s_rtcPresent) return false;
      return rtcReadAlarm2Stamp(out);
   }

   bool rtcWriteTriggeredBackwashStamp(const RtcTime &in) {
      if (!s_rtcPresent) return false;
      return rtcWriteAlarm2Stamp(in);
   }

   bool rtcResetTriggeredBackwashStamp() {
      if (!s_rtcPresent) return false;
      RtcTime t{};
      t.year = 2000;
      t.month = 1;
      t.day = 1;
      t.dow = calcDowDs3231(t);
      t.hour = 0;
      t.minute = 0;
      t.second = 0;
      return rtcWriteAlarm2Stamp(t);
   }

   // -------------------- Button edge flags (PinChangeInterrupt) --------------------
   static volatile uint8_t s_btnPressedMask = 0;
   static volatile uint8_t s_btnReleasedMask = 0;

   static inline void handleButtonEdge(uint8_t pin, uint8_t bit) {
      const bool isPressed = (digitalRead(pin) == LOW); // buttons are active-low
      if (isPressed) s_btnPressedMask |= bit;
      else s_btnReleasedMask |= bit;
   }

   static void isrBtnFrontDisp() { handleButtonEdge(KioskPins::BTN_FRONT_DISPENSE, (uint8_t)(1u << BtnFrontDisp)); }
   static void isrOzoneCtl()      { handleButtonEdge(KioskPins::BTN_OZONE_CTL, (uint8_t)(1u << BtnOzoneCtl)); }
   static void isrBackwashCtl()   { handleButtonEdge(KioskPins::BTN_BACKWASH_CTL, (uint8_t)(1u << BtnBackwashCtl)); }
   static void isrSensorBypass()  { handleButtonEdge(KioskPins::BTN_SENSOR_BYPASS, (uint8_t)(1u << BtnSensorBypass)); }
   static void isrContCirc()      { handleButtonEdge(KioskPins::BTN_CONT_CIRC, (uint8_t)(1u << BtnContCirc)); }
   static void isrWaterInlet()    { handleButtonEdge(KioskPins::BTN_WATER_INLET, (uint8_t)(1u << BtnWaterInlet)); }
   static void isrContDisp()      { handleButtonEdge(KioskPins::BTN_CONT_DISP, (uint8_t)(1u << BtnContDisp)); }
   static void isrSingleDisp()    { handleButtonEdge(KioskPins::BTN_SINGLE_DISP, (uint8_t)(1u << BtnSingleDisp)); }

   // -------------------- Flow pulse counters (PinChangeInterrupt) --------------------
   static volatile uint32_t s_dispenseFlowPulses = 0;
   static volatile uint32_t s_inletFlowPulses = 0;

   static void isrDispenseFlowPulse() { s_dispenseFlowPulses++; }
   static void isrInletFlowPulse() { s_inletFlowPulses++; }

   uint32_t dispenseFlowPulses() {
      uint32_t v;
      noInterrupts();
      v = s_dispenseFlowPulses;
      interrupts();
      return v;
   }

   uint32_t inletFlowPulses() {
      uint32_t v;
      noInterrupts();
      v = s_inletFlowPulses;
      interrupts();
      return v;
   }

   uint8_t getAndClearButtonEvent(ButtonId id)
   {
      const uint8_t bit = (uint8_t)(1u << id);
      uint8_t event = 0;
      noInterrupts();
      if (s_btnPressedMask & bit) event |= 0x1;
      if (s_btnReleasedMask & bit) event |= 0x2;
      s_btnPressedMask &= (uint8_t)~bit;
      s_btnReleasedMask &= (uint8_t)~bit;
      interrupts();
      return event;
   }

   void getAndClearButtonEdges(ButtonId id, ButtonEdges &out)
   {
      const uint8_t bit = (uint8_t)(1u << id);
      noInterrupts();
      out.pressed = (s_btnPressedMask & bit) != 0;
      out.released = (s_btnReleasedMask & bit) != 0;
      s_btnPressedMask &= (uint8_t)~bit;
      s_btnReleasedMask &= (uint8_t)~bit;
      interrupts();
   }

   // -------------------- Tank water level --------------------
   uint8_t tankWaterLevel() {
      // Read in descending order: FULL -> MID -> LOW.
      // Note: FULL level sensor is ACTIVE-LOW (asserted when pin reads LOW).
      if (digitalRead(KioskPins::IN_WATER_LEVEL_FULL) == LOW) return 3;
      if (readPin(KioskPins::IN_WATER_LEVEL_MED))             return 2;
      if (readPin(KioskPins::IN_WATER_LEVEL_LOW))             return 1;
      return 0;
   }

   // -------------------- Water path actuators --------------------
   // Basic on/off setters plus inlet kick->hold profile behavior.

   static bool s_backwashEnable      = false;
   static bool s_sensorBypassEnable  = false;
   static bool s_dispenseEnable      = false;
   static bool s_circulationEnable   = false;
   // Inlet controller state
   enum : uint8_t {
      INLET_OFF  = 0,
      INLET_KICK = 1,
      INLET_HOLD = 2
   };

   static bool     s_inletDesired = false;
   static uint8_t  s_inletPhase   = INLET_OFF;
   static uint32_t s_inletT0ms    = 0;     // kick start time (also used to time kick->hold)
   static uint8_t  s_inletKickPwm = 255;   // kick duty (0..255)
   static uint8_t  s_inletHoldPwm = 120;   // default hold duty (0..255)
   static uint32_t s_inletKickMs  = 2000UL; // default kick duration
   static uint32_t s_lastInletSetMs = 0;
   static uint8_t  s_lastInletOwner = (uint8_t)INLET_OWNER_UNKNOWN;
   static bool     s_inletOwnerConflict = false;

   static void applyBackwash() {
      if (s_backwashEnable) {
         analogWrite(KioskPins::PWM_BACKWASH_SOL, 255);
         writePin(KioskPins::OUT_BACKWASH_PUMP, true);
      } else {
         writePin(KioskPins::OUT_BACKWASH_PUMP, false);
         analogWrite(KioskPins::PWM_BACKWASH_SOL, 0);
      }
   }

   static void applySensorBypass() {
      analogWrite(KioskPins::PWM_SENSOR_BYP_SOL, s_sensorBypassEnable ? 255 : 0);
   }

   static void applyDispenseAndCirculate() {
      // Dispense solenoid only opens for dispense.
      analogWrite(KioskPins::PWM_WATERDISP_SOL, s_dispenseEnable ? 255 : 0);

      // Pump runs for dispense or circulation; solenoid remains closed for circulation-only.
      writePin(KioskPins::OUT_WATERDISP_PUMP, (s_dispenseEnable || s_circulationEnable));
   }

   static void inletRequest(bool enable, uint32_t now) {
      s_inletDesired = enable;

      if (!enable) {
         s_inletPhase = INLET_OFF;
         return;
      }

      if (s_inletPhase == INLET_OFF) {
         s_inletPhase = INLET_KICK;
         s_inletT0ms  = now; // kick start
      }
   }

static void inletService(uint32_t now) {
      // If not desired, force OFF.
      if (!s_inletDesired) s_inletPhase = INLET_OFF;

      if (s_inletPhase == INLET_OFF) {
         analogWrite(KioskPins::PWM_INLET_SOL, 0);
         writePin(KioskPins::OUT_INLET_BOOST_PUMP, false);
         return;
      }

      // Kick -> Hold transition.
      if (s_inletPhase == INLET_KICK) {
         if ((uint32_t)(now - s_inletT0ms) >= s_inletKickMs) s_inletPhase = INLET_HOLD;
      }

      // Drive outputs.
      writePin(KioskPins::OUT_INLET_BOOST_PUMP, true);
      analogWrite(KioskPins::PWM_INLET_SOL, (s_inletPhase == INLET_KICK) ? s_inletKickPwm : s_inletHoldPwm);
   }

void setBackwash(bool enable) {
      s_backwashEnable = enable;
      applyBackwash();
   }

   void setSensorBypass(bool enable) {
      s_sensorBypassEnable = enable;
      applySensorBypass();
   }

   void setWaterDispense(bool enable) {
      s_dispenseEnable = enable;
      applyDispenseAndCirculate();
   }

   void setWaterCirculation(bool enable) {
      s_circulationEnable = enable;
      applyDispenseAndCirculate();
   }

   void setKlaranUv(bool enable) {
      writePin(KioskPins::OUT_KLARAN_UV, enable);
   }

   bool klaranUvOk() {
      return readPin(KioskPins::IN_KLARAN_UV_OK);
   }

   void setAcousticAlert(bool enable) {
      // Acoustic alert drive is mapped to the OzoneGen output.
      writePin(KioskPins::OUT_OZONE, enable);
   }

   void setWaterInlet(bool enable, uint32_t nowMs, InletOwner owner) {
      const uint8_t ownerU8 = (uint8_t)owner;
      if (s_lastInletSetMs == nowMs &&
          s_lastInletOwner != (uint8_t)INLET_OWNER_UNKNOWN &&
          ownerU8 != (uint8_t)INLET_OWNER_UNKNOWN &&
          s_lastInletOwner != ownerU8) {
         s_inletOwnerConflict = true;
         Serial.print(F("DBG: inlet owner conflict ms="));
         Serial.print(nowMs);
         Serial.print(F(" prev="));
         Serial.print(s_lastInletOwner);
         Serial.print(F(" cur="));
         Serial.println(ownerU8);
      }
      s_lastInletSetMs = nowMs;
      s_lastInletOwner = ownerU8;
      inletRequest(enable, nowMs);
      inletService(nowMs);
   }

   void setWaterInlet(bool enable, uint32_t nowMs) {
      setWaterInlet(enable, nowMs, INLET_OWNER_UNKNOWN);
   }

   void setWaterInlet(bool enable, InletOwner owner) {
      const uint32_t nowMs = (uint32_t)millis();
      setWaterInlet(enable, nowMs, owner);
   }

   void setWaterInlet(bool enable) {
      const uint32_t nowMs = (uint32_t)millis();
      setWaterInlet(enable, nowMs, INLET_OWNER_UNKNOWN);
   }

   void setWaterInletKickPwm(uint8_t kickPwm) {
      s_inletKickPwm = kickPwm;
   }

   void setWaterInletHoldPwm(uint8_t holdPwm) {
      s_inletHoldPwm = holdPwm;
   }

   void setWaterInletKickMs(uint32_t kickMs) {
      s_inletKickMs = kickMs;
   }

   uint8_t waterInletPhase() {
      return s_inletPhase;
   }

   bool waterInletOwnerConflict() {
      return s_inletOwnerConflict;
   }

// -------------------- NFC (PN532) --------------------
   static Adafruit_PN532 s_nfc(KioskPins::PN532_IRQ, KioskPins::PN532_RESET);

   static constexpr uint8_t  NFC_API_VERSION  = 1;
   static constexpr uint8_t  NFC_CREDIT_UNITS = 1;

   static uint8_t NFC_SELECT_APDU[] = {
      0x00, 0xA4, 0x04, 0x00, 0x08,
      0x25, 0x2A, 0x59, 0xBF, 0x00,
      0x31, 0xE7, 0xC9, 0x00
   };

   static constexpr unsigned long NFC_PHONE_HOLDOFF_MS = 10000UL; // 10 s
   static constexpr unsigned long NFC_UID_HOLDOFF_MS   = 5000UL;  // 5 s

   static uint32_t      s_lastUsedTagHash = 0;
   static unsigned long s_lastUidUsedAt   = 0;
   static unsigned long s_lastCreditTime  = 0;
   static bool          s_nfcEnabled      = false;

   // -------------------- EEPROM token registry --------------------
   // Set this to match your EEPROM layout (must not overlap other data).
   // Prefer to put it near the end of EEPROM used-space.
   static constexpr int EEPROM_NFC_BASE = 0x100;

   static constexpr uint16_t NFC_EE_MAGIC   = 0x4E46; // 'N''F'
   static constexpr uint8_t  NFC_EE_VERSION = 1;
   static constexpr uint8_t  NFC_EE_MAX_TOKENS = 64;

   struct NfcEeHeader {
      uint16_t magic;
      uint8_t  version;
      uint8_t  count;
   };

   static inline uint32_t fnv1a32(const uint8_t* data, uint8_t len) {
      uint32_t h = 2166136261UL;
      for (uint8_t i = 0; i < len; i++) {
         h ^= data[i];
         h *= 16777619UL;
      }
      return h;
   }

   static bool nfcEeReadHeader(NfcEeHeader &hdr) {
      EEPROM.get(EEPROM_NFC_BASE, hdr);

      if (hdr.magic != NFC_EE_MAGIC) return false;
      if (hdr.version != NFC_EE_VERSION) return false;
      if (hdr.count > NFC_EE_MAX_TOKENS) return false;
      return true;
   }

   bool nfcRegistryIsValid() {
      NfcEeHeader hdr;
      return nfcEeReadHeader(hdr);
   }

   uint8_t nfcRegistryCount() {
      NfcEeHeader hdr;
      if (!nfcEeReadHeader(hdr)) return 0;
      return hdr.count;
   }

   static bool nfcEeContainsHash(uint32_t hash) {
      NfcEeHeader hdr;
      if (!nfcEeReadHeader(hdr) || hdr.count == 0) return false;

      const int hashesBase = EEPROM_NFC_BASE + (int)sizeof(NfcEeHeader);

      // Linear scan: fast enough for <=64 entries.
      for (uint8_t i = 0; i < hdr.count; i++) {
         uint32_t stored = 0;
         EEPROM.get(hashesBase + (int)(i * sizeof(uint32_t)), stored);
         if (stored == hash) return true;
      }
      return false;
   }

   // -------------------- Lifecycle --------------------
   void begin() {
      // Canonical pin initialisation from KioskIOpins.h.
      KioskPins::initPins();

      // Probe for RTC presence (I2C). Wire is expected to be initialized by host.
      rtcBegin();

      // Timer5 10ms tick for ISR-driven LED waveform updates.
      setupTimer5_10ms();

      // Attach pin-change interrupts for buttons (active-low).
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_FRONT_DISPENSE), isrBtnFrontDisp, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_OZONE_CTL), isrOzoneCtl, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_BACKWASH_CTL), isrBackwashCtl, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_SENSOR_BYPASS), isrSensorBypass, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_CONT_CIRC), isrContCirc, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_WATER_INLET), isrWaterInlet, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_CONT_DISP), isrContDisp, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::BTN_SINGLE_DISP), isrSingleDisp, CHANGE);

      // Flow pulse counting (counts both edges).
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::IN_DISP_FLOW_PULSE), isrDispenseFlowPulse, CHANGE);
      attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(KioskPins::IN_INLET_FLOW_PULSE), isrInletFlowPulse, CHANGE);

      // Keep PN532 in reset; NFC scans will bring it up on demand.
      writePin(KioskPins::PN532_RESET, false);
      s_nfcEnabled = false;
   }

   void service(unsigned long now) {
      // Time-based I/O housekeeping (currently inlet kick->hold progression).
      inletService((uint32_t)now);
      // Re-apply basic actuator outputs (idempotent).
      applyBackwash();
      applySensorBypass();
      applyDispenseAndCirculate();
   }

   // -------------------- NFC API --------------------
   void setNfcEnabled(bool enable) {
      if (s_nfcEnabled == enable) return;
      s_nfcEnabled = enable;
      if (!enable) {
         // Force PN532 back into reset when disabled.
         writePin(KioskPins::PN532_RESET, false);
      }
   }

   bool nfcEnabled() {
      return s_nfcEnabled;
   }

   int readNfc(uint32_t &lastSeenHash) {
      lastSeenHash = 0;
      if (!s_nfcEnabled) return 0;

      // Bring PN532 out of reset and initialize for this scan cycle.
      writePin(KioskPins::PN532_RESET, true);
      delay(100);
      s_nfc.begin();
      s_nfc.SAMConfig();

      struct ResetGuard {
         ~ResetGuard() { writePin(KioskPins::PN532_RESET, false); }
      } resetGuard;

      // Detect any passive target present
      if (!s_nfc.inListPassiveTarget()) return 0;

      const unsigned long now = millis();

      // 1) Try phone credit exchange first (holdoff protected)
      if ((now - s_lastCreditTime) >= NFC_PHONE_HOLDOFF_MS) {
         uint8_t apduResponse[10];
         uint8_t apduLen = sizeof(apduResponse);
         (void)s_nfc.inDataExchange(
            NFC_SELECT_APDU, sizeof(NFC_SELECT_APDU),
            apduResponse, &apduLen
         );

         uint8_t idsResponse[128];
         uint8_t idsLen = sizeof(idsResponse);

         uint8_t creditsSpent[3] = { 1, NFC_CREDIT_UNITS, NFC_API_VERSION };

         bool creditSuccess = s_nfc.inDataExchange(
            creditsSpent, sizeof(creditsSpent),
            idsResponse, &idsLen
         );

         if (creditSuccess) {
            s_lastCreditTime = now;
            return 1;
         }
      }

      // 2) Read tag UID, hash it
      uint8_t uid[7];
      uint8_t uidLen = 0;

      if (!s_nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 100)) {
         return 0;
      }

      const uint32_t hash = fnv1a32(uid, uidLen);
      lastSeenHash = hash;

      // 3) UID holdoff to suppress repeats
      if (hash == s_lastUsedTagHash && (now - s_lastUidUsedAt) < NFC_UID_HOLDOFF_MS) {
         return 0;
      }

      s_lastUsedTagHash = hash;
      s_lastUidUsedAt   = now;

      // 4) EEPROM-backed lookup
      if (nfcEeContainsHash(hash)) {
         return 2;
      }

      return 3;
   }

} // namespace KioskIO

ISR(TIMER5_COMPA_vect)
{
   static uint8_t tick = 0;
   static uint8_t lastBrightness = 255;

   if (!KioskIO::s_frontLedIsrEnabled) return;

   tick++;
   if (tick >= KioskIO::LED_WAVE_SAMPLES) tick = 0;

   KioskIO::LedMode mode = KioskIO::s_frontLedMode;
   if (mode >= KioskIO::LED_MODE_COUNT) mode = KioskIO::LED_OFF;
   const uint8_t brightness =
      (uint8_t)pgm_read_byte(&KioskIO::ledWaveforms[mode][tick]);

   if (brightness != lastBrightness) {
      analogWrite(KioskPins::PWM_DISP_BTTN_LED, brightness);
      lastBrightness = brightness;
   }
}
