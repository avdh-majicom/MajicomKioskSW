// KioskIO.h
// -----------------------------------------------------------------------------
// Kiosk I/O facade.
// - Owns all non-trivial I/O behavior (initialization, helpers, device services)
// - Iterates KioskPins::kInitTable[] from KioskIOpins.h to safely configure pins
// - Includes stable NFC support (PN532) with EEPROM-backed hash registry
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

namespace KioskIO {

   // -------------------- Lifecycle --------------------
   // Call once from setup().
   void begin();

   // Call from loop() with millis().
   // Currently used for optional time-based housekeeping.
   void service(unsigned long now);

   // -------------------- Basic digital IO helpers --------------------
   // These are thin wrappers intended to keep all pin access going through KioskIO.
   // They do NOT do debouncing.
   bool readPin(uint8_t pin);
   void writePin(uint8_t pin, bool high);

   // Convenience: configure a pin as output, glitch-minimized (LOW then OUTPUT).
   // Rarely needed if you always use the init table, but useful for dynamic cases.
   void safeMakeOutputLow(uint8_t pin);

   // Returns true when OVERRIDE_SWITCH input is HIGH.
   bool overrideActive();

   // -------------------- Button helpers (raw, no debounce) --------------------
   bool btnBackwashCtl();
   bool btnSensorBypass();
   bool btnContCirc();
   bool btnWaterInlet();
   bool btnContDisp();
   bool btnSingleDisp();
   bool btnOzoneCtl();
   bool btnFrontDisp();

   // -------------------- Button edge flags (PinChangeInterrupt) --------------------
   enum ButtonId : uint8_t {
      BtnFrontDisp = 0,
      BtnOzoneCtl,
      BtnBackwashCtl,
      BtnSensorBypass,
      BtnContCirc,
      BtnWaterInlet,
      BtnContDisp,
      BtnSingleDisp
   };

   struct ButtonEdges {
      bool pressed;
      bool released;
   };

   // Returns a 2-bit event code:
   // 0 = none, 1 = pressed, 2 = released, 3 = pressed+released
   uint8_t getAndClearButtonEvent(ButtonId id);

   // Returns and clears any pending edge flags for the given button.
   void getAndClearButtonEdges(ButtonId id, ButtonEdges &out);

   // -------------------- Tank water level --------------------
   // Returns:
   //   3 = FULL asserted (active-low sensor)
   //   2 = MID asserted
   //   1 = LOW asserted
   //   0 = none
   uint8_t tankWaterLevel();

   // -------------------- Flow pulse inputs --------------------
   // Returns cumulative pulse counts since boot (interrupt counted).
   uint32_t dispenseFlowPulses();
   uint32_t inletFlowPulses();

   // -------------------- RTC (I2C) --------------------
   // Abstracted RTC access; current implementation targets DS3231 over I2C.
   struct RtcTime {
      uint16_t year;  // full year (e.g., 2026)
      uint8_t  month; // 1..12
      uint8_t  day;   // 1..31
      uint8_t  dow;   // 1..7 (Sun..Sat)
      uint8_t  hour;  // 0..23
      uint8_t  minute;// 0..59
      uint8_t  second;// 0..59
   };

   // Probe for an RTC at boot; returns 1 if present, 0 if not.
   uint8_t rtcBegin();
   // True if the RTC was detected.
   bool rtcPresent();
   // Read RTC time into out; returns false if not present or read failed.
   bool rtcRead(RtcTime &out);
   // Set RTC time; returns false if not present or write failed.
   bool rtcSet(const RtcTime &in);
   // Alarm1 registers are repurposed as storage for the most recent daily backwash
   // timestamp (YY/MM/DD + HH:MM:SS with EEPROM-backed YY/MM metadata).
   // Alarm interrupts are disabled by firmware.
   bool rtcReadDailyBackwashStamp(RtcTime &out);
   bool rtcWriteDailyBackwashStamp(const RtcTime &in);
   // Reset Alarm1-stored daily backwash stamp to 2000-01-01 00:00:00.
   bool rtcResetDailyBackwashStamp();
   // Alarm2 registers are repurposed as storage for the most recent manually or
   // dispense-count-triggered backwash timestamp (YY/MM/DD + HH:MM with EEPROM-backed YY/MM metadata).
   bool rtcReadTriggeredBackwashStamp(RtcTime &out);
   bool rtcWriteTriggeredBackwashStamp(const RtcTime &in);
   // Reset Alarm2-stored backwash trigger stamp to 2000-01-01 00:00.
   bool rtcResetTriggeredBackwashStamp();

   // -------------------- Front Button LED Ring (Timer5 ISR waveform) --------------------
   enum LedMode : uint8_t {
      LED_OFF = 0,
      LED_BEACON = 1,
      LED_FLASH = 2,
      LED_ON = 3,
      LED_MODE_COUNT
   };

   // Enable/disable ISR-driven LED updates (default disabled).
   void enableBtnFrontDispLEDIsr(bool enable);
   // Set LED waveform mode (also enables ISR-driven control).
   void setBtnFrontDispLEDMode(LedMode mode);
   LedMode btnFrontDispLEDMode();

   // -------------------- Water path actuators (basic setters) --------------------
   // These are simple on/off setters. The inlet solenoid includes an internal
   // kick->hold profile (both PWM levels and kick duration are configurable).
   enum InletOwner : uint8_t {
      INLET_OWNER_UNKNOWN = 0,
      INLET_OWNER_BEHAVIOR = 1,
      INLET_OWNER_HYDRAULICS = 2
   };
   void setBackwash(bool enable);
   void setSensorBypass(bool enable);
   void setWaterInlet(bool enable);
   void setWaterInlet(bool enable, uint32_t nowMs);
   void setWaterInlet(bool enable, InletOwner owner);
   void setWaterInlet(bool enable, uint32_t nowMs, InletOwner owner);
   void setWaterDispense(bool enable);
   void setWaterCirculation(bool enable);
   // Klaran UV output and health input (UV_OK).
   void setKlaranUv(bool enable);
   bool klaranUvOk();
   // Acoustic alert output used by Periodic Nozzle Flush pre-alert.
   void setAcousticAlert(bool enable);
   // Inlet solenoid tuning (only affects setWaterInlet / inlet behavior)
   // kickPwm/holdPwm: 0..255
   // kickMs: kick duration in milliseconds before transitioning to HOLD.
   void setWaterInletKickPwm(uint8_t kickPwm);
   void setWaterInletHoldPwm(uint8_t holdPwm);
   void setWaterInletKickMs(uint32_t kickMs);
   // 0=OFF, 1=KICK, 2=HOLD
   uint8_t waterInletPhase();
   bool waterInletOwnerConflict();

// -------------------- NFC --------------------
   // Returns:
   //   0 = no tag / nothing
   //   1 = phone credit success
   //   2 = accepted tag (UID hash found in EEPROM registry)
   //   3 = rejected/unknown tag
   int readNfc(uint32_t &lastSeenHash);
   // Enable/disable NFC scans (disabling forces PN532 reset low).
   void setNfcEnabled(bool enable);
   bool nfcEnabled();

   // Optional: allow your EEPROM editor / tools to validate the token registry.
   // (Not required for normal kiosk operation.)
   bool nfcRegistryIsValid();
   uint8_t nfcRegistryCount();

} // namespace KioskIO
