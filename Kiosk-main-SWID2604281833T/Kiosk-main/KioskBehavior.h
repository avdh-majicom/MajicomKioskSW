// KioskBehavior.h
// -----------------------------------------------------------------------------
// High-level behavior glue that coordinates I/O and hydraulics.
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

namespace Kiosk {
   class KioskEeprom;
}

namespace KioskBehavior {

   enum BackwashEvent : uint8_t {
      BackwashNone     = 0,
      BackwashStartReq = 1,
      BackwashStopReq  = 2,
      BackwashIgnored  = 3
   };

   enum ErrorCode : uint16_t {
      ERR_NONE = 0,
      ERR_DISP_PROFILE_UNAVAILABLE = 100,
      ERR_UV_NOT_OK = 101
   };

   // Last payment result (also used for UI routing):
   //   0   = PaymentOverride
   //   1   = Coin
   //   2   = NFC Token
   //   3   = NFC App
   //   254 = Bad Coin
   //   255 = Rejected/Unknown Tag
   extern uint8_t LastPayment;

   struct NfcEvent {
      int result;       // matches KioskIO::readNfc() return codes
      uint32_t hash;    // UID hash (0 for phone credit)
      uint32_t timeMs;  // millis() at event
   };

   // Returns true if a new NFC event was captured since last call.
   bool popNfcEvent(NfcEvent& out);

   void setError(ErrorCode code);
   void raiseError(ErrorCode code);
   void clearError();
   bool errorActive();
   uint16_t errorCode();

   // Diagnostics/introspection for Kiosk Operations status rendering.
   bool sensorBypassActive();
   bool sensorBypassPending();
   uint32_t sensorBypassRemainingMs(uint32_t now);
   bool sensorBypassPeriodicActive();
   uint32_t sensorBypassPeriodicRemainingMs(uint32_t now);
   bool backwashPending();
   bool waterInletPending();
   bool waterInletHoldActive();
   uint32_t waterInletHoldElapsedMs(uint32_t now);
   bool waterInletAutoDemandActive();
   bool waterInletManualDemandActive();
   // Water circulation summary state used by the status screen.
   enum WaterCircStatus : uint8_t { WaterCircOff = 0, WaterCircAuto = 1, WaterCircOn = 2 };
   WaterCircStatus waterCircStatus();
   uint32_t waterCircDisplayRemainingMs(uint32_t now);
   void requestDayEndCircBwMode();
   bool dayEndModeActive();
   bool dayEndRecircActive();
   bool dayEndBackwashActive();
   // Waiting states mean DayEnd owns the phase but has not started output yet.
   bool dayEndRecircWaiting();
   bool dayEndBackwashWaiting();
   uint32_t dayEndRecircRemainingMs(uint32_t now);
   uint32_t dayEndBackwashRemainingMs(uint32_t now);
   bool periodicNozzleFlushPending();
   uint32_t periodicNozzleFlushPendingRemainingMs(uint32_t now);
   bool periodicNozzleFlushDispensing();
   uint32_t periodicNozzleFlushDispenseRemainingMs(uint32_t now);

   // Returns true if a valid payment was accepted. Gated by enable flags.
   bool KioskPayment(bool enableCoin, bool enableNfcToken, bool enableNfcApp,
                     Kiosk::KioskEeprom* eeprom);

   // Apply the hydraulics policy for this loop; returns any backwash event.
   BackwashEvent updateHydraulics(uint32_t now,
                                  bool inletTogglePulse,
                                  bool btnFrontDispPulse,
                                  bool rearDispensePulse,
                                  bool contDispPressPulse,
                                  bool contDispDown,
                                  bool contDispReleasePulse,
                                  uint16_t contDispLastPressMs,
                                  bool sensorBypassTogglePulse,
                                  uint16_t sensorBypassLastPressMs,
                                  bool backwashReqPulse,
                                  uint16_t backwashLastPressMs,
                                  bool uiWelcomeActive,
                                  bool darkModeActive,
                                  Kiosk::KioskEeprom* eeprom);

   namespace Ui {

      // Semantic UI states. Map to UI_* names via docStateName().
      enum UiState : uint8_t {
         UI_BOOT,
         UI_WELCOME,
         UI_PAY_PROMPT,
         UI_PAY_FAILED,
         UI_PAY_COIN_OK,
         UI_PAY_APP_OK,
         UI_PAY_TOKEN_OK,
         UI_PAY_TOKEN_BAD,
         UI_OVERRIDE,
         UI_DARK_MODE_IN,
         UI_DARK_MODE_OUT,
         UI_DISP_RDY,
         UI_DISPENSING,
         UI_DISP_DONE,
         UI_TIMEOUT,
         UI_OUT_OF_SERVICE,
         UI_ERROR
      };

      // LED/backlight intent: the caller decides how to drive actual hardware.
      enum LedMode : uint8_t { LED_OFF = 0, LED_ON = 1, LED_PULSE = 2 };
      enum BacklightMode : uint8_t { BL_OFF = 0, BL_ON = 1, BL_FLICK_ON_INTRO = 2 };

      struct UiInputs {
         bool btnFrontDispPressed;
         bool btnFrontDispDown;
         bool btnRearDispPressed;
         bool dispenseNotOk;
         bool filterBackwashActive;
         bool hasCoinAcceptor;
         bool hasNfc;
      };

      struct UiOutputs {
         LedMode btnFrontDispLED;
         LedMode dispenseButtonLed;
         BacklightMode backlight;
         bool requestDispensePulse;
         bool requestReset;
         bool lcdDirty;
         char line0[17];
         char line1[17];
         char line2[17];
         char line3[17];
      };

      struct UiContext {
         UiState state;
         UiState lastRendered;
         uint32_t stateStartMs;
         uint32_t paymentStartMs;
         uint32_t dispenseStartMs;
         uint32_t lastProgressMs;
         uint8_t lastProgressBars;
         uint8_t lastProgressPct;
         bool dispensePulseArmed;
         bool countedThisState;
         bool welcomePressArmed;
         bool darkModeArmed;
         bool darkModeSkipArmed;
         uint32_t darkModeHoldStartMs;
         int16_t darkModeLastRemain;
         Kiosk::KioskEeprom* eeprom;
      };

      void uiInit(UiContext& ctx, Kiosk::KioskEeprom* eeprom, uint32_t now);
      void uiUpdate(UiContext& ctx, const UiInputs& in, UiOutputs& out, uint32_t now);
      const char* docStateName(UiState st);
      bool parseDocStateName(const char* s, UiState& out);
      const char* uiStateId(UiState st);

   } // namespace Ui

} // namespace KioskBehavior
