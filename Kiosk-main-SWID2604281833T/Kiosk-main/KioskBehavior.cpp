// KioskBehavior.cpp
// -----------------------------------------------------------------------------

#include "KioskBehavior.h"
#include "KioskIO.h"
#include "KioskHydraulics.h"
#include "KioskEeprom.h"
#include <string.h>
#include <stdio.h>

namespace KioskBehavior {

   // Shared behavior state -----------------------------------------------------
   uint8_t LastPayment = 255;
   static uint32_t s_lastNfcHash = 0;
   static int s_lastNfcResult = 0;
   static uint32_t s_lastNfcEventMs = 0;
   static bool s_nfcEventPending = false;
   static uint16_t s_errorCode = ERR_NONE;

   static bool s_autoRefillLatch = false;
   static bool s_manualInletOn = false;
   static bool s_inletStartupPrimed = false;
   static bool s_inletHoldActive = false;
   static uint32_t s_inletHoldStartMs = 0;
   static uint32_t s_inletHoldUntilMs = 0;
   static constexpr uint32_t INLET_HOLD_MAX_MS = 60UL * 60UL * 1000UL;
   static constexpr uint32_t DAILY_BW_RTC_POLL_MS = 5000UL;
   static bool s_sensorBypass = false;
   static bool s_sensorBypassPendingToggle = false;
   static uint32_t s_sensorBypassPendingDurationMs = 0;
   static bool s_sensorBypassAutoPendingStart = false;
   static uint32_t s_sensorBypassUntilMs = 0;
   static uint32_t s_sensorBypassAutoNextMs = 0;
   static uint8_t s_sensorBypassAutoPeriodMin = 0;
   static bool s_backwashPendingUntilBypassClear = false;
   static uint32_t s_backwashPendingDurationMs = 0;
   static bool s_waterCircActive = false;
   static uint32_t s_waterCircUntilMs = 0;
   static bool s_waterCircPausedByDispense = false;
   static uint32_t s_waterCircPauseStartMs = 0;
   static bool s_waterCircBtnTracking = false;
   static uint32_t s_waterCircBtnPressMs = 0;
   static uint8_t s_autoCircDurMin = 0;
   static uint8_t s_autoCircPeriod10Min = 0;
   static uint32_t s_autoCircNextMs = 0;
   static bool s_autoBackwashReqPending = false;
   static uint32_t s_lastRtcPollMs = 0;
   static uint32_t s_lastDailyBackwashDate = 0;
   static bool s_dailyBackwashTriggeredThisLoop = false;
   enum DayEndPhase : uint8_t { DayEndNone = 0, DayEndRecirc = 1, DayEndBackwash = 2 };
   static bool s_dayEndReqPending = false;
   static DayEndPhase s_dayEndPhase = DayEndNone;
   static uint32_t s_dayEndCircDurationMs = 0;
   static bool s_dayEndCircStarted = false;
   static uint32_t s_dayEndBwDurationMs = 0;
   static bool s_dayEndBwStarted = false;
   enum PeriodicFlushPhase : uint8_t {
      PeriodicFlushIdle = 0,
      PeriodicFlushPreDelay,
      PeriodicFlushBeepOn,
      PeriodicFlushBeepOff,
      PeriodicFlushAwaitDispenseStart,
      PeriodicFlushAwaitDispenseDone
   };
   static PeriodicFlushPhase s_periodicFlushPhase = PeriodicFlushIdle;
   static uint32_t s_periodicFlushDueMs = 0;
   static uint32_t s_periodicFlushStepUntilMs = 0;
   static uint32_t s_periodicFlushPreDelayMs = 0;
   static uint32_t s_periodicFlushBeepOnMs = 0;
   static uint32_t s_periodicFlushBeepPeriodMs = 0;
   static uint32_t s_periodicFlushBeepStartMs = 0;
   static uint8_t s_periodicFlushBeepsRemaining = 0;
   static uint32_t s_periodicFlushDispenseMs = 0;
   static uint32_t s_periodicFlushNextDueMs = 0;
   static bool s_periodicFlushDispenseInFlight = false;
   static uint32_t s_periodicFlushDispenseStartMs = 0;
   static uint32_t s_periodicFlushDispenseDurationMs = 0;
   static bool s_prevUiWelcomeActive = false;

   static bool shouldRequestDailyBackwash(uint32_t now,
                                          uint8_t waterLevel,
                                          bool overrideActive,
                                          bool backwashActiveBefore,
                                          bool sensorBypassActive,
                                          Kiosk::KioskEeprom* eeprom,
                                          uint32_t& durationMs)
   {
      durationMs = 0;
      if (!eeprom || !eeprom->isReady()) return false;
      if (!KioskIO::rtcPresent()) return false;

      const uint8_t dur10s = eeprom->dailyBackwashDuration10s();
      if (dur10s == 0) return false;

      if ((uint32_t)(now - s_lastRtcPollMs) < DAILY_BW_RTC_POLL_MS) return false;
      s_lastRtcPollMs = now;

      KioskIO::RtcTime rtc{};
      if (!KioskIO::rtcRead(rtc)) return false;

      const uint32_t todayKey =
         (uint32_t)rtc.year * 10000UL + (uint32_t)rtc.month * 100UL + (uint32_t)rtc.day;
      if (s_lastDailyBackwashDate == todayKey) return false;

      const uint16_t nowMinutes = (uint16_t)rtc.hour * 60U + (uint16_t)rtc.minute;
      const uint16_t schedMinutes = eeprom->dailyBackwashTimeMinutes();
      if (nowMinutes < schedMinutes) return false;
      // Daily RTC backwash is only eligible in the first hour after the configured time.
      if ((uint16_t)(nowMinutes - schedMinutes) >= 60U) return false;

      if (backwashActiveBefore) return false;
      if (sensorBypassActive) return false;
      if ((!overrideActive && waterLevel <= 1) || (overrideActive && waterLevel == 0)) return false;

      s_lastDailyBackwashDate = todayKey;
      // Persist the latest daily backwash timestamp for operator diagnostics.
      (void)KioskIO::rtcWriteDailyBackwashStamp(rtc);
      s_dailyBackwashTriggeredThisLoop = true;
      durationMs = (uint32_t)dur10s * 10UL * 1000UL;
      return true;
   }

   void setError(ErrorCode code) {
      s_errorCode = code;
   }

   void raiseError(ErrorCode code) {
      if (code == ERR_NONE) return;
      s_errorCode = code;
   }

   void clearError() {
      s_errorCode = ERR_NONE;
   }

   bool errorActive() {
      return s_errorCode != ERR_NONE;
   }

   uint16_t errorCode() {
      return s_errorCode;
   }

   bool sensorBypassActive() {
      return s_sensorBypass;
   }

   bool sensorBypassPending() {
      return s_sensorBypassPendingToggle;
   }

   uint32_t sensorBypassRemainingMs(uint32_t now) {
      if (!s_sensorBypass) return 0;
      if ((int32_t)(s_sensorBypassUntilMs - now) <= 0) return 0;
      return (uint32_t)(s_sensorBypassUntilMs - now);
   }

   bool sensorBypassPeriodicActive() {
      return s_sensorBypassAutoPeriodMin > 0U;
   }

   uint32_t sensorBypassPeriodicRemainingMs(uint32_t now) {
      if (s_sensorBypassAutoPeriodMin == 0U) return 0;
      if ((int32_t)(s_sensorBypassAutoNextMs - now) <= 0) return 0;
      return (uint32_t)(s_sensorBypassAutoNextMs - now);
   }

   bool backwashPending() {
      return s_backwashPendingUntilBypassClear;
   }

   bool waterInletPending() {
      const uint8_t wl = KioskIO::tankWaterLevel();
      const bool inletDemand = (s_autoRefillLatch || s_manualInletOn) && !s_inletHoldActive && (wl < 3U);
      return inletDemand && (KioskHydraulics::backwashActive() || s_sensorBypass);
   }

   bool waterInletAutoDemandActive() {
      return s_autoRefillLatch && !s_inletHoldActive;
   }

   bool waterInletManualDemandActive() {
      return s_manualInletOn && !s_inletHoldActive;
   }

   bool waterInletHoldActive() {
      return s_inletHoldActive;
   }

   uint32_t waterInletHoldElapsedMs(uint32_t now) {
      if (!s_inletHoldActive) return 0;
      return (uint32_t)(now - s_inletHoldStartMs);
   }

   WaterCircStatus waterCircStatus() {
      if (s_waterCircActive) return WaterCircOn;
      if (s_autoCircDurMin > 0U && s_autoCircPeriod10Min > 0U) return WaterCircAuto;
      return WaterCircOff;
   }

   uint32_t waterCircDisplayRemainingMs(uint32_t now) {
      if (s_waterCircActive) {
         const uint32_t refNow = s_waterCircPausedByDispense ? s_waterCircPauseStartMs : now;
         if ((int32_t)(s_waterCircUntilMs - refNow) <= 0) return 0;
         return (uint32_t)(s_waterCircUntilMs - refNow);
      }
      if (s_autoCircDurMin > 0U && s_autoCircPeriod10Min > 0U) {
         if ((int32_t)(s_autoCircNextMs - now) <= 0) return 0;
         return (uint32_t)(s_autoCircNextMs - now);
      }
      return 0;
   }

   void requestDayEndCircBwMode() {
      if (!KioskIO::overrideActive()) return;
      s_dayEndReqPending = true;
   }

   bool dayEndModeActive() {
      return s_dayEndReqPending || (s_dayEndPhase != DayEndNone);
   }

   bool dayEndRecircActive() {
      return s_dayEndPhase == DayEndRecirc;
   }

   bool dayEndBackwashActive() {
      return s_dayEndPhase == DayEndBackwash;
   }

   bool dayEndRecircWaiting() {
      return (s_dayEndPhase == DayEndRecirc) && !s_dayEndCircStarted;
   }

   bool dayEndBackwashWaiting() {
      return (s_dayEndPhase == DayEndBackwash) && !s_dayEndBwStarted &&
             !KioskHydraulics::backwashActive();
   }

   uint32_t dayEndRecircRemainingMs(uint32_t now) {
      if (s_dayEndPhase != DayEndRecirc) return 0;
      if (!s_dayEndCircStarted) return 0;
      const uint32_t refNow = s_waterCircPausedByDispense ? s_waterCircPauseStartMs : now;
      if ((int32_t)(s_waterCircUntilMs - refNow) <= 0) return 0;
      return (uint32_t)(s_waterCircUntilMs - refNow);
   }

   uint32_t dayEndBackwashRemainingMs(uint32_t now) {
      if (s_dayEndPhase != DayEndBackwash) return 0;
      if (KioskHydraulics::backwashActive()) {
         return KioskHydraulics::backwashRemainingMs(now);
      }
      if (!s_dayEndBwStarted) return s_dayEndBwDurationMs;
      return 0;
   }

   bool periodicNozzleFlushPending() {
      return s_periodicFlushPhase == PeriodicFlushBeepOn ||
             s_periodicFlushPhase == PeriodicFlushBeepOff ||
             s_periodicFlushPhase == PeriodicFlushPreDelay ||
             s_periodicFlushPhase == PeriodicFlushAwaitDispenseStart;
   }

   uint32_t periodicNozzleFlushPendingRemainingMs(uint32_t now) {
      if (!periodicNozzleFlushPending()) return 0;
      const uint32_t targetMs = s_periodicFlushDueMs + s_periodicFlushPreDelayMs;
      if ((int32_t)(targetMs - now) <= 0) return 0;
      return (uint32_t)(targetMs - now);
   }

   bool periodicNozzleFlushDispensing() {
      return s_periodicFlushDispenseInFlight;
   }

   uint32_t periodicNozzleFlushDispenseRemainingMs(uint32_t now) {
      if (!s_periodicFlushDispenseInFlight) return 0;
      const uint32_t elapsed = (uint32_t)(now - s_periodicFlushDispenseStartMs);
      if (elapsed >= s_periodicFlushDispenseDurationMs) return 0;
      return (uint32_t)(s_periodicFlushDispenseDurationMs - elapsed);
   }

   bool KioskPayment(bool enableCoin, bool enableNfcToken, bool enableNfcApp,
                     Kiosk::KioskEeprom* eeprom) {
      // Override mode bypasses payment checks entirely.
      if (KioskIO::overrideActive()) {
         LastPayment = 0;
         return true;
      }

      (void)enableCoin; // Coin acceptor input is not wired in this module.

      s_lastNfcHash = 0;
      if (enableNfcToken || enableNfcApp) {
         uint32_t hash = 0;
         const int nfcRes = KioskIO::readNfc(hash);
         if (hash != 0) s_lastNfcHash = hash;
         if (nfcRes == 1 && enableNfcApp) {
            LastPayment = 3; // NFC app payment
            s_lastNfcResult = 1;
            s_lastNfcEventMs = millis();
            s_nfcEventPending = true;
            return true;
         }
         if ((nfcRes == 2 || nfcRes == 3) && hash != 0 && enableNfcToken) {
            bool tokenOk = false;
            if (eeprom && eeprom->isReady()) {
               tokenOk = eeprom->tokenHashExists(hash);
            } else {
               // Fallback to legacy NFC registry if EEPROM manager isn't ready.
               tokenOk = (nfcRes == 2);
            }
            if (tokenOk) {
               LastPayment = 2; // NFC token payment
               s_lastNfcResult = 2;
            } else {
               LastPayment = 255; // token bad
               s_lastNfcResult = 3;
            }
            s_lastNfcEventMs = millis();
            s_nfcEventPending = true;
            return tokenOk;
         }
      }

      return false;
   }

   bool popNfcEvent(NfcEvent& out) {
      if (!s_nfcEventPending) return false;
      out.result = s_lastNfcResult;
      out.hash = s_lastNfcHash;
      out.timeMs = s_lastNfcEventMs;
      s_nfcEventPending = false;
      return true;
   }

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
                                  Kiosk::KioskEeprom* eeprom) {
      BackwashEvent evt = BackwashNone;
      if (errorActive()) {
         // Fail-safe: clear all software latches and force every hydraulic output OFF.
         s_autoRefillLatch = false;
         s_manualInletOn = false;
         s_inletHoldActive = false;
         s_inletHoldStartMs = 0;
         s_inletHoldUntilMs = 0;
         s_sensorBypass = false;
         s_sensorBypassPendingToggle = false;
         s_sensorBypassPendingDurationMs = 0;
         s_sensorBypassAutoPendingStart = false;
         s_sensorBypassUntilMs = 0;
         s_sensorBypassAutoNextMs = 0;
         s_sensorBypassAutoPeriodMin = 0;
         s_backwashPendingUntilBypassClear = false;
         s_backwashPendingDurationMs = 0;
         s_waterCircActive = false;
         s_waterCircUntilMs = 0;
         s_waterCircPausedByDispense = false;
         s_waterCircPauseStartMs = 0;
         s_waterCircBtnTracking = false;
         s_waterCircBtnPressMs = 0;
         s_autoCircDurMin = 0;
         s_autoCircPeriod10Min = 0;
         s_autoCircNextMs = 0;
         s_autoBackwashReqPending = false;
         s_dayEndReqPending = false;
         s_dayEndPhase = DayEndNone;
         s_dayEndCircDurationMs = 0;
         s_dayEndCircStarted = false;
         s_dayEndBwDurationMs = 0;
         s_dayEndBwStarted = false;
         s_periodicFlushPhase = PeriodicFlushIdle;
         s_periodicFlushDueMs = 0;
         s_periodicFlushStepUntilMs = 0;
         s_periodicFlushPreDelayMs = 0;
         s_periodicFlushBeepOnMs = 0;
         s_periodicFlushBeepPeriodMs = 0;
         s_periodicFlushBeepStartMs = 0;
         s_periodicFlushBeepsRemaining = 0;
         s_periodicFlushDispenseMs = 0;
         s_periodicFlushNextDueMs = 0;
         s_periodicFlushDispenseInFlight = false;
         s_periodicFlushDispenseStartMs = 0;
         s_periodicFlushDispenseDurationMs = 0;
         s_prevUiWelcomeActive = false;
         KioskIO::setBackwash(false);
         KioskIO::setWaterInlet(false, now, KioskIO::INLET_OWNER_BEHAVIOR);
         KioskIO::setWaterDispense(false);
         KioskIO::setWaterCirculation(false);
         KioskIO::setSensorBypass(false);
         KioskIO::setAcousticAlert(false);
         return evt;
      }
      const bool overrideActive = KioskIO::overrideActive();
      KioskHydraulics::setOverrideActive(overrideActive);
      s_dailyBackwashTriggeredThisLoop = false;
      const bool dayEndActiveAtEntry = s_dayEndReqPending || (s_dayEndPhase != DayEndNone);
      const bool backwashActiveBefore = KioskHydraulics::backwashActive();
      const bool dispensingNow = (KioskHydraulics::state() != 0);
      bool dayEndCanceledDispenseThisLoop = false;
      const bool autoBackwashReq = s_autoBackwashReqPending;
      const uint8_t wl = KioskIO::tankWaterLevel();
      // One-shot startup prime:
      // - On first hydraulics update after boot, force inlet demand ON when level < 3.
      // - This ensures refill starts automatically from startup without waiting for operator input.
      if (!s_inletStartupPrimed) {
         s_inletStartupPrimed = true;
         if (wl < 3U) {
            s_manualInletOn = true;
            s_inletHoldActive = false;
            s_inletHoldStartMs = 0;
            s_inletHoldUntilMs = 0;
         }
      }
      const bool welcomeEntered = uiWelcomeActive && !s_prevUiWelcomeActive;
      const bool welcomeExited = !uiWelcomeActive && s_prevUiWelcomeActive;
      s_prevUiWelcomeActive = uiWelcomeActive;
      auto sensorBypassDurationMs = [&]() -> uint32_t {
         if (eeprom && eeprom->isReady()) {
            return (uint32_t)eeprom->sensorBypassDuration100ms() * 100UL;
         }
         return 5000UL;
      };
      auto manualSensorBypassDurationMs = [&]() -> uint32_t {
         static constexpr uint16_t SHORT_PRESS_THRESHOLD_MS = 2000U;
         static constexpr uint16_t PRESS_DURATION_OVERFLOW_MS = 65535U;
         uint8_t short100ms = 50U; // 5.0s
         uint8_t long10s = 6U;     // 60s
         if (eeprom && eeprom->isReady()) {
            short100ms = eeprom->sensorBypassManualShort100ms();
            long10s = eeprom->sensorBypassManualLong10s();
         }
         if (sensorBypassLastPressMs != PRESS_DURATION_OVERFLOW_MS &&
             sensorBypassLastPressMs <= SHORT_PRESS_THRESHOLD_MS) {
            return (uint32_t)short100ms * 100UL;
         }
         return (uint32_t)long10s * 10UL * 1000UL;
      };
      auto applySensorBypassState = [&](bool enable, uint32_t durationMs = 0) {
         s_sensorBypass = enable;
         if (enable) {
            const uint32_t durMs = (durationMs > 0) ? durationMs : sensorBypassDurationMs();
            s_sensorBypassUntilMs = now + durMs;
         } else {
            s_sensorBypassUntilMs = 0;
         }
         KioskIO::setSensorBypass(enable);
      };
      const uint8_t sensorBypassPeriodMin =
         (eeprom && eeprom->isReady()) ? eeprom->sensorBypassPeriodMinutes() : 0U;
      if (sensorBypassPeriodMin != s_sensorBypassAutoPeriodMin) {
         s_sensorBypassAutoPeriodMin = sensorBypassPeriodMin;
         s_sensorBypassAutoPendingStart = false;
         if (sensorBypassPeriodMin > 0U) {
            s_sensorBypassAutoNextMs = now + (uint32_t)s_sensorBypassAutoPeriodMin * 60000UL;
         } else {
            s_sensorBypassAutoNextMs = 0;
         }
      }
      if (s_sensorBypassAutoPeriodMin > 0U && (int32_t)(now - s_sensorBypassAutoNextMs) >= 0) {
         s_sensorBypassAutoPendingStart = true;
         do {
            s_sensorBypassAutoNextMs += (uint32_t)s_sensorBypassAutoPeriodMin * 60000UL;
         } while ((int32_t)(now - s_sensorBypassAutoNextMs) >= 0);
      }
      if (s_sensorBypass && (int32_t)(now - s_sensorBypassUntilMs) >= 0) {
         applySensorBypassState(false);
      }
      if (eeprom && eeprom->isReady()) {
         KioskHydraulics::setBackwashDefaultDurationMs(eeprom->backwashDurationMs());
      }
      auto manualBackwashDurationMs = [&]() -> uint32_t {
         static constexpr uint16_t SHORT_PRESS_THRESHOLD_MS = 2000U;
         static constexpr uint16_t PRESS_DURATION_OVERFLOW_MS = 65535U;
         uint8_t shortSec = 15U;
         uint8_t long2Min = 15U;
         if (eeprom && eeprom->isReady()) {
            shortSec = eeprom->backwashManualShortSeconds();
            long2Min = eeprom->backwashManualLong2Minutes();
         }
         if (backwashLastPressMs != PRESS_DURATION_OVERFLOW_MS &&
             backwashLastPressMs <= SHORT_PRESS_THRESHOLD_MS) {
            return (uint32_t)shortSec * 1000UL;
         }
         return (uint32_t)long2Min * 2UL * 60UL * 1000UL;
      };

      bool effContDispPressPulse = contDispPressPulse;
      bool effContDispDown = contDispDown;
      bool effContDispReleasePulse = contDispReleasePulse;

      if (s_dayEndReqPending) {
         s_dayEndReqPending = false;

         // DayEnd takes ownership of recirculation/backwash sequencing.
         // Cancel all existing circulation/backwash requests and active runs first.
         s_waterCircActive = false;
         s_waterCircUntilMs = 0;
         s_waterCircPausedByDispense = false;
         s_waterCircPauseStartMs = 0;
         s_waterCircBtnTracking = false;
         s_waterCircBtnPressMs = 0;

         s_backwashPendingUntilBypassClear = false;
         s_backwashPendingDurationMs = 0;
         s_autoBackwashReqPending = false;
         if (KioskHydraulics::state() != 0) {
            KioskHydraulics::abortDispense();
            dayEndCanceledDispenseThisLoop = true;
         }
         if (KioskHydraulics::backwashActive()) {
            KioskHydraulics::requestBackwash(); // second press semantics: stop
            if (evt == BackwashNone) evt = BackwashStopReq;
         }
         if (wl < 3U) {
            // On DayEnd entry, restore inlet demand so refill resumes immediately.
            s_manualInletOn = true;
            s_inletHoldActive = false;
            s_inletHoldStartMs = 0;
            s_inletHoldUntilMs = 0;
         }

         const uint32_t dayEndCircMs =
            (eeprom && eeprom->isReady())
               ? ((uint32_t)eeprom->eodCircDuration15s() * 15UL * 1000UL)
               : (30UL * 60UL * 1000UL);
         const uint32_t dayEndBwMs =
            (eeprom && eeprom->isReady())
               ? ((uint32_t)eeprom->eodBackwashDuration1s() * 1000UL)
               : (3UL * 60UL * 1000UL);

         s_dayEndCircDurationMs = dayEndCircMs;
         s_dayEndCircStarted = false;
         s_dayEndBwDurationMs = dayEndBwMs;
         s_dayEndBwStarted = false;

         if (dayEndCircMs > 0UL) {
            s_dayEndPhase = DayEndRecirc;
            if (wl > 1U) {
               s_dayEndCircStarted = true;
               s_waterCircActive = true;
               s_waterCircUntilMs = now + dayEndCircMs;
            }
         } else if (dayEndBwMs > 0UL) {
            s_dayEndPhase = DayEndBackwash;
            const bool bwEligibleNow = (wl > 1U);
            if (bwEligibleNow) {
               KioskHydraulics::requestBackwashWithDuration(dayEndBwMs);
               if (evt == BackwashNone) evt = BackwashStartReq;
            }
         } else {
            s_dayEndPhase = DayEndNone;
            s_dayEndCircDurationMs = 0;
         }
      }

      if (s_dayEndPhase != DayEndNone) {
         // Ignore manual circulation button inputs while DayEnd mode is active.
         effContDispPressPulse = false;
         effContDispDown = false;
         effContDispReleasePulse = false;
      }

      // Manual inlet OFF hold expires automatically after 1 hour.
      if (s_inletHoldActive && (int32_t)(now - s_inletHoldUntilMs) >= 0) {
         s_inletHoldActive = false;
      }
      // Auto-refill hysteresis:
      // - ON when level < 2.
      // - OFF when level > 2.
      if (wl < 2U) s_autoRefillLatch = true;
      else if (wl > 2U) s_autoRefillLatch = false;
      // Inlet is always forced OFF when water level > 2.
      if (wl > 2U) {
         s_manualInletOn = false;
      }

      // WATER_INLET button behavior:
      // - If currently demanding inlet, press enters OFF hold (up to 1 hour).
      // - If currently OFF/HOLD and level < 3, press forces manual inlet demand ON.
      // - In override mode, press can first terminate active SensorBypass/Backwash.
      if (inletTogglePulse) {
         bool suppressedByBackwashOrBypass = false;
         if (overrideActive) {
            if (s_sensorBypass) {
               suppressedByBackwashOrBypass = true;
               s_sensorBypassPendingToggle = false;
               s_sensorBypassPendingDurationMs = 0;
               applySensorBypassState(false);
            }
            if (backwashActiveBefore) {
               suppressedByBackwashOrBypass = true;
               KioskHydraulics::requestBackwash(); // second press semantics: stop
               if (evt == BackwashNone) evt = BackwashStopReq;
               s_backwashPendingUntilBypassClear = false;
               s_backwashPendingDurationMs = 0;
            }
         }

         // Override source rule: if inlet was being suppressed by Backwash and/or SensorBypass,
         // pressing BTN_WATER_INLET cancels suppression and re-enables inlet demand.
         if (overrideActive && suppressedByBackwashOrBypass && wl < 3U) {
            s_manualInletOn = true;
            s_inletHoldActive = false;
            s_inletHoldStartMs = 0;
            s_inletHoldUntilMs = 0;
         } else if (((s_autoRefillLatch || s_manualInletOn) && !s_inletHoldActive && (wl < 3U))) {
            s_inletHoldActive = true;
            s_inletHoldStartMs = now;
            s_inletHoldUntilMs = now + INLET_HOLD_MAX_MS;
         } else if (wl < 3U) {
            s_manualInletOn = true;
            s_inletHoldActive = false;
            s_inletHoldStartMs = 0;
            s_inletHoldUntilMs = 0;
         }
      }

      // Backwash request handling.
      // Override mode relaxes start eligibility from water level >1 to >0.
      if ((backwashReqPulse || autoBackwashReq) && s_dayEndPhase == DayEndNone) {
         const bool reqFromButton = backwashReqPulse;
         const uint32_t reqDurationMs = reqFromButton ? manualBackwashDurationMs() : 0UL;
         bool autoReqConsumed = false;
         if (KioskHydraulics::backwashActive()) {
            if (reqFromButton) {
               KioskHydraulics::requestBackwash();
               evt = BackwashStopReq;
               s_backwashPendingUntilBypassClear = false;
               s_backwashPendingDurationMs = 0;
            } else {
               // Already in backwash; consume pending auto request.
               autoReqConsumed = true;
            }
         } else if ((!overrideActive && wl > 1) || (overrideActive && wl > 0)) {
            if (s_sensorBypass) {
               if (reqFromButton && overrideActive) {
                  // Source override: terminate active SensorBypass and start Backwash.
                  s_sensorBypassPendingToggle = false;
                  applySensorBypassState(false);
                  KioskHydraulics::requestBackwashWithDuration(reqDurationMs);
                  if (evt == BackwashNone) evt = BackwashStartReq;
                  s_backwashPendingUntilBypassClear = false;
                  s_backwashPendingDurationMs = 0;
                  autoReqConsumed = !reqFromButton;
               } else {
                  // Hold off until SensorBypass clears.
                  s_backwashPendingUntilBypassClear = true;
                  s_backwashPendingDurationMs = reqDurationMs;
                  if (reqFromButton && evt == BackwashNone) evt = BackwashIgnored;
                  autoReqConsumed = !reqFromButton;
               }
            } else {
               if (reqFromButton) KioskHydraulics::requestBackwashWithDuration(reqDurationMs);
               else KioskHydraulics::requestBackwash();
               if (evt == BackwashNone) evt = BackwashStartReq;
               autoReqConsumed = !reqFromButton;
            }
         } else {
            if (reqFromButton && evt == BackwashNone) evt = BackwashIgnored;
         }
         if (autoBackwashReq && autoReqConsumed) s_autoBackwashReqPending = false;
      }

      uint32_t dailyBwDurationMs = 0;
      if (shouldRequestDailyBackwash(now, wl, overrideActive, backwashActiveBefore, s_sensorBypass, eeprom, dailyBwDurationMs)) {
         KioskHydraulics::requestBackwashWithDuration(dailyBwDurationMs);
         if (evt == BackwashNone) evt = BackwashStartReq;
      }

      // SENSOR_BYPASS button:
      // - Normal mode: hold off while Backwash is active/starting.
      // - Override source: if Backwash active, terminate it; then toggle bypass.
      if (sensorBypassTogglePulse) {
         if (s_sensorBypassAutoPeriodMin > 0U) {
            // Button-triggered SensorBypass interaction restarts the periodic schedule.
            s_sensorBypassAutoNextMs = now + (uint32_t)s_sensorBypassAutoPeriodMin * 60000UL;
            s_sensorBypassAutoPendingStart = false;
         }
         const uint32_t reqDurationMs = manualSensorBypassDurationMs();
         if (backwashActiveBefore) {
            if (overrideActive) {
               KioskHydraulics::requestBackwash(); // second press semantics: stop
               if (evt == BackwashNone) evt = BackwashStopReq;
               s_backwashPendingUntilBypassClear = false;
               s_backwashPendingDurationMs = 0;
               s_sensorBypassPendingToggle = false;
               s_sensorBypassPendingDurationMs = 0;
               if (s_sensorBypass) applySensorBypassState(false);
               else applySensorBypassState(true, reqDurationMs);
            } else {
               s_sensorBypassPendingToggle = true;
               s_sensorBypassPendingDurationMs = s_sensorBypass ? 0UL : reqDurationMs;
            }
         } else if (evt == BackwashStartReq) {
            s_sensorBypassPendingToggle = true;
            s_sensorBypassPendingDurationMs = s_sensorBypass ? 0UL : reqDurationMs;
         } else {
            if (s_sensorBypass) applySensorBypassState(false);
            else applySensorBypassState(true, reqDurationMs);
         }
      }
      // Periodic SensorBypass auto-start:
      // - Uses SensByp Period (minutes) when > 0.
      // - Starts only when bypass is currently OFF.
      // - If Backwash is active/starting, hold pending until it clears.
      if (s_sensorBypassAutoPendingStart && !s_sensorBypass) {
         if (!backwashActiveBefore && evt != BackwashStartReq) {
            applySensorBypassState(true);
            s_sensorBypassAutoPendingStart = false;
         }
      }

      // If a backwash start was held off due to active sensor bypass, start it once bypass clears.
      const bool bwEligibleNow = ((!overrideActive && wl > 1) || (overrideActive && wl > 0));
      if (s_backwashPendingUntilBypassClear && !backwashActiveBefore && !s_sensorBypass && bwEligibleNow) {
         if (s_backwashPendingDurationMs > 0) KioskHydraulics::requestBackwashWithDuration(s_backwashPendingDurationMs);
         else KioskHydraulics::requestBackwash();
         if (evt == BackwashNone) evt = BackwashStartReq;
         s_backwashPendingUntilBypassClear = false;
         s_backwashPendingDurationMs = 0;
      }

      // Water circulation controls.
      // - Auto settings: duration (minutes) + period (10-minute units), both must be >0.
      // - Manual short/long settings: 1-minute / 10-minute units.
      // - BTN_CONT_CIRC press starts circulation immediately (if wl > 0).
      // - On BTN_CONT_CIRC release:
      //    * hold <= 2s => short profile
      //    * hold > 2s  => long profile
      //   Remaining runtime is profile_target - hold_duration.
      // - Pressing BTN_CONT_CIRC while already active cancels circulation and
      //   restarts the auto-circulation schedule window.
      uint8_t autoDurMin = 0;
      uint8_t autoPeriod10Min = 0;
      uint8_t manShortMin = 1;
      uint8_t manLong10Min = 1;
      if (eeprom && eeprom->isReady()) {
         autoDurMin = eeprom->autoCircDurationMinutes();
         autoPeriod10Min = eeprom->autoCircPeriod10Minutes();
         manShortMin = eeprom->manCircDurationShortMinutes();
         manLong10Min = eeprom->manCircDurationLong10Minutes();
      }
      if (autoDurMin != s_autoCircDurMin || autoPeriod10Min != s_autoCircPeriod10Min) {
         s_autoCircDurMin = autoDurMin;
         s_autoCircPeriod10Min = autoPeriod10Min;
         if (s_autoCircDurMin > 0U && s_autoCircPeriod10Min > 0U) {
            s_autoCircNextMs = now + (uint32_t)s_autoCircPeriod10Min * 10UL * 60000UL;
         } else {
            s_autoCircNextMs = 0;
         }
      }

      if (effContDispPressPulse) {
         if (s_waterCircActive && !s_waterCircBtnTracking) {
            // Press while already active: terminate circulation and reset tracking state.
            s_waterCircActive = false;
            s_waterCircUntilMs = 0;
            s_waterCircPausedByDispense = false;
            s_waterCircPauseStartMs = 0;
            s_waterCircBtnTracking = false;
            s_waterCircBtnPressMs = 0;
            if (s_autoCircDurMin > 0U && s_autoCircPeriod10Min > 0U) {
               s_autoCircNextMs = now + (uint32_t)s_autoCircPeriod10Min * 10UL * 60000UL;
            }
         } else if (!s_waterCircActive && wl > 0U) {
            // Start circulation immediately on press; timeout is resolved on release.
            s_waterCircActive = true;
            s_waterCircPausedByDispense = false;
            s_waterCircPauseStartMs = 0;
            s_waterCircBtnTracking = true;
            s_waterCircBtnPressMs = now;
            // Sentinel deadline while held; release computes the actual timeout.
            s_waterCircUntilMs = 0xFFFFFFFFUL;
         }
      }

      if (s_waterCircBtnTracking && effContDispReleasePulse) {
         const uint32_t pressMs = (uint32_t)contDispLastPressMs;
         const bool useShort = (pressMs <= 2000UL);
         const uint32_t targetMs = useShort
            ? ((uint32_t)manShortMin * 60000UL)
            : ((uint32_t)manLong10Min * 10UL * 60000UL);

         if (wl == 0U || targetMs == 0UL || pressMs >= targetMs) {
            s_waterCircActive = false;
            s_waterCircUntilMs = 0;
            s_waterCircPausedByDispense = false;
            s_waterCircPauseStartMs = 0;
         } else {
            s_waterCircUntilMs = now + (targetMs - pressMs);
            s_waterCircActive = true;
         }
         s_waterCircBtnTracking = false;
         s_waterCircBtnPressMs = 0;
      }

      // Auto-circulation starts only outside DayEnd and only when tank is non-empty.
      if (!s_waterCircActive && s_dayEndPhase == DayEndNone &&
          s_autoCircDurMin > 0U && s_autoCircPeriod10Min > 0U &&
          (int32_t)(now - s_autoCircNextMs) >= 0 && wl > 0U) {
         const uint32_t durMs = (uint32_t)s_autoCircDurMin * 60000UL;
         if (durMs > 0UL) {
            s_waterCircActive = true;
            s_waterCircUntilMs = now + durMs;
            s_waterCircPausedByDispense = false;
         }
         do {
            s_autoCircNextMs += (uint32_t)s_autoCircPeriod10Min * 10UL * 60000UL;
         } while ((int32_t)(now - s_autoCircNextMs) >= 0);
      }

      if (s_waterCircActive) {
         const bool heldManual = s_waterCircBtnTracking && effContDispDown;
         const uint32_t expiryNow = s_waterCircPausedByDispense ? s_waterCircPauseStartMs : now;
         if (wl == 0U || (!heldManual && (int32_t)(expiryNow - s_waterCircUntilMs) >= 0)) {
            s_waterCircActive = false;
            s_waterCircUntilMs = 0;
            s_waterCircPausedByDispense = false;
            s_waterCircPauseStartMs = 0;
            s_waterCircBtnTracking = false;
            s_waterCircBtnPressMs = 0;
         }
      }

      bool dispenseReqPulse = false;
      if (s_dayEndPhase == DayEndNone) {
         if (overrideActive) {
            if (btnFrontDispPulse || rearDispensePulse) {
               dispenseReqPulse = true;
               LastPayment = 0;
            }
         } else {
            if (rearDispensePulse) {
               dispenseReqPulse = true;
            } else if (btnFrontDispPulse && KioskPayment(true, true, true, eeprom)) {
               dispenseReqPulse = true;
            }
         }
      }

      const bool periodicFlushPermittedMode =
         (s_dayEndPhase == DayEndNone) && !s_dayEndReqPending && uiWelcomeActive;
      const uint8_t dnfRateMin =
         (eeprom && eeprom->isReady()) ? eeprom->dnfReptPeriodMinutes() : 0U;
      const uint8_t dnfDur100ms =
         (eeprom && eeprom->isReady()) ? eeprom->dnfReptDurat100ms() : 0U;
      const uint8_t preBeepDelaySec =
         (eeprom && eeprom->isReady()) ? eeprom->preFlshBeepDelySeconds() : 0U;
      const uint8_t beepOn10ms =
         (eeprom && eeprom->isReady()) ? eeprom->beepOnTime10ms() : 0U;
      const uint8_t beepPeriod100ms =
         (eeprom && eeprom->isReady()) ? eeprom->beepOffTime100ms() : 0U;
      const uint8_t beepCount =
         (eeprom && eeprom->isReady()) ? eeprom->beepCount() : 0U;
      const bool periodicFlushBeepEnabled =
         (beepCount > 0U) &&
         (beepOn10ms > 0U) &&
         (beepPeriod100ms > 0U);

      const bool periodicFlushConfigValid =
         (dnfRateMin > 0U) &&
         (dnfDur100ms > 0U) &&
         ((beepCount == 0U) || periodicFlushBeepEnabled);

      if (welcomeEntered) {
         s_periodicFlushPhase = PeriodicFlushIdle;
         s_periodicFlushDueMs = 0;
         s_periodicFlushStepUntilMs = 0;
         s_periodicFlushBeepsRemaining = 0;
         s_periodicFlushDispenseInFlight = false;
         s_periodicFlushDispenseStartMs = 0;
         s_periodicFlushDispenseDurationMs = 0;
         s_periodicFlushNextDueMs = 0;
         KioskIO::setAcousticAlert(false);
      }
      if (welcomeExited) {
         if (!s_periodicFlushDispenseInFlight) {
            s_periodicFlushPhase = PeriodicFlushIdle;
            s_periodicFlushDueMs = 0;
            s_periodicFlushStepUntilMs = 0;
            s_periodicFlushBeepsRemaining = 0;
            s_periodicFlushDispenseStartMs = 0;
            s_periodicFlushDispenseDurationMs = 0;
            s_periodicFlushNextDueMs = 0;
            KioskIO::setAcousticAlert(false);
         }
      }

      if ((!periodicFlushConfigValid || darkModeActive || !periodicFlushPermittedMode) &&
          !s_periodicFlushDispenseInFlight) {
         s_periodicFlushPhase = PeriodicFlushIdle;
         s_periodicFlushDueMs = 0;
         s_periodicFlushStepUntilMs = 0;
         s_periodicFlushBeepsRemaining = 0;
         s_periodicFlushDispenseMs = 0;
         s_periodicFlushNextDueMs = 0;
         s_periodicFlushDispenseInFlight = false;
         s_periodicFlushDispenseStartMs = 0;
         s_periodicFlushDispenseDurationMs = 0;
         KioskIO::setAcousticAlert(false);
      } else {
         s_periodicFlushPreDelayMs = (uint32_t)preBeepDelaySec * 1000UL;
         s_periodicFlushBeepOnMs = (uint32_t)beepOn10ms * 10UL;
         s_periodicFlushBeepPeriodMs = (uint32_t)beepPeriod100ms * 100UL;
         s_periodicFlushDispenseMs = (uint32_t)dnfDur100ms * 100UL;

         const uint32_t dnfPeriodMs = (uint32_t)dnfRateMin * 60000UL;
         if (s_periodicFlushNextDueMs == 0U) {
            s_periodicFlushNextDueMs = now + dnfPeriodMs;
         }
         if (s_periodicFlushPhase == PeriodicFlushIdle &&
             !dispensingNow &&
             (int32_t)(now - s_periodicFlushNextDueMs) >= 0) {
            s_periodicFlushDueMs = s_periodicFlushNextDueMs;
            do {
               s_periodicFlushNextDueMs += dnfPeriodMs;
            } while ((int32_t)(now - s_periodicFlushNextDueMs) >= 0);
            if (periodicFlushBeepEnabled) {
               s_periodicFlushBeepsRemaining = beepCount;
               s_periodicFlushPhase = PeriodicFlushBeepOn;
               s_periodicFlushBeepStartMs = now;
               s_periodicFlushStepUntilMs = now + s_periodicFlushBeepOnMs;
               KioskIO::setAcousticAlert(true);
            } else {
               s_periodicFlushBeepsRemaining = 0;
               const uint32_t targetMs = s_periodicFlushDueMs + s_periodicFlushPreDelayMs;
               if ((int32_t)(now - targetMs) >= 0) {
                  s_periodicFlushPhase = PeriodicFlushAwaitDispenseStart;
               } else {
                  s_periodicFlushPhase = PeriodicFlushPreDelay;
                  s_periodicFlushStepUntilMs = targetMs;
               }
               KioskIO::setAcousticAlert(false);
            }
         } else {
            if (!periodicFlushBeepEnabled &&
                (s_periodicFlushPhase == PeriodicFlushBeepOn ||
                 s_periodicFlushPhase == PeriodicFlushBeepOff)) {
               const uint32_t targetMs = s_periodicFlushDueMs + s_periodicFlushPreDelayMs;
               if ((int32_t)(now - targetMs) >= 0) {
                  s_periodicFlushPhase = PeriodicFlushAwaitDispenseStart;
               } else {
                  s_periodicFlushPhase = PeriodicFlushPreDelay;
                  s_periodicFlushStepUntilMs = targetMs;
               }
               s_periodicFlushBeepsRemaining = 0;
               KioskIO::setAcousticAlert(false);
            }
            switch (s_periodicFlushPhase) {
               case PeriodicFlushBeepOn:
                  KioskIO::setAcousticAlert(true);
                  if ((int32_t)(now - s_periodicFlushStepUntilMs) >= 0) {
                     KioskIO::setAcousticAlert(false);
                     if (s_periodicFlushBeepsRemaining > 0U) --s_periodicFlushBeepsRemaining;
                     if (s_periodicFlushBeepsRemaining > 0U) {
                        s_periodicFlushPhase = PeriodicFlushBeepOff;
                        s_periodicFlushStepUntilMs = s_periodicFlushBeepStartMs + s_periodicFlushBeepPeriodMs;
                     } else {
                        const uint32_t targetMs = s_periodicFlushDueMs + s_periodicFlushPreDelayMs;
                        if ((int32_t)(now - targetMs) >= 0) {
                           s_periodicFlushPhase = PeriodicFlushAwaitDispenseStart;
                        } else {
                           s_periodicFlushPhase = PeriodicFlushPreDelay;
                           s_periodicFlushStepUntilMs = targetMs;
                        }
                     }
                  }
                  break;
               case PeriodicFlushBeepOff:
                  KioskIO::setAcousticAlert(false);
                  if ((int32_t)(now - s_periodicFlushStepUntilMs) >= 0) {
                     s_periodicFlushPhase = PeriodicFlushBeepOn;
                     s_periodicFlushBeepStartMs = now;
                     s_periodicFlushStepUntilMs = now + s_periodicFlushBeepOnMs;
                     KioskIO::setAcousticAlert(true);
                  }
                  break;
               case PeriodicFlushPreDelay:
                  KioskIO::setAcousticAlert(false);
                  if ((int32_t)(now - s_periodicFlushStepUntilMs) >= 0) {
                     s_periodicFlushPhase = PeriodicFlushAwaitDispenseStart;
                  }
                  break;
               case PeriodicFlushAwaitDispenseStart:
               case PeriodicFlushAwaitDispenseDone:
               case PeriodicFlushIdle:
               default:
                  KioskIO::setAcousticAlert(false);
                  break;
            }
         }
      }

      bool periodicFlushDispenseReq = false;
      bool periodicFlushOwnsDispense = false;
      if (s_periodicFlushPhase == PeriodicFlushAwaitDispenseStart &&
          !dispensingNow &&
          !dispenseReqPulse) {
         periodicFlushDispenseReq = true;
         periodicFlushOwnsDispense = true;
      }
      if (periodicFlushDispenseReq) dispenseReqPulse = true;
      const bool periodicFlushRequestThisLoop = periodicFlushOwnsDispense && dispenseReqPulse;

      if (dispenseReqPulse) {
         if (eeprom && eeprom->isReady()) {
            if (periodicFlushOwnsDispense) {
               // Periodic Nozzle Flush always uses timed dispense mode.
               KioskHydraulics::setDispenseProfile(s_periodicFlushDispenseMs,
                                                   0U,
                                                   0U);
            } else {
               const auto profile = eeprom->dispMeasuredProfile();
               KioskHydraulics::setDispenseProfile(eeprom->dispMeasuredDurationMs(),
                                                   profile.pulses,
                                                   profile.modeSel);
            }
         } else {
            raiseError(ERR_DISP_PROFILE_UNAVAILABLE);
            dispenseReqPulse = false;
         }
      }

      // KioskHydraulics is the single writer for backwash/inlet/dispense outputs.
      KioskHydraulics::update(now, dispenseReqPulse);
      if (KioskHydraulics::takeDispenseUvFault()) {
         raiseError(ERR_UV_NOT_OK);
      }
      const bool dispensingAfter = (KioskHydraulics::state() != 0);

      // Circulation uses only the circulation output.
      // During dispensing, circulation output is forced OFF and timer progression is paused.
      if (s_waterCircActive) {
         if (dispensingAfter && !s_waterCircPausedByDispense) {
            s_waterCircPausedByDispense = true;
            s_waterCircPauseStartMs = now;
         } else if (!dispensingAfter && s_waterCircPausedByDispense) {
            const uint32_t pausedMs = (uint32_t)(now - s_waterCircPauseStartMs);
            s_waterCircUntilMs += pausedMs;
            s_waterCircPausedByDispense = false;
         }
      } else {
         s_waterCircPausedByDispense = false;
      }
      KioskIO::setWaterCirculation(s_waterCircActive && !dispensingAfter);

      // Increment BW DispCount after each completed dispense.
      // If BW AutoAfter N is disabled (0), only increment.
      // If enabled and reached, reset to 0 and trigger AutoBackwash.
      const bool dispenseCompletedThisLoop = (dispensingNow && !dispensingAfter &&
                                              !dayEndCanceledDispenseThisLoop);
      const bool dispenseStartedThisLoop = (!dispensingNow && dispensingAfter &&
                                            !dayEndCanceledDispenseThisLoop);
      if (dispenseStartedThisLoop) {
         if (s_periodicFlushPhase == PeriodicFlushAwaitDispenseStart &&
             periodicFlushRequestThisLoop) {
            s_periodicFlushPhase = PeriodicFlushAwaitDispenseDone;
            s_periodicFlushDispenseInFlight = true;
            s_periodicFlushDispenseStartMs = now;
            s_periodicFlushDispenseDurationMs = s_periodicFlushDispenseMs;
         } else if (s_periodicFlushPhase != PeriodicFlushIdle &&
                    s_periodicFlushPhase != PeriodicFlushAwaitDispenseDone) {
            // Any non-periodic dispense activity cancels a pending periodic flush.
            s_periodicFlushPhase = PeriodicFlushIdle;
            s_periodicFlushBeepsRemaining = 0;
            s_periodicFlushDispenseInFlight = false;
            s_periodicFlushDispenseStartMs = 0;
            s_periodicFlushDispenseDurationMs = 0;
            KioskIO::setAcousticAlert(false);
         } else if (s_periodicFlushPhase == PeriodicFlushAwaitDispenseStart) {
            // A non-periodic dispense started while waiting for periodic flush start.
            s_periodicFlushPhase = PeriodicFlushIdle;
            s_periodicFlushBeepsRemaining = 0;
            s_periodicFlushDispenseInFlight = false;
            s_periodicFlushDispenseStartMs = 0;
            s_periodicFlushDispenseDurationMs = 0;
            KioskIO::setAcousticAlert(false);
         }
      }
      if (dispenseCompletedThisLoop && s_periodicFlushDispenseInFlight) {
         s_periodicFlushPhase = PeriodicFlushIdle;
         s_periodicFlushDispenseInFlight = false;
         s_periodicFlushDispenseStartMs = 0;
         s_periodicFlushDispenseDurationMs = 0;
      }
      if (dispenseCompletedThisLoop && eeprom && eeprom->isReady()) {
         uint16_t bwCount = eeprom->backwashDispenseCounter();
         if (bwCount < 0xFFFFU) ++bwCount;
         const uint8_t autoAfterN = eeprom->backwashAfterNDispenses();
         if (autoAfterN == 0U) {
            eeprom->setBackwashDispenseCounter(bwCount);
         } else if (bwCount >= (uint16_t)autoAfterN) {
            eeprom->setBackwashDispenseCounter(0);
            s_autoBackwashReqPending = true;
         } else {
            eeprom->setBackwashDispenseCounter(bwCount);
         }
      }

      // DayEnd mode sequencing:
      // 1) Recirculation for EoD Circ duration.
      // 2) Backwash for EoD BW duration.
      if (s_dayEndPhase == DayEndRecirc) {
         if (!s_dayEndCircStarted) {
            if (s_dayEndCircDurationMs > 0UL && wl > 1U) {
               s_dayEndCircStarted = true;
               s_waterCircActive = true;
               s_waterCircPausedByDispense = false;
               s_waterCircPauseStartMs = 0;
               s_waterCircUntilMs = now + s_dayEndCircDurationMs;
            }
         } else if (!s_waterCircActive) {
            s_dayEndCircStarted = false;
            s_dayEndCircDurationMs = 0;
            if (s_dayEndBwDurationMs > 0UL) {
               s_dayEndPhase = DayEndBackwash;
               s_dayEndBwStarted = false;
               const bool eodBwEligible = (wl > 1U);
               if (eodBwEligible) {
                  KioskHydraulics::requestBackwashWithDuration(s_dayEndBwDurationMs);
                  if (evt == BackwashNone) evt = BackwashStartReq;
               }
            } else {
               s_dayEndPhase = DayEndNone;
               s_dayEndBwStarted = false;
               s_dayEndBwDurationMs = 0;
            }
         }
      }

      if (s_dayEndPhase == DayEndBackwash) {
         const bool bwNow = KioskHydraulics::backwashActive();
         if (bwNow) {
            s_dayEndBwStarted = true;
         } else if (s_dayEndBwStarted) {
            s_dayEndPhase = DayEndNone;
            s_dayEndCircDurationMs = 0;
            s_dayEndCircStarted = false;
            s_dayEndBwStarted = false;
            s_dayEndBwDurationMs = 0;
         } else {
            const bool eodBwEligible = (wl > 1U);
            if (s_dayEndBwDurationMs > 0UL && eodBwEligible) {
               KioskHydraulics::requestBackwashWithDuration(s_dayEndBwDurationMs);
               if (evt == BackwashNone) evt = BackwashStartReq;
            }
         }
      }

      if (!KioskHydraulics::backwashActive() && s_sensorBypassPendingToggle) {
         s_sensorBypassPendingToggle = false;
         if (s_sensorBypass) applySensorBypassState(false);
         else applySensorBypassState(true, s_sensorBypassPendingDurationMs);
         s_sensorBypassPendingDurationMs = 0;
      }

      // When a backwash cycle finishes, reset BWdispCount.
      const bool backwashActiveAfter = KioskHydraulics::backwashActive();
      // Persist non-daily backwash starts only on an actual start transition.
      if (!backwashActiveBefore && backwashActiveAfter &&
          !s_dailyBackwashTriggeredThisLoop && KioskIO::rtcPresent()) {
         KioskIO::RtcTime rtc{};
         if (KioskIO::rtcRead(rtc)) {
            (void)KioskIO::rtcWriteTriggeredBackwashStamp(rtc);
         }
      }
      if (backwashActiveBefore && !backwashActiveAfter && eeprom && eeprom->isReady()) {
         eeprom->setBackwashDispenseCounter(0);
      }
      const bool dayEndInactiveNow = (!s_dayEndReqPending && (s_dayEndPhase == DayEndNone));
      if (dayEndActiveAtEntry && dayEndInactiveNow && wl < 3U) {
         // On DayEnd exit, restore inlet demand so refill resumes immediately.
         s_manualInletOn = true;
         s_inletHoldActive = false;
         s_inletHoldStartMs = 0;
         s_inletHoldUntilMs = 0;
      }
      // Inlet demand is button-latched, subject to 1h hold and water-level safety.
      const bool inletDemand = (s_autoRefillLatch || s_manualInletOn) && !s_inletHoldActive && (wl < 3U);
      const bool wantInlet = inletDemand && !KioskHydraulics::backwashActive() && !s_sensorBypass;
      KioskIO::setWaterInlet(wantInlet, now, KioskIO::INLET_OWNER_BEHAVIOR);
      return evt;
   }

   namespace Ui {

      // Pad/truncate to the LCD's 16-character width.
      static void pad16(char* s)
      {
         size_t n = strlen(s);
         if (n > 16) {
            s[16] = 0;
            return;
         }
         while (n < 16) s[n++] = ' ';
         s[16] = 0;
      }

      // Trim leading/trailing spaces and tabs into a small buffer.
      static void trimText(const char* src, char* out, size_t outSize)
      {
         if (!src || outSize == 0) return;
         const char* start = src;
         while (*start == ' ' || *start == '\t') ++start;
         const char* end = start + strlen(start);
         while (end > start && (end[-1] == ' ' || end[-1] == '\t')) --end;
         size_t len = (size_t)(end - start);
         if (len >= outSize) len = outSize - 1;
         memcpy(out, start, len);
         out[len] = 0;
      }

      // Copy and pad a single LCD line.
      static void setLine(char* dst, const char* src)
      {
         char trimmed[17];
         trimText(src, trimmed, sizeof(trimmed));
         const size_t len = strlen(trimmed);
         for (size_t i = 0; i < 16; ++i) dst[i] = ' ';
         dst[16] = 0;
         if (len >= 16) {
            strncpy(dst, trimmed, 16);
            dst[16] = 0;
            return;
         }
         const size_t start = (16 - len) / 2;
         memcpy(dst + start, trimmed, len);
      }

      // Limit counters to 5 digits for display.
      static uint32_t clamp5(uint32_t v) { return (v > 99999UL) ? 99999UL : v; }

      // Helper to set all four LCD lines at once.
      static void setLines(UiOutputs& out, const char* l0, const char* l1, const char* l2, const char* l3)
      {
         setLine(out.line0, l0);
         setLine(out.line1, l1);
         setLine(out.line2, l2);
         setLine(out.line3, l3);
         out.lcdDirty = true;
      }

      // Left-justify label and right-justify a 5-digit counter.
      static void setLineLabelCounter(char* dst, const char* label, uint32_t value)
      {
         char trimmed[17];
         trimText(label, trimmed, sizeof(trimmed));
         const size_t labelLen = strlen(trimmed);
         for (size_t i = 0; i < 16; ++i) dst[i] = ' ';
         dst[16] = 0;
         const size_t maxLabel = (labelLen > 11) ? 11 : labelLen;
         memcpy(dst, trimmed, maxLabel);
         char num[6];
         snprintf(num, sizeof(num), "%5lu", (unsigned long)clamp5(value));
         memcpy(dst + 11, num, 5);
      }

      const char* uiStateId(UiState st)
      {
         switch (st) {
            case UI_BOOT: return "UI_BOOT";
            case UI_WELCOME: return "UI_WELCOME";
            case UI_PAY_PROMPT: return "UI_PAY_PROMPT";
            case UI_PAY_FAILED: return "UI_PAY_FAILED";
            case UI_PAY_COIN_OK: return "UI_PAY_COIN_OK";
            case UI_PAY_APP_OK: return "UI_PAY_APP_OK";
            case UI_PAY_TOKEN_OK: return "UI_PAY_TOKEN_OK";
            case UI_PAY_TOKEN_BAD: return "UI_PAY_TOKEN_BAD";
            case UI_OVERRIDE: return "UI_OVERRIDE";
            case UI_DARK_MODE_IN: return "UI_DRKMOD_ENT";
            case UI_DARK_MODE_OUT: return "UI_DRKMDE_EXT";
            case UI_DISP_RDY: return "UI_DISP_RDY";
            case UI_DISPENSING: return "UI_DISPENSING";
            case UI_DISP_DONE: return "UI_DISP_DONE";
            case UI_TIMEOUT: return "UI_TIMEOUT";
            case UI_OUT_OF_SERVICE: return "UI_OUT_OF_SERVICE";
            case UI_ERROR: return "UI_ERROR";
            default: return "UNKNOWN";
         }
      }

      // Format a "Label:nnnnn" line with fixed width.
      static void formatCounterLine(char* dst, const char* label, uint32_t value)
      {
         char buf[20];
         snprintf(buf, sizeof(buf), "%s%5lu", label, (unsigned long)clamp5(value));
         setLine(dst, buf);
      }

      // Enter a new UI state and reset its timers/one-shot flags.
      static void enter(UiContext& ctx, UiState st, uint32_t now)
      {
         ctx.state = st;
         ctx.stateStartMs = now;
         ctx.dispensePulseArmed = true;
         ctx.countedThisState = false;
         if (st == UI_DARK_MODE_IN || st == UI_DARK_MODE_OUT) {
            ctx.darkModeSkipArmed = false;
            ctx.darkModeHoldStartMs = 0;
            ctx.darkModeLastRemain = -1;
         }
         if (st == UI_PAY_PROMPT) ctx.paymentStartMs = 0;
      }

      // Preferred dispense duration (EEPROM configured, else fallback).
      static uint32_t dispenseDurationMs(const UiContext& ctx)
      {
         if (ctx.eeprom) {
            const uint32_t d = ctx.eeprom->dispMeasuredDurationMs();
            if (d > 0) return d;
         }
         return 5000UL;
      }

      // Initialize the UI state machine.
      void uiInit(UiContext& ctx, Kiosk::KioskEeprom* eeprom, uint32_t now)
      {
         memset(&ctx, 0, sizeof(ctx));
         ctx.eeprom = eeprom;
         ctx.state = UI_BOOT;
         ctx.lastRendered = (UiState)255;
         ctx.stateStartMs = now;
         ctx.paymentStartMs = 0;
         ctx.dispenseStartMs = 0;
         ctx.lastProgressMs = 0;
         ctx.lastProgressBars = 0;
         ctx.lastProgressPct = 0;
         ctx.dispensePulseArmed = true;
         ctx.countedThisState = false;
         ctx.welcomePressArmed = false;
         ctx.darkModeArmed = false;
         ctx.darkModeSkipArmed = false;
         ctx.darkModeHoldStartMs = 0;
         ctx.darkModeLastRemain = -1;
      }

      void uiUpdate(UiContext& ctx, const UiInputs& in, UiOutputs& out, uint32_t now)
      {
         const UiState prevRendered = ctx.lastRendered;
         out.btnFrontDispLED = LED_OFF;
         out.dispenseButtonLed = LED_OFF;
         out.backlight = BL_OFF;
         out.requestDispensePulse = false;
         out.requestReset = false;
         out.lcdDirty = false;
         setLine(out.line0, "                ");
         setLine(out.line1, "                ");
         setLine(out.line2, "                ");
         setLine(out.line3, "                ");

         const bool overrideActive = KioskIO::overrideActive();
      const bool backwashActive = KioskHydraulics::backwashActive();
      const bool dispensing = (KioskHydraulics::state() != 0);

      bool forceRender = false;

      // Press/release latches are only meaningful in the WELCOME state.
      if (ctx.state != UI_WELCOME) {
         ctx.welcomePressArmed = false;
         ctx.darkModeArmed = false;
      }
      if (ctx.state != UI_DARK_MODE_IN && ctx.state != UI_DARK_MODE_OUT) {
         ctx.darkModeSkipArmed = false;
      }

      if (ctx.state != UI_ERROR &&
          ctx.state != UI_DARK_MODE_IN &&
          ctx.state != UI_DARK_MODE_OUT &&
          errorActive()) {
         enter(ctx, UI_ERROR, now);
         forceRender = true;
      }

      switch (ctx.state) {
            case UI_BOOT: {
               out.btnFrontDispLED = LED_ON;
               out.backlight = BL_ON;
               if (ctx.eeprom) {
                  setLineLabelCounter(out.line0, "App Sales:", ctx.eeprom->dispenseCounter(Kiosk::KioskEeprom::DISP_APP));
                  setLineLabelCounter(out.line1, "Coin Sales:", ctx.eeprom->dispenseCounter(Kiosk::KioskEeprom::DISP_COIN));
                  setLineLabelCounter(out.line2, "NFC Tag:", ctx.eeprom->dispenseCounter(Kiosk::KioskEeprom::DISP_NFC));
                  setLineLabelCounter(out.line3, "Override:", ctx.eeprom->dispenseCounter(Kiosk::KioskEeprom::DISP_BYPASS));
               } else {
                  setLineLabelCounter(out.line0, "App Sales:", 0);
                  setLineLabelCounter(out.line1, "Coin Sales:", 0);
                  setLineLabelCounter(out.line2, "NFC Tag:", 0);
                  setLineLabelCounter(out.line3, "Override:", 0);
               }
               out.lcdDirty = true;

               if (in.btnFrontDispPressed || (uint32_t)(now - ctx.stateStartMs) >= 60000UL) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_WELCOME: {
               out.btnFrontDispLED = LED_PULSE;
               out.backlight = BL_FLICK_ON_INTRO;
               if (overrideActive) {
                  setLines(out, "Majicom Welcome!", "================", "Press button", "to start");
               } else {
                  setLines(out, "Majicom Welcome!", "Press button", "to start", "KSh10/500ml");
               }

               // Operator dark-mode gesture:
               // - press while override is ON (arms darkModeArmed)
               // - release after override is switched OFF (enters UI_DARK_MODE_IN)
               // A normal press/release with override still ON routes to UI_OVERRIDE.
               if (in.btnFrontDispPressed) {
                  ctx.welcomePressArmed = true;
                  if (overrideActive) ctx.darkModeArmed = true;
               }
               if (ctx.welcomePressArmed && !in.btnFrontDispDown) {
                  if (ctx.darkModeArmed && !overrideActive) {
                     enter(ctx, UI_DARK_MODE_IN, now);
                  } else if (overrideActive) {
                     enter(ctx, UI_OVERRIDE, now);
                  } else {
                     enter(ctx, UI_PAY_PROMPT, now);
                  }
                  ctx.welcomePressArmed = false;
                  ctx.darkModeArmed = false;
                  forceRender = true;
               }
               if (ctx.state != UI_WELCOME) break;

               // Route to OUT_OF_SERVICE only when dispensing is not OK and no
               // operator override/dark-mode gesture is in progress.
               if (in.dispenseNotOk && !overrideActive && !ctx.darkModeArmed) {
                  enter(ctx, UI_OUT_OF_SERVICE, now);
                  forceRender = true;
               } else if ((uint32_t)(now - ctx.stateStartMs) >= 5000UL) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_PAY_PROMPT: {
               if (overrideActive) {
                  enter(ctx, UI_OVERRIDE, now);
                  forceRender = true;
                  break;
               }

               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_ON;
               if (in.hasCoinAcceptor) {
                  setLines(out, "Buy with coins", "or the Majicom", "mobile/cell app", "KSh10/500ml");
               } else {
                  setLines(out, "Buy with the", "Majicom mobile/", "cell phone app", "KSh10/500ml");
               }

               // Arm payment after the LCD text has been rendered at least once.
               if (ctx.paymentStartMs == 0) {
                  ctx.paymentStartMs = now;
                  forceRender = true;
                  break;
               }

               // Enable NFC after the UI has rendered once.
               KioskIO::setNfcEnabled(in.hasNfc);

               const bool enableCoin = in.hasCoinAcceptor;
               const bool enableNfc = in.hasNfc;
               const bool paymentOk = KioskPayment(enableCoin, enableNfc, enableNfc, ctx.eeprom);

               if (paymentOk) {
                  switch (LastPayment) {
                     case 1: enter(ctx, UI_PAY_COIN_OK, now); break;
                     case 2: enter(ctx, UI_PAY_TOKEN_OK, now); break;
                     case 3: enter(ctx, UI_PAY_APP_OK, now); break;
                     default: enter(ctx, UI_PAY_FAILED, now); break;
                  }
                  ctx.paymentStartMs = 0;
                  forceRender = true;
               } else if (LastPayment == 255 && s_lastNfcHash != 0) {
                  enter(ctx, UI_PAY_TOKEN_BAD, now);
                  ctx.paymentStartMs = 0;
                  forceRender = true;
               } else if (LastPayment == 254) {
                  enter(ctx, UI_PAY_FAILED, now);
                  ctx.paymentStartMs = 0;
                  forceRender = true;
               } else if ((uint32_t)(now - ctx.paymentStartMs) >= 15000UL) {
                  enter(ctx, UI_WELCOME, now);
                  ctx.paymentStartMs = 0;
                  forceRender = true;
               }
               break;
            }
            case UI_PAY_FAILED: {
               out.btnFrontDispLED = LED_ON;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Payment failed!", "Press button to", "attempt payment", "again");
               if (in.btnFrontDispPressed) {
                  enter(ctx, UI_PAY_PROMPT, now);
                  forceRender = true;
               } else if ((uint32_t)(now - ctx.stateStartMs) >= 15000UL) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_PAY_COIN_OK: {
               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Thank you!", "Your coin", "payment has", "been processed");
               if (!ctx.countedThisState && ctx.eeprom) {
                  ctx.eeprom->incrementDispenseCounter(Kiosk::KioskEeprom::DISP_COIN);
                  ctx.countedThisState = true;
               }
               if ((uint32_t)(now - ctx.stateStartMs) >= 2000UL) {
                  enter(ctx, UI_DISP_RDY, now);
                  forceRender = true;
               }
               break;
            }
            case UI_PAY_APP_OK: {
               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Thank you!", "Your NFC Mobile", "App payment has", "been processed");
               if (!ctx.countedThisState && ctx.eeprom) {
                  ctx.eeprom->incrementDispenseCounter(Kiosk::KioskEeprom::DISP_APP);
                  ctx.countedThisState = true;
               }
               if ((uint32_t)(now - ctx.stateStartMs) >= 2000UL) {
                  enter(ctx, UI_DISP_RDY, now);
                  forceRender = true;
               }
               break;
            }
            case UI_PAY_TOKEN_OK: {
               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Thank you!", "Your NFC Token", "payment has", "been processed");
               if (!ctx.countedThisState && ctx.eeprom) {
                  ctx.eeprom->incrementDispenseCounter(Kiosk::KioskEeprom::DISP_NFC);
                  ctx.countedThisState = true;
               }
               if ((uint32_t)(now - ctx.stateStartMs) >= 2000UL) {
                  enter(ctx, UI_DISP_RDY, now);
                  forceRender = true;
               }
               break;
            }
            case UI_PAY_TOKEN_BAD: {
               out.btnFrontDispLED = LED_ON;
               out.backlight = BL_FLICK_ON_INTRO;
               char codeLine[17];
               snprintf(codeLine, sizeof(codeLine), "Code:%08lX", (unsigned long)s_lastNfcHash);
               setLines(out, "Invalid Token!", "Not accepted", "on this Kiosk", codeLine);
               if ((uint32_t)(now - ctx.stateStartMs) >= 3000UL || in.btnFrontDispPressed) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_OVERRIDE: {
               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Thank you for", "using this", "** Majicom **", "Water Kiosk!");
               if (!ctx.countedThisState && ctx.eeprom) {
                  ctx.eeprom->incrementDispenseCounter(Kiosk::KioskEeprom::DISP_BYPASS);
                  ctx.countedThisState = true;
               }
               if ((uint32_t)(now - ctx.stateStartMs) >= 2000UL) {
                  enter(ctx, UI_DISP_RDY, now);
                  forceRender = true;
               }
               break;
            }
            case UI_DARK_MODE_IN: {
               uint32_t ageMs = (uint32_t)(now - ctx.stateStartMs);
               const uint32_t longDelayMs = 60000UL;
               const uint32_t shortDelayMs = 2000UL;
               if (ageMs < longDelayMs && in.btnFrontDispPressed) {
                  // Truncate the entry countdown immediately on button press.
                  ctx.stateStartMs = now - longDelayMs;
                  ageMs = longDelayMs;
                  forceRender = true;
               }
               const uint32_t remainMs = (ageMs >= longDelayMs) ? 0 : (longDelayMs - ageMs);
               const int16_t remainSec = (int16_t)((remainMs + 999UL) / 1000UL);

               if (ageMs < longDelayMs) {
                  out.backlight = BL_ON;
                  out.btnFrontDispLED = LED_ON;
                  setLine(out.line0, "Entering");
                  setLine(out.line1, "Dark Mode");
                  setLine(out.line2, "----------------");
                  char countBuf[17];
                  snprintf(countBuf, sizeof(countBuf), "Countdown: %2ds", (int)remainSec);
                  setLine(out.line3, countBuf);
                  if (remainSec != ctx.darkModeLastRemain) {
                     ctx.darkModeLastRemain = remainSec;
                     forceRender = true;
                  }
               } else if (ageMs < (longDelayMs + shortDelayMs)) {
                  out.backlight = BL_ON;
                  out.btnFrontDispLED = LED_ON;
                  setLine(out.line0, "Entering");
                  setLine(out.line1, "DARK");
                  setLine(out.line2, "MODE");
                  setLine(out.line3, "----------------");
                  if (ctx.darkModeLastRemain != -2) {
                     ctx.darkModeLastRemain = -2;
                     forceRender = true;
                  }
               } else {
                  out.backlight = BL_OFF;
                  out.btnFrontDispLED = LED_OFF;
               }

               // Exit dark mode path:
               // - Once actually dark, button press enters explicit exit-hold state.
               if (ageMs >= (longDelayMs + shortDelayMs)) {
                  if (in.btnFrontDispPressed) {
                     enter(ctx, UI_DARK_MODE_OUT, now);
                     forceRender = true;
                  }
               }
               break;
            }
            case UI_DARK_MODE_OUT: {
               static constexpr uint32_t exitHoldMs = 8000UL;
               static constexpr uint32_t exitBacklightLeadMs = 3000UL;
               const uint32_t longDelayMs = 60000UL;
               const uint32_t shortDelayMs = 2000UL;
               const uint32_t elapsed = (uint32_t)(now - ctx.stateStartMs);
               const uint32_t remainMs = (elapsed >= exitHoldMs) ? 0UL : (exitHoldMs - elapsed);
               const int16_t remainSec = (int16_t)((remainMs + 999UL) / 1000UL);

               out.backlight = (remainMs <= exitBacklightLeadMs) ? BL_ON : BL_OFF;
               out.btnFrontDispLED = LED_ON;
               setLine(out.line0, "Exiting");
               setLine(out.line1, "Dark Mode");
               setLine(out.line2, "----------------");
               char countBuf[17];
               snprintf(countBuf, sizeof(countBuf), "Countdown: %2ds", (int)remainSec);
               setLine(out.line3, countBuf);

               if (remainSec != ctx.darkModeLastRemain) {
                  ctx.darkModeLastRemain = remainSec;
                  forceRender = true;
               }
               // Releasing before completion cancels exit and returns to dark mode.
               if (!in.btnFrontDispDown) {
                  enter(ctx, UI_DARK_MODE_IN, now);
                  ctx.stateStartMs = now - (longDelayMs + shortDelayMs);
                  forceRender = true;
               } else if (elapsed >= exitHoldMs) {
                  out.requestReset = true;
               }
               break;
            }
            case UI_DISP_RDY: {
               out.btnFrontDispLED = LED_ON;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Place your", "container under", "the nozzle and", "press the button");
               if (in.btnFrontDispPressed || in.btnRearDispPressed) {
                  enter(ctx, UI_DISPENSING, now);
                  if (ctx.dispensePulseArmed) {
                     out.requestDispensePulse = true;
                     ctx.dispensePulseArmed = false;
                     ctx.dispenseStartMs = now;
                     ctx.lastProgressMs = 0;
                     ctx.lastProgressBars = 0;
                     ctx.lastProgressPct = 0;
                  }
                  forceRender = true;
               } else if ((uint32_t)(now - ctx.stateStartMs) >= 60000UL) {
                  enter(ctx, UI_TIMEOUT, now);
                  forceRender = true;
               }
               break;
            }
            case UI_DISPENSING: {
               out.btnFrontDispLED = LED_PULSE;
               out.backlight = BL_ON;
               if (ctx.dispenseStartMs == 0) ctx.dispenseStartMs = now;

               const uint32_t elapsed = (uint32_t)(now - ctx.dispenseStartMs);
               const uint32_t duration = dispenseDurationMs(ctx);
               const uint32_t maxFill = duration;

               uint8_t pct = 0;
               if (duration > 0) {
                  const uint32_t scaled = (elapsed * 100UL) / duration;
                  pct = (scaled > 100UL) ? 100U : (uint8_t)scaled;
               }

               bool progressUpdate = false;
               if (ctx.lastProgressMs == 0 || (uint32_t)(now - ctx.lastProgressMs) >= 1000UL) {
                  const uint8_t coarseBars = (uint8_t)((pct * 12U) / 100U);
                  if (coarseBars != ctx.lastProgressBars || pct != ctx.lastProgressPct) {
                     ctx.lastProgressBars = coarseBars;
                     ctx.lastProgressPct = pct;
                     progressUpdate = true;
                  }
                  ctx.lastProgressMs = now;
               }

               char barLine[17];
               char bar[13];
               const uint16_t totalCols = 12U * 5U;
               uint16_t filledCols = (uint16_t)((pct * totalCols) / 100U);
               for (uint8_t i = 0; i < 12; ++i) {
                  uint8_t cellCols = 0;
                  if (filledCols >= 5U) {
                     cellCols = 5U;
                     filledCols -= 5U;
                  } else {
                     cellCols = (uint8_t)filledCols;
                     filledCols = 0;
                  }
                  if (cellCols == 0) bar[i] = ' ';
                  else bar[i] = (char)cellCols; // LCD custom chars 1..5
               }
               bar[12] = 0;
               snprintf(barLine, sizeof(barLine), "%s %2u%%", bar, (unsigned)pct);
               setLines(out, "Dispensing clean", "pure water...", "", barLine);

               if (dispensing == false && elapsed > 0) {
                  enter(ctx, UI_DISP_DONE, now);
                  forceRender = true;
               } else if (elapsed >= maxFill) {
                  enter(ctx, UI_DISP_DONE, now);
                  forceRender = true;
               } else if (progressUpdate) {
                  forceRender = true;
               }
               break;
            }
            case UI_DISP_DONE: {
               out.btnFrontDispLED = LED_PULSE;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Dispensing done", "Thank you!", "Please come back", "again soon!");
               if ((uint32_t)(now - ctx.stateStartMs) >= 2000UL || in.btnFrontDispPressed) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_TIMEOUT: {
               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_FLICK_ON_INTRO;
               setLines(out, "Sorry!", "", "your transaction", "has timed out");
               if ((uint32_t)(now - ctx.stateStartMs) >= 3000UL) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_OUT_OF_SERVICE: {
               out.btnFrontDispLED = LED_OFF;
               out.backlight = BL_ON;
               setLines(out, "Sorry! We're not", "dispensing now", "Purifying water", "Come Back Soon !!");
               // Override should always let the operator return to WELCOME,
               // including low/empty water conditions, so dark mode remains reachable.
               if (!in.dispenseNotOk || overrideActive) {
                  enter(ctx, UI_WELCOME, now);
                  forceRender = true;
               }
               break;
            }
            case UI_ERROR: {
               out.btnFrontDispLED = LED_OFF;
               const uint32_t ageMs = (uint32_t)(now - ctx.stateStartMs);
               out.backlight = ((ageMs % 1000UL) < 100UL) ? BL_OFF : BL_ON;
               char codeLine[17];
               const uint16_t ec = errorCode();
               if (ec == ERR_UV_NOT_OK) {
                  snprintf(codeLine, sizeof(codeLine), "Code:%4u (UV)", (unsigned)ec);
               } else {
                  snprintf(codeLine, sizeof(codeLine), "Code:%4u", (unsigned)ec);
               }
               setLines(out, "Not Dispensing", "Purifying Water", "Come Back Soon", codeLine);
               if (ageMs >= 1000UL) {
                  enter(ctx, UI_ERROR, now);
                  forceRender = true;
               }
               break;
            }
            default:
               enter(ctx, UI_WELCOME, now);
               forceRender = true;
               break;
         }

         if (ctx.lastRendered != ctx.state || forceRender) {
            ctx.lastRendered = ctx.state;
            out.lcdDirty = true;
         }

         const bool wantNfc =
            (ctx.state == UI_PAY_PROMPT) &&
            (ctx.paymentStartMs != 0) &&
            (prevRendered == UI_PAY_PROMPT) &&
            in.hasNfc;
         KioskIO::setNfcEnabled(wantNfc);

         // Default to mirroring the btnFrontDispLED for the dispense ring.
         out.dispenseButtonLed = out.btnFrontDispLED;
      }

      struct UiDocMap {
         UiState st;
         const char* doc;
      };

      static const UiDocMap kUiDocMap[] = {
         { UI_BOOT,            "UI_BOOT" },
         { UI_WELCOME,    "UI_WELCOME" },
         { UI_PAY_PROMPT,  "UI_PAY_PROMPT" },
         { UI_PAY_FAILED,  "UI_PAY_FAILED" },
         { UI_PAY_COIN_OK, "UI_PAY_COIN_OK" },
         { UI_PAY_APP_OK,  "UI_PAY_APP_OK" },
         { UI_PAY_TOKEN_OK,"UI_PAY_TOKEN_OK" },
         { UI_PAY_TOKEN_BAD,"UI_PAY_TOKEN_BAD" },
         { UI_OVERRIDE, "UI_OVERRIDE" },
         { UI_DARK_MODE_IN, "UI_DARK_MODE_IN" },
         { UI_DARK_MODE_OUT, "UI_DARK_MODE_OUT" },
         { UI_DISP_RDY,  "UI_DISP_RDY" },
         { UI_DISPENSING,      "UI_DISPENSING" },
         { UI_DISP_DONE,   "UI_DISP_DONE" },
         { UI_TIMEOUT,        "UI_TIMEOUT" },
         { UI_OUT_OF_SERVICE,  "UI_OUT_OF_SERVICE" },
         { UI_ERROR,           "UI_ERROR" }
      };

      static bool strEqIgnoreCase(const char* a, const char* b)
      {
         while (*a && *b) {
            char ca = *a++;
            char cb = *b++;
            if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
            if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
            if (ca != cb) return false;
         }
         return *a == 0 && *b == 0;
      }

      const char* docStateName(UiState st)
      {
         for (size_t i = 0; i < (sizeof(kUiDocMap) / sizeof(kUiDocMap[0])); ++i) {
            if (kUiDocMap[i].st == st) return kUiDocMap[i].doc;
         }
         return "?";
      }

      bool parseDocStateName(const char* s, UiState& out)
      {
         for (size_t i = 0; i < (sizeof(kUiDocMap) / sizeof(kUiDocMap[0])); ++i) {
            if (strEqIgnoreCase(s, kUiDocMap[i].doc)) {
               out = kUiDocMap[i].st;
               return true;
            }
         }
         return false;
      }

   } // namespace Ui

} // namespace KioskBehavior
