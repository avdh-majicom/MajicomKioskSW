// KioskHydraulics.cpp
// -----------------------------------------------------------------------------

#include "KioskHydraulics.h"
#include "KioskIO.h"

namespace KioskHydraulics {

   // Dispense state machine. Backwash is handled as an independent overlay.
   enum : uint8_t {
      ST_IDLE      = 0,
      ST_UV_WAIT   = 1,
      ST_DISPENSE  = 2,
      ST_UV_POST   = 3
   };

   static uint8_t  s_state              = ST_IDLE;
   static uint32_t s_tStateMs           = 0;
   static bool     s_prevDispenseCmd    = false;

   // Informational counter for UI/status only.
   // It is reset when a backwash cycle starts.
   static uint8_t  s_dispenseSinceBw    = 0;
   static bool     s_backwashReq        = false;  // one-loop request pulse
   static bool     s_backwashActive     = false;
   static uint32_t s_bwStartMs          = 0;

   static bool     s_overrideActive     = false;

   // Fallback default when no EEPROM-driven duration is applied.
   static constexpr uint32_t BACKWASH_MS  = 5000UL;
   static uint32_t s_backwashDefaultMs   = BACKWASH_MS;
   static uint32_t s_backwashActiveMs    = BACKWASH_MS;
   static uint32_t s_backwashReqMs       = 0;
   static constexpr uint8_t  DISP_MODE_TIME = 0;
   static constexpr uint8_t  DISP_MODE_PULSES = 1;

   static uint32_t s_dispenseDurationMs = 0;
   static uint16_t s_dispensePulseTarget = 0;
   static uint8_t  s_dispenseModeSel = DISP_MODE_TIME;
   static uint32_t s_dispensePulseStart = 0;
   static bool     s_uvFaultLatched = false;
   static constexpr uint32_t UV_PRECHECK_MS = 100UL;
   static constexpr uint32_t UV_POST_MS = 100UL;

   static bool dispenseProfileEnabled()
   {
      if (s_dispenseModeSel == DISP_MODE_TIME) return (s_dispenseDurationMs > 0);
      if (s_dispenseModeSel == DISP_MODE_PULSES) return (s_dispensePulseTarget > 0);
      return false;
   }

   static void enter(uint8_t st, uint32_t now)
   {
      s_state    = st;
      s_tStateMs = now;
   }

   static void markDispenseComplete()
   {
      // Keep this bounded for compact OLED/LCD rendering.
      if (s_dispenseSinceBw < 5) s_dispenseSinceBw++;
   }

   static void updateBackwash(uint32_t now, uint8_t waterLevel)
   {
      // Request semantics (toggle behavior):
      // - Request while idle: start if level is eligible.
      //   Normal mode requires wl > 1; override relaxes this to wl > 0.
      // - Request while active: stop immediately.
      if (s_backwashReq) {
         if (!s_backwashActive) {
            if (waterLevel > 1 || (s_overrideActive && waterLevel > 0)) {
               const uint32_t runMs = (s_backwashReqMs > 0) ? s_backwashReqMs : s_backwashDefaultMs;
               if (runMs > 0) {
                  s_backwashActive  = true;
                  s_bwStartMs       = now;
                  s_backwashActiveMs = runMs;
                  s_dispenseSinceBw = 0;
               }
               s_backwashReqMs = 0;
            } else {
               s_backwashReqMs = 0;
            }
         } else {
            s_backwashActive = false;
            s_backwashReqMs = 0;
         }
      }

      // Active stop conditions: duration elapsed or tank empty.
      if (s_backwashActive) {
         if ((uint32_t)(now - s_bwStartMs) >= s_backwashActiveMs) {
            s_backwashActive = false;
         } else if (waterLevel < 1) {
            s_backwashActive = false;
         }
      }

      // Consume request so each external pulse is processed once.
      s_backwashReq = false;
   }


   void begin()
   {
      s_state              = ST_IDLE;
      s_tStateMs           = 0;
      s_prevDispenseCmd    = false;

      s_dispenseSinceBw    = 0;
      s_backwashReq        = false;
      s_backwashActive     = false;
      s_bwStartMs          = 0;
      s_backwashDefaultMs  = BACKWASH_MS;
      s_backwashActiveMs   = BACKWASH_MS;
      s_backwashReqMs      = 0;

      s_overrideActive     = false;

      s_dispenseDurationMs = 0;
      s_dispensePulseTarget = 0;
      s_dispenseModeSel = DISP_MODE_TIME;
      s_dispensePulseStart = 0;
      s_uvFaultLatched = false;
   }

   void setOverrideActive(bool enable)
   {
      s_overrideActive = enable;
   }

   void requestBackwash()
   {
      s_backwashReq = true;
      s_backwashReqMs = 0;
   }

   void requestBackwashWithDuration(uint32_t durationMs)
   {
      s_backwashReq = true;
      s_backwashReqMs = durationMs;
   }

   void setBackwashDefaultDurationMs(uint32_t durationMs)
   {
      s_backwashDefaultMs = durationMs;
   }

   void setDispenseProfile(uint32_t durationMs, uint16_t pulses, uint8_t modeSel)
   {
      s_dispenseDurationMs = durationMs;
      s_dispensePulseTarget = pulses;
      s_dispenseModeSel = modeSel;
   }

   bool backwashActive()
   {
      return s_backwashActive;
   }

   uint32_t backwashRemainingMs(uint32_t now)
   {
      if (!s_backwashActive) return 0;
      const uint32_t elapsed = (uint32_t)(now - s_bwStartMs);
      if (elapsed >= s_backwashActiveMs) return 0;
      return (uint32_t)(s_backwashActiveMs - elapsed);
   }

   uint8_t state() { return s_state; }

   uint8_t dispenseCount() { return s_dispenseSinceBw; }

   bool takeDispenseUvFault()
   {
      const bool v = s_uvFaultLatched;
      s_uvFaultLatched = false;
      return v;
   }

   void abortDispense()
   {
      if (s_state == ST_DISPENSE || s_state == ST_UV_WAIT || s_state == ST_UV_POST) {
         enter(ST_IDLE, millis());
      }
      s_prevDispenseCmd = false;
      KioskIO::setWaterDispense(false);
      KioskIO::setKlaranUv(false);
   }

   void update(unsigned long nowUL, bool dispenseStart)
   {
      const uint32_t now = (uint32_t)nowUL;

      // Read current tank level (0..3).
      const uint8_t waterLevel = KioskIO::tankWaterLevel();

      // Backwash is an overlay: it can run independently of dispense state.
      updateBackwash(now, waterLevel);

      // Default actuator intent for dispense outputs.
      bool wantDispense = false;
      bool wantUv = false;

      // Edge-detect dispense start request.
      const bool dispenseEdge = (dispenseStart && !s_prevDispenseCmd);
      s_prevDispenseCmd = dispenseStart;

      switch (s_state) {

         case ST_IDLE: {
            // Start a dispense cycle on edge if allowed.
            if (dispenseEdge && (waterLevel > 0) && dispenseProfileEnabled()) {
               // Enable UV first; dispense starts only after UV precheck delay + UV_OK.
               wantUv = true;
               enter(ST_UV_WAIT, now);
            }
            break;
         }

         case ST_UV_WAIT: {
            wantUv = true;
            if ((uint32_t)(now - s_tStateMs) >= UV_PRECHECK_MS) {
               if (KioskIO::klaranUvOk()) {
                  wantDispense = true;
                  s_dispensePulseStart = KioskIO::dispenseFlowPulses();
                  enter(ST_DISPENSE, now);
               } else {
                  // UV failed readiness check; abort dispense and latch fault.
                  s_uvFaultLatched = true;
                  enter(ST_IDLE, now);
               }
            }
            break;
         }

         case ST_DISPENSE: {
            wantUv = true;
            wantDispense = true;

            // UV must remain healthy throughout dispense.
            if (!KioskIO::klaranUvOk()) {
               wantDispense = false;
               s_uvFaultLatched = true;
               enter(ST_IDLE, now);
               break;
            }

            bool dispenseDone = false;
            if (s_dispenseModeSel == DISP_MODE_PULSES) {
               if (s_dispensePulseTarget == 0) {
                  dispenseDone = true;
               } else {
                  const uint32_t pulses = KioskIO::dispenseFlowPulses() - s_dispensePulseStart;
                  dispenseDone = (pulses >= s_dispensePulseTarget);
               }
            } else if (s_dispenseModeSel == DISP_MODE_TIME) {
               if (s_dispenseDurationMs == 0) {
                  dispenseDone = true;
               } else {
                  dispenseDone = ((uint32_t)(now - s_tStateMs) >= s_dispenseDurationMs);
               }
            } else {
               // Unknown mode: fail safe by terminating dispense.
               dispenseDone = true;
            }

            // Dispense completion is mode-driven (time or pulses).
            if (dispenseDone) {
               wantDispense = false;
               markDispenseComplete();
               enter(ST_UV_POST, now);
            }
            break;
         }

         case ST_UV_POST: {
            // Keep UV enabled briefly after dispense completes.
            wantUv = true;
            if ((uint32_t)(now - s_tStateMs) >= UV_POST_MS) {
               enter(ST_IDLE, now);
               wantUv = false;
            }
            break;
         }

         default:
            enter(ST_IDLE, now);
            break;
      }

      // Apply final outputs once.
      KioskIO::setBackwash(s_backwashActive);
      KioskIO::setWaterDispense(wantDispense);
      KioskIO::setKlaranUv(wantUv);

      // Keep KioskIO time-based services (e.g., inlet kick->hold) running.
      KioskIO::service(now);
   }

} // namespace KioskHydraulics
