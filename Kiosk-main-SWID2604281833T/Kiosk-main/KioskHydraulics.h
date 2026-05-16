// KioskHydraulics.h
// -----------------------------------------------------------------------------
// Low-level hydraulic actuator state machine built on top of KioskIO.*
//
// Scope of this module:
// - Executes dispense cycles (time- or pulse-based).
// - Runs/cancels timed backwash requests.
//
// Out of scope (handled by KioskBehavior):
// - Button choreography and override semantics.
// - DayEnd sequencing and queued actions.
// - Periodic auto features and higher-level policy arbitration.
// -----------------------------------------------------------------------------

#pragma once
#include <Arduino.h>

namespace KioskHydraulics {

   // Call once from setup().
   void begin();

   // Inform hydraulics of override mode (affects backwash eligibility).
   void setOverrideActive(bool enable);

   // Request a backwash to start as soon as allowed.
   void requestBackwash();
   // Request a backwash to start with a one-shot duration override (ms).
   void requestBackwashWithDuration(uint32_t durationMs);
   // Set the default backwash duration (ms) for future requests.
   // 0 disables default-duration backwash starts.
   void setBackwashDefaultDurationMs(uint32_t durationMs);

   // Configure dispense completion behavior (from EEPROM).
   // modeSel matches KioskEeprom::WdMode (0=time, 1=pulses).
   void setDispenseProfile(uint32_t durationMs, uint16_t pulses, uint8_t modeSel);

   // True while backwash is active.
   bool backwashActive();
   // Remaining time for active backwash (ms). Returns 0 when inactive/expired.
   uint32_t backwashRemainingMs(uint32_t now);

   // Main update; call once per loop().
   // - `dispenseStart`: dispense start request; edge-detected internally.
   void update(unsigned long now, bool dispenseStart);

   // Optional introspection of the dispense sequence state (0=IDLE).
   uint8_t state();

   // Returns the number of completed dispense cycles since the last backwash start (0..5).
   uint8_t dispenseCount();

   // Returns true once when a UV fault occurred during dispense sequencing.
   bool takeDispenseUvFault();

   // Force-stop an active dispense cycle immediately.
   void abortDispense();

} // namespace KioskHydraulics
