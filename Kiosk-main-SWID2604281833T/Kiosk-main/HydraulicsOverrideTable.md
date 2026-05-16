# Hydraulics Override Mode Summary
#
# Printable table of hydraulics logic in override mode.
# Source: KioskBehavior.cpp, KioskHydraulics.cpp.

Input / Condition                                     Outputs                 Action (override mode)
---------------------------------------------------  ----------------------  --------------------------------------------------------------
overrideActive == true                                (global behavior)       Payment checks are bypassed; dispense requests are accepted.
inletTogglePulse                                     setWaterInlet(...)       Toggle manual inlet: ON allowed when wl != 3 and backwash not
                                                                              active; OFF allowed at any level.
wl == 3 (FULL)                                        setWaterInlet(false)     Manual inlet ON latch clears at FULL; auto-refill latch clears.
wl < 1 (EMPTY)                                        setWaterInlet(...)       Manual inlet OFF latch clears when empty.
wl < 2                                               setWaterInlet(...)       Auto-refill latch turns ON (inlet demand true).
backwashReqPulse                                     setBackwash(...)         Start if wl > 0; stop on second press; ignore if wl == 0.
backwash active                                      setBackwash(false)       Auto-stop on timeout (BACKWASH_MS) or wl < 1.
backwash active                                      setWaterInlet(false)     Inlet forced OFF while backwash is active.
sensorBypassTogglePulse                              setSensorBypass(...)     Toggle immediately unless backwash active or starting; then deferred.
frontDispensePulse OR rearDispensePulse              setWaterDispense(...)    Either pulse requests dispense (no payment check).
contDispenseTogglePulse                              setWaterDispense(...)    Continuous dispense toggles ON/OFF (override only).
s_contDispense && hydraulics state == IDLE && wl > 0 setWaterDispense(true)   Auto-requests dispense while continuous mode is ON.
dispense start edge && wl > 0                         setWaterDispense(true)   Dispense allowed when tank not empty; runs per measured profile (time or pulses).

Notes:
- wl is tank water level: 0=EMPTY, 1=LOW, 2=MID, 3=FULL.
- Backwash start threshold in override is wl > 0 (vs wl > 1 normally).
- Inlet demand is (auto-refill OR manual ON) AND not manual OFF; inlet is suppressed during backwash.
