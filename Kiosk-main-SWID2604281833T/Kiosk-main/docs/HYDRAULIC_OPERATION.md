# Hydraulic Operation

This document describes how the firmware activates and operates:

1. `Backwash`
2. `SensorBypass`
3. `WaterInp` (inlet solenoid)

It reflects current behavior implemented in:

1. `KioskBehavior.cpp`
2. `KioskHydraulics.cpp`
3. `Kiosk.ino` (status rendering)

## 1) Input Signals and Operating Context

The control logic uses:

1. `water_level` from `KioskIO::tankWaterLevel()` (`0..3`).
2. Buttons:
3. `BTN_BACKWASH_CTL`
4. `BTN_SENSOR_BYPASS`
5. `BTN_WATER_INLET`
6. Mode switch:
7. `NORMAL` / `OVERRIDE` (`KioskIO::overrideActive()`).
8. EEPROM parameters:
9. `BW AutoDuration`
10. `BW AutoAfter N`
11. `BW Daily Time`
12. `BW Daily Duratn`
13. `SensByp Duration` (`100 ms/LSB`)

## 2) Global Precedence and Interlocks

The following constraints are always enforced:

1. Inlet is forced OFF when `water_level > 2`.
2. Inlet ON demand is suppressed while `Backwash` is active.
3. Inlet ON demand is suppressed while `SensorBypass` is active.
4. `OVERRIDE` mode allows source-button overrides that terminate conflicting active functions.
5. `NORMAL` mode uses hold-off behavior rather than forced termination in conflict cases.

## 3) Backwash

### 3.1 Start Sources

Backwash can be requested by:

1. Manual button (`BTN_BACKWASH_CTL`).
2. Auto-after-dispense logic:
3. `BW DispCount` increments after each completed dispense.
4. When `BW AutoAfter N != 0` and count reaches/exceeds threshold:
5. Counter resets to `0`.
6. Auto backwash request is queued.
7. RTC daily trigger:
8. Active only if RTC is present.
9. Current time must be within the first hour after `BW Daily Time`.
10. `BW Daily Duratn` must be non-zero.

### 3.2 Start Eligibility by Mode

1. `NORMAL`: start allowed if `water_level > 1`.
2. `OVERRIDE`: start allowed if `water_level > 0`.

### 3.3 Interaction with SensorBypass

1. If SensorBypass is active and Backwash is requested:
2. `NORMAL`: Backwash is held pending until SensorBypass clears.
3. `OVERRIDE` and request source is Backwash button:
4. SensorBypass is terminated first.
5. Backwash then starts (if eligible).

### 3.4 Active and Stop Behavior

1. Backwash runs for configured duration.
2. A second Backwash button press while active requests stop (toggle-off behavior).
3. Hydraulics layer also stops Backwash on safety/timeout conditions.
4. On completion, `BW DispCount` is reset to `0`.

### 3.5 RTC Stamping

1. Daily RTC-triggered Backwash updates the Daily stamp.
2. Non-daily Backwash starts update the Triggered stamp.

## 4) SensorBypass

### 4.1 Activation and Timing

1. `BTN_SENSOR_BYPASS` toggles SensorBypass.
2. When SensorBypass turns ON, duration is loaded from EEPROM:
3. `SensByp Duration` in `100 ms` units.
4. Auto-expiry timestamp is set at activation time.
5. SensorBypass auto-terminates when expiry is reached.
6. Button press while active toggles it OFF immediately.

### 4.2 Interaction with Backwash

1. If Backwash is active when SensorBypass button is pressed:
2. `NORMAL`: SensorBypass toggle is held pending until Backwash clears.
3. `OVERRIDE`: active Backwash is terminated, then SensorBypass toggle is applied.

### 4.3 Operational Effect

1. While active, inlet ON demand is blocked.
2. Any Backwash request held due to SensorBypass can start after SensorBypass clears (if still eligible).

## 5) WaterInp (Inlet Solenoid)

### 5.1 Startup Initialization

At first control loop after boot, inlet behavior is level-driven via an auto-refill latch:

1. If `water_level < 2`, auto-refill latch is ON.
2. If `water_level > 2`, auto-refill latch is OFF.
3. At `water_level == 2`, latch keeps prior state.

### 5.2 Button-Driven State

`BTN_WATER_INLET` implements a manual latch plus timed hold:

1. Press when OFF or HLD and `water_level < 3`:
2. Set inlet demand ON.
3. Clear any active hold.
4. Press when ON:
5. Enter `HLD` (forced OFF hold) for up to 1 hour.
6. Hold auto-expires after 1 hour.

### 5.3 Safety Rule

1. If `water_level > 2`, inlet output is forced OFF.
2. At the same time, auto-refill latch is forced OFF.
3. Auto operation resumes only after level drops below 2 (which re-arms the auto-refill latch).

### 5.4 Mode-Specific Override

1. `NORMAL`:
2. Inlet button does not terminate active Backwash/SensorBypass.
3. Inlet remains blocked while those are active.
4. `OVERRIDE`:
5. Inlet button first terminates active SensorBypass and active Backwash.
6. If the inlet was being suppressed by Backwash and/or SensorBypass, that same press re-enables inlet demand (clears HLD and sets ON intent), provided `water_level < 3`.
7. Otherwise, normal toggle/hold behavior applies.

### 5.5 Final Inlet ON Condition

Inlet is physically commanded ON only when all are true:

1. Auto-refill latch OR manual inlet demand is ON.
2. Hold (`HLD`) is not active.
3. `water_level < 3`.
4. Backwash is not active.
5. SensorBypass is not active.

### 5.6 PWM Drive Profile

When inlet is commanded ON, `KioskIO` applies two-phase PWM:

1. Start phase at configured start PWM.
2. After configured switch delay, transition to hold PWM.
3. OFF command sets PWM to `0`.

## 6) OLED State Reporting (Hydraulics Screen)

Relevant live status rows:

1. SensorBypass row: `OFF / PEND / ON` plus countdown when ON.
2. Backwash row: `OFF / PEND / ON` plus countdown when ON.
3. WaterInp row:
4. `ON` with elapsed ON time.
5. `HLD` with count-up in minutes while hold is active.
6. `OFF` otherwise.

## 7) Summary of Conflict Resolution

1. Inlet never bypasses `water_level > 2` safety.
2. SensorBypass and Backwash suppress inlet output.
3. In `NORMAL`, conflicts are generally deferred (pending).
4. In `OVERRIDE`, source-button actions can terminate conflicting active states immediately.
