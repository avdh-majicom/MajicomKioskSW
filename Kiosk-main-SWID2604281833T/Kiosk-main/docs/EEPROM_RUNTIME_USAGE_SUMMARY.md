# EEPROM Runtime Usage Summary

This note maps EEPROM-backed fields to the current firmware behavior.
Where older docs and the implementation differ, the implementation is treated as
the source of truth.

Primary code paths:
- `Kiosk.ino`
- `KioskBehavior.cpp`
- `KioskHydraulics.cpp`
- `KioskIO.cpp`
- `KioskEeprom.cpp`

## Status Labels

| Label | Meaning |
| --- | --- |
| Active | Current runtime reads the field and behavior changes. |
| Partial | Current runtime uses the field for setup, UI, or diagnostics, but not as a full process-control input. |
| System | Firmware maintains the field automatically; it is not mainly an operator control setting. |
| Unused | Field is stored and may be editor-visible, but current runtime does not consume it. |

## Active Or Partially Active Operator Settings

| Group | EEPROM addr | Status | Current effect | Primary consumer |
| --- | --- | --- | --- | --- |
| `HWID` | `0` | Partial | Used at boot to force setup flow when `HWID == 0`; also used in prompts and diagnostics. | `Kiosk.ino` |
| Measured dispense duration, pulses, mode | `1-5` | Active | Defines the normal customer dispense profile. Periodic nozzle flush temporarily overrides this with its own timed profile. | `Kiosk.ino`, `KioskBehavior.cpp`, `KioskHydraulics.cpp` |
| Inlet solenoid profile: start PWM, hold PWM, switch delay | `20-22` | Active | Sets inlet kick PWM, hold PWM, and kick duration at boot. | `Kiosk.ino`, `KioskIO.cpp` |
| Backwash auto duration | `31` | Active | Default backwash run time for requests that do not carry a manual duration override. `0` disables default-duration starts. | `KioskBehavior.cpp`, `KioskHydraulics.cpp` |
| Backwash auto-after-N | `32` | Active | Triggers auto backwash after the configured number of completed dispenses. | `KioskBehavior.cpp` |
| Daily backwash time and duration | `43-45` | Active | Schedules RTC-based daily backwash when the RTC is present and the kiosk is eligible to run it. | `KioskBehavior.cpp`, `KioskIO.cpp` |
| Sensor bypass auto duration and period | `51-52` | Active | Controls timed sensor bypass and periodic auto sensor-bypass scheduling. | `KioskBehavior.cpp` |
| Water circulation auto and manual durations | `54-57` | Active | Controls automatic recirculation and short/long manual circulation timing. | `KioskBehavior.cpp` |
| Backwash manual short and long durations | `62-63` | Active | Rear backwash button press length selects the short or long manual backwash duration. | `KioskBehavior.cpp` |
| Sensor bypass manual short and long durations | `64-65` | Active | Rear sensor-bypass button press length selects the short or long manual bypass duration. | `KioskBehavior.cpp` |
| Periodic flush timing and beep settings | `66-71` | Active | Controls automatic nozzle flush cadence, pre-alert timing, beep pattern, and flush dispense duration. | `KioskBehavior.cpp` |
| Coin acceptor fitted | `46` | Partial | Changes pay-prompt UI wording and the `hasCoinAcceptor` flag, but current payment acceptance logic does not process coin input in `KioskPayment()`. | `Kiosk.ino`, `KioskBehavior.cpp` |
| NFC token table | dynamic, end of EEPROM | Active | Stores accepted NFC token hashes used for token-based payment authorization. | `KioskBehavior.cpp`, `KioskEeprom.cpp` |

## System-Managed EEPROM Data

| Group | EEPROM addr | Status | Current effect | Primary consumer |
| --- | --- | --- | --- | --- |
| Backwash dispense counter | `33-34` | System | Tracks dispenses since the last backwash. Used for auto-after-N logic and reset when a backwash completes. | `KioskBehavior.cpp` |
| Daily and triggered backwash year/month metadata | `47-50` | System | Stores year/month companions for RTC alarm-register timestamps used to record the last daily and last triggered backwash times. | `KioskIO.cpp` |
| Layout magic | `124-127` | System | Determines whether EEPROM is valid for the current schema and whether migration or reinitialization is required. | `KioskEeprom.cpp` |
| Dispensed counters: app, coin, NFC, override | `128-143` | System | Displayed in the boot/counters screen. App, NFC-token, and override counters are incremented by the current runtime; the coin counter remains present in EEPROM/UI but current coin acceptance is not implemented in `KioskPayment()`. | `KioskBehavior.cpp`, `Kiosk.ino` |

## Stored But Not Currently Consumed By Runtime

| Group | EEPROM addr | Status | Current note |
| --- | --- | --- | --- |
| Backwash solenoid profile | `10-12` | Unused | Runtime currently drives the backwash solenoid at fixed full PWM when active. |
| Sensor-bypass solenoid profile | `15-17` | Unused | Runtime currently drives the sensor-bypass solenoid at fixed full PWM when active. |
| Dispense solenoid profile | `25-27` | Unused | Runtime currently opens the dispense solenoid at fixed full PWM during dispense. |
| Legacy solenoid reserved bytes | `13-14`, `18-19`, `23-24`, `28-29` | Unused | Kept only for compatibility with older EEPROM layouts. |
| Deprecated backwash frequency | `30` | Unused | Kept reserved for compatibility. |
| UV OK delay | `37` | Unused | Runtime does not currently read this value. Dispense uses a fixed UV precheck delay in `KioskHydraulics.cpp`. |
| UV MAX ON time | `38` | Unused | No runtime maximum-on timer is currently enforced. |
| Water temperature sensor select | `39` | Unused | Stored and editor-visible, but no runtime behavior currently reads it. |
| NFC init delay, scan duration, inter-read delay | `40-42` | Unused | `KioskIO::readNfc()` currently uses hardcoded timing values and does not read these EEPROM fields. |
| Deprecated timed-dispense averaging count | `53` | Unused | Reserved only. |
| Pre-dispense circulation and purge settings | `58-61` | Unused | Stored and editor-visible, but the current hydraulics flow does not apply a pre-dispense circulation or purge phase. |

## Not In The Current EEPROM Layout

- End-of-day recirculation duration is not stored in EEPROM in the current layout.
  `KioskEeprom::eodCircDuration15s()` currently returns a fixed compatibility fallback
  of 30 minutes.
- End-of-day backwash duration is not stored in EEPROM in the current layout.
  `KioskEeprom::eodBackwashDuration1s()` currently returns a fixed compatibility
  fallback of 3 minutes.
- The RTC clock itself is not EEPROM-backed. The editor's RTC page writes the DS3231
  directly; EEPROM only stores the year/month companion bytes for the two backwash
  timestamp records.

## Practical Summary

- If the goal is to tune live kiosk behavior today, the fields that matter most are
  the dispense profile, inlet profile, backwash settings, sensor-bypass settings,
  water-circulation settings, periodic flush settings, and the NFC token table.
- If the goal is to clean up the firmware, the largest editor-to-runtime gaps are the
  non-inlet solenoid profiles, UV timing fields, NFC timing fields, water-temperature
  sensor select, and the pre-dispense circulation/purge fields.
- Coin handling is currently only partially implemented: the UI and counters still
  model coin sales, but the main payment path in `KioskPayment()` is effectively
  NFC-or-override in the current build.
