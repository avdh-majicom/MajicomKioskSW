# Majicom Kiosk Embedded Software

Firmware for the Majicom water kiosk, targeting the Arduino MEGA 2560.

The software controls the customer dispense flow, rear-operator service interface,
hydraulic sequencing, NFC payment/token handling, and EEPROM-backed configuration.
The current implementation is centred around these modules:

- `Kiosk.ino`: boot flow, button handling, display updates, service-mode entry
- `KioskBehavior.*`: customer UI state machine and high-level hydraulic policy
- `KioskHydraulics.*`: low-level dispense and backwash state machine
- `KioskIO.*`: hardware I/O, RTC, PN532 NFC, and actuator helpers
- `KioskEeprom.*`: EEPROM layout, defaults, migrations, counters, and token storage
- `KioskEepromEditor.*`: boot-chord-only EEPROM editor

## Overview

`Kiosk.ino` is the sketch entry point. It initialises hardware I/O, EEPROM access,
the UI state machine, and the hydraulics policy. The main loop polls inputs,
updates the UI, applies hydraulics decisions, and reports status to the
displays/serial console.

## Module Map

- `Kiosk.ino`: demo harness, button edge detection, UI test harness, display updates
- `KioskIOpins.h`: canonical pin map and safe init table
- `KioskIO.*`: hardware facade (pins, actuators, inlet kick/hold timing, PN532 NFC)
- `KioskHydraulics.*`: inlet/backwash/dispense policy state machine
- `KioskBehavior.*`: high-level behavior and UI state machine; payment gating; drives hydraulics requests
- `KioskEeprom.*`: EEPROM layout, defaults, migrations, counters, NFC token registry
- `KioskEepromEditor.*`: standalone EEPROM editor; must remain boot-chord-only (Backwash + Sensor Bypass at boot)

## Data Flow

1. Inputs and test harness flags are gathered in `Kiosk.ino`.
2. UI inputs feed `KioskBehavior::Ui::uiUpdate()`, which returns LCD/LED/backlight intents.
3. Hydraulics requests are consolidated in `KioskBehavior::updateHydraulics()`, which delegates to `KioskHydraulics::update()`.
4. `KioskHydraulics` drives `KioskIO` actuators; `KioskIO` performs the physical writes and services time-based I/O.
5. NFC payment flows through `KioskBehavior::KioskPayment()` -> `KioskIO::readNfc()` -> EEPROM token registry.

## UI State Machine

The canonical UI state diagram is `docs/ui_state_machine_lcd.dot` (source) and
`docs/ui_state_machine_lcd.png` (rendered). States are defined in
`KioskBehavior::Ui::UiState`, with doc codes via `docStateName()`.

## EEPROM At A Glance

EEPROM is used for both live operating parameters and system-managed records.
In the current firmware, the settings that actively affect runtime behaviour are:

- dispense profile: measured duration, pulse target, and dispense mode
- inlet solenoid tuning: kick PWM, hold PWM, and kick duration
- backwash behaviour: default duration, auto-after-N, daily schedule, manual short/long durations
- sensor-bypass behaviour: timed duration, periodic schedule, manual short/long durations
- water-circulation behaviour: auto and manual durations
- periodic nozzle flush timing and beep settings
- NFC token table for accepted token-based payments

Some EEPROM fields are maintained by the firmware rather than used by the operator
controls, including dispense counters, the backwash dispense counter, and backwash
timestamp metadata, and the EEPROM layout magic marker.

The current code also stores several fields that are editor-visible but not yet
applied by runtime logic. These include:

- non-inlet solenoid profiles
- UV timing parameters
- NFC timing parameters
- water-temperature sensor selection
- pre-dispense circulation and purge settings

Two implementation caveats matter when reading or editing EEPROM values:

- `coinAcceptorFitted` currently affects UI wording, but the main payment path is effectively NFC-or-override in the current build
- end-of-day recirculation and backwash durations are not stored in the current EEPROM layout and currently fall back to fixed compatibility values

For the full field-by-field breakdown, see
[`docs/EEPROM_RUNTIME_USAGE_SUMMARY.md`](Kiosk-main-SWID2604281833T/Kiosk-main/docs/EEPROM_RUNTIME_USAGE_SUMMARY.md).

## Build Target

- Board: Arduino MEGA 2560
- FQBN: `arduino:avr:mega`
- Main sketch: `Kiosk.ino`
- Serial baud: `250000`
- Helper scripts: `scripts/build.sh`, `scripts/upload.sh`, `scripts/monitor.sh`
- `justfile` tasks: `build`, `upload`, `monitor`

Build note: `scripts/build.sh` refreshes the `ver:YYMMDDHHMMSS` timestamp in any
module being built before compiling. When running via Codex, build and upload
require unsandboxed device access.

## Key Docs

- [Architecture](Kiosk-main-SWID2604281833T/Kiosk-main/docs/architecture.md)
- [System Operation](Kiosk-main-SWID2604281833T/Kiosk-main/docs/SYSTEM_OPERATION.md)
- [EEPROM Runtime Usage Summary](Kiosk-main-SWID2604281833T/Kiosk-main/docs/EEPROM_RUNTIME_USAGE_SUMMARY.md)
- [EEPROM Editor](Kiosk-main-SWID2604281833T/Kiosk-main/docs/eeprom_editor.md)
