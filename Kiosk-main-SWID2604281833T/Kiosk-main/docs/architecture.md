# Architecture

## Overview
Kiosk.ino is the sketch entry point. It initializes hardware I/O, EEPROM access, the UI state machine, and the hydraulics policy. The main loop polls inputs, updates the UI, applies hydraulics decisions, and reports status to the displays/serial console.

## Module map
- Kiosk.ino: demo harness, button edge detection, UI test harness, display updates.
- KioskIOpins.h: canonical pin map and safe init table.
- KioskIO.*: hardware facade (pins, actuators, inlet kick/hold timing, PN532 NFC).
- KioskHydraulics.*: inlet/backwash/dispense policy state machine.
- KioskBehavior.*: high-level behavior and UI state machine; payment gating; drives hydraulics requests.
- KioskEeprom.*: EEPROM layout, defaults, migrations, counters, NFC token registry.
- KioskEepromEditor.*: standalone EEPROM editor; must remain boot-chord-only (Backwash + Sensor Bypass at boot).

## Data flow
1) Inputs and test harness flags are gathered in Kiosk.ino.
2) UI inputs feed KioskBehavior::Ui::uiUpdate(), which returns LCD/LED/backlight intents.
3) Hydraulics requests are consolidated in KioskBehavior::updateHydraulics(), which delegates to KioskHydraulics::update().
4) KioskHydraulics drives KioskIO actuators; KioskIO performs the physical writes and services time-based IO.
5) NFC payment: KioskBehavior::KioskPayment() -> KioskIO::readNfc() -> EEPROM token registry.

## UI state machine
The canonical UI state diagram is `docs/ui_state_machine_lcd.dot` (source) and `docs/ui_state_machine_lcd.png` (rendered).
States are defined in KioskBehavior::Ui::UiState, with doc codes via docStateName().

## Tooling
- Target board: Arduino MEGA 2560 (arduino:avr:mega).
- Serial baud: 250000.
- Helper scripts: scripts/build.sh, scripts/upload.sh, scripts/monitor.sh.
- justfile tasks: build, upload, monitor (override PORT/FQBN/BUILD_DIR/SKETCH as needed).
- Build script behavior: scripts/build.sh refreshes the `ver:YYMMDDHHMMSS` timestamp in any module being built (e.g., `Kiosk.ino`) before compiling.
- When running via Codex, build and upload should be executed with unsandboxed device access.
- Note: uploads from sandboxed environments (like Codex) require unsandboxed device access.
