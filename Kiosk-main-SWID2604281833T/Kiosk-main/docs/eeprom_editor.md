# EEPROM Editor

This kiosk firmware includes a standalone EEPROM editor module for field setup and diagnostics.

## Entry

The editor is entered on boot when the **Backwash** + **Water Sensor Bypass** buttons are held together
(active-low). The firmware displays a prompt and waits for the buttons to be released before entering
edit mode.

**Important:** The EEPROM editor must remain a **boot‑chord‑only** feature. It should not be exposed
as a normal runtime menu or invoked from regular UI flows.

## Exit behavior

The editor is **non-returning**: it exits only via reset/watchdog (e.g., long BACK or explicit reset
paths inside the editor). Control does not return to the main application after `run()`.

## Integration points

- Files: `KioskEepromEditor.h`, `KioskEepromEditor.cpp`
- Call site: `Kiosk.ino` (boot-chord gating and `run()` invocation)

## Notes

- Displays: OLED (U8x8 text mode) and 16x4 I2C LCD
- Buttons are active-low with external pullups and RC debounce
- PN532 is held in reset except during NFC scan operations
- RAM usage is static for the lifetime of the program; the editor does not return
