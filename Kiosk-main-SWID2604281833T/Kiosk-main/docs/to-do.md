# To-Do

- EEPROM Editor UI consistency: adjust the remaining 4 screens that still use row 12 for value/status output so they follow the centered row 13 pattern used by other parameter screens.
  - `LAYER_PCB`
  - `LAYER_SOL_MENU` / `LAYER_SOL_PARAM_EDIT`
  - `LAYER_NFC_MENU` / `LAYER_NFC_BROWSE` / `LAYER_NFC_ADD` / `LAYER_NFC_DEL`
  - `LAYER_RTC_CONFIG` (currently intentionally uses row 12; decide whether to keep or align)
- UI progress bar: refine `UI_DISPENSING` row 3 progress rendering so it correctly reflects both time-based and pulse-based dispensing modes.
- Hourly squirt behavior: when not in dark mode, trigger a squirt once per hour and beep 5 times (0.5s ON every 2s).
- Move code to Majicom GD after commit.
