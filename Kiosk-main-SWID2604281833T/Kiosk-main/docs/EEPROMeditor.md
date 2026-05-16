# EEPROMeditor (EEPROM Editor Module)

This firmware includes a standalone, boot-chord-only EEPROM editor intended for field setup and diagnostics.

## Location

- Editor UI: `KioskEepromEditor.h`, `KioskEepromEditor.cpp`
- EEPROM access module: `KioskEeprom.h`, `KioskEeprom.cpp`
- Private EEPROM schema (addresses, magic, defaults): `KioskEepromLayout.h`
- Boot gating and invocation: `Kiosk.ino`

## Summary (Structure And Operation)

- Public entrypoint is non-returning: `Kiosk::KioskEepromEditor::run(const Config&)`.
- `Config` injects all required hardware handles and policy:
  - Required: `KioskEeprom* ee`, `U8X8* oled`, `hd44780_I2Cexp* lcd`
  - Optional: `Adafruit_PN532* nfc`, and `pinReinitGuard` for an emergency reset chord
  - Button pins are active-low and assumed to have external pullups and RC debounce
  - PN532 is held in reset except during explicit scans to prevent I2C bus lockups
- Internals are a UI state machine:
  - A `currentLayer` (`LAYER_*`) selects the active screen or mode.
  - Cursors track the currently-highlighted menu entry per menu-layer.
  - Edit screens load EEPROM-backed values into caches on entry (`onEnterLayer()`), set dirty flags during edits, and write back only on long SELECT (`saveIfDirtyForLayer()`).
  - OLED redraw uses a per-row cache to minimize flicker; LCD shows a fixed frame, with line1 reserved for transient overlays.
- Entry is boot-only and gated in `Kiosk.ino`:
  - Auto-entry if EEPROM magic is invalid or HWID is 0.
  - Manual entry if the boot chord is held (Sensor Bypass + Backwash, active-low), with a “release buttons” prompt before entering.
- Exit is reset-only:
  - Long BACK triggers watchdog reset.
  - Some modes consume BACK to avoid accidental long-press reset (notably NFC add/del).

## Controls

- `UP` and `DOWN`: navigate menus; in edit layers UP increases and DOWN decreases.
- Short `SELECT`: enter a menu, advance field selection, or toggle values (layer-specific).
- Long `SELECT` (about 3s): save current edit layer (writes to EEPROM), or confirm destructive actions.
- Short `BACK`: go back one layer (with special handling for modal prompts).
- Long `BACK` (about 4s): watchdog reset (exit).

## Exact Menu Tree (Layers And Edits)

All menu flows below are implemented by `shortSelect()` transitions and layer-specific handlers in `KioskEepromEditor.cpp`.

| Menu Path | Layer(s) | What It Shows | What It Edits (Backed By) | Commit Behavior |
| --- | --- | --- | --- | --- |
| Top: `PCB HW ID` | `LAYER_TOP` -> `LAYER_PCB` | Current HWID text | `KioskEeprom::hwId()` | Long SELECT writes `setHwId(hwid)` |
| Top: `Water Disp Ctrl` | `LAYER_TOP` -> `LAYER_WD_MENU` -> `LAYER_WD_EDIT` | Measured dispense profile field value | `KioskEeprom::MeasuredProfile` fields: duration (100ms units), pulses, modeSel | Long SELECT writes `storeMeasuredProfile(profile)` |
| Water Disp Ctrl field: `Timed Dispense` | `LAYER_WD_MENU`/`LAYER_WD_EDIT` | `X.Y s` | `MeasuredProfile.duration100ms` | Long SELECT saves (see above) |
| Water Disp Ctrl field: `Pulse Count` | `LAYER_WD_MENU`/`LAYER_WD_EDIT` | `N pulses` | `MeasuredProfile.pulses` | Long SELECT saves (see above) |
| Water Disp Ctrl field: `Disp Ctrl Sel` | `LAYER_WD_MENU`/`LAYER_WD_EDIT` | `Mode=TIME` or `Mode=PULSES` | `MeasuredProfile.modeSel` | Long SELECT saves (see above) |
| Top: `Dispensed Units` | `LAYER_TOP` -> `LAYER_DISPENSED_MENU` | Per-counter values and total | Counters: `dispenseCounter(idx)`; total: `totalDispensedUnits()` | Edits are only for the first 4 counters; long SELECT writes `setDispenseCounter(idx, v)` |
| Dispensed: `App Disp` | `LAYER_DISPENSED_MENU` -> `LAYER_DISPENSED_EDIT` | `<value> dispensed` | `DispenseCounter::DISP_APP` | Long SELECT commits `setDispenseCounter(DISP_APP, v)` |
| Dispensed: `Coin Disp` | `LAYER_DISPENSED_MENU` -> `LAYER_DISPENSED_EDIT` | `<value> dispensed` | `DispenseCounter::DISP_COIN` | Long SELECT commits `setDispenseCounter(DISP_COIN, v)` |
| Dispensed: `NFC Token Disp` | `LAYER_DISPENSED_MENU` -> `LAYER_DISPENSED_EDIT` | `<value> dispensed` | `DispenseCounter::DISP_NFC` | Long SELECT commits `setDispenseCounter(DISP_NFC, v)` |
| Dispensed: `Bypassed Disp` | `LAYER_DISPENSED_MENU` -> `LAYER_DISPENSED_EDIT` | `<value> dispensed` | `DispenseCounter::DISP_BYPASS` | Long SELECT commits `setDispenseCounter(DISP_BYPASS, v)` |
| Dispensed: `TOTAL Dispensed` | `LAYER_DISPENSED_MENU` | `TOTAL:<sum>` | Read-only computed sum | No edit layer is entered |
| Top: `KioskParm Edit` | `LAYER_TOP` -> `LAYER_KIOSKPARM_MENU` | Category menu | N/A | N/A |
| KioskParm: `Solenoid Config` | `LAYER_KIOSKPARM_MENU` -> `LAYER_SOL_MENU` -> `LAYER_SOL_PARAM_EDIT` | Per-solenoid PWM profile parameter | `KioskEeprom::PwmSolenoidProfile` for selected solenoid | Long SELECT writes `storeSolenoidProfile(solenoid, profile)` |
| Solenoid parameter: `StartPWM` | `LAYER_SOL_PARAM_EDIT` | `StartPWM=<0..255>` | `PwmSolenoidProfile.startPwm` | Long SELECT saves profile |
| Solenoid parameter: `HoldPWM` | `LAYER_SOL_PARAM_EDIT` | `HoldPWM=<0..255>` | `PwmSolenoidProfile.holdPwm` | Long SELECT saves profile |
| Solenoid parameter: `SwOnDelay` | `LAYER_SOL_PARAM_EDIT` | `SwOnDelay=<secs>` | `PwmSolenoidProfile.swDelaySec` | Long SELECT saves profile |
| KioskParm: `KlaranUV Config` | `LAYER_KIOSKPARM_MENU` -> `LAYER_KLARAN_MENU` -> `LAYER_KLARAN_EDIT` | UV OK delay or max on-time | `klaranUvOkDelay10ms()`, `klaranUvMaxOntimeMinutes()` | Long SELECT writes both `setKlaranUvOkDelay10ms(v)` and `setKlaranUvMaxOntimeMinutes(v)` |
| KioskParm: `WaterTempSensor` | `LAYER_KIOSKPARM_MENU` -> `LAYER_WATERTEMP_EDIT` | `Use T1` or `Use T2` | `waterTempSense()` (1 or 2) | Long SELECT writes `setWaterTempSense(v)` |
| KioskParm: `CoinAcc Config` | `LAYER_KIOSKPARM_MENU` -> `LAYER_COINACC_EDIT` | `Fitted ? YES/NO` | `coinAcceptorFitted()` | Long SELECT writes `setCoinAcceptorFitted(bool)` |
| KioskParm: `Backwash Config` | `LAYER_KIOSKPARM_MENU` -> `LAYER_BACKWASH_MENU` -> `LAYER_BACKWASH_EDIT` | Backwash parameter value | `backwashDuration()`, `backwashAfterNDispenses()`, `backwashDispenseCounter()`, daily time/duration | Long SELECT writes the currently-edited parameter via its setter |
| Backwash: `BW Duration` | `LAYER_BACKWASH_EDIT` | `Dur=<secs>` (10s steps) | `setBackwashDuration(u8_10s)` | Long SELECT commits |
| Backwash: `BW After N` | `LAYER_BACKWASH_EDIT` | `AfterN=<N>` | `setBackwashAfterNDispenses(u8)` | Long SELECT commits |
| Backwash: `BW DispCount` | `LAYER_BACKWASH_EDIT` | `Count=<u16>` | `setBackwashDispenseCounter(u16)` | Long SELECT commits |
| Backwash: `BW Daily Time` | `LAYER_BACKWASH_EDIT` | `Time=HH:MM` | `setDailyBackwashTimeMinutes(u16)` | Long SELECT commits |
| Backwash: `BW Daily Dur` | `LAYER_BACKWASH_EDIT` | `Disabled` or `Dur=<secs>` (10s steps) | `setDailyBackwashDuration10s(u8)` | Long SELECT commits |
| KioskParm: `RTC Config` | `LAYER_KIOSKPARM_MENU` -> `LAYER_RTC_CONFIG` | DS3231 date/time and caret for active cluster | DS3231 registers (I2C), not EEPROM | Short SELECT advances cluster and writes the full time to DS3231; BACK moves to prior cluster or exits |
| KioskParm: `NFC Params Mgt` | `LAYER_KIOSKPARM_MENU` -> `LAYER_NFC_PARMS_MENU` -> `LAYER_NFC_PARMS_EDIT` | NFC reader timing (ms) | `nfcInitDelayMs`, `nfcReadDurationMs`, `nfcInterNfcDelayMs` | Long SELECT writes `setNfcInitDelayMs`, `setNfcReadDurationMs`, `setNfcInterNfcDelayMs` |
| Top: `NFC Token Mgt` | `LAYER_TOP` -> `LAYER_NFC_MENU` | Token operations menu | `KioskEeprom` token hash table | Some actions are immediate; some require confirm |
| NFC: `Browse/Delete` | `LAYER_NFC_MENU` -> `LAYER_NFC_BROWSE` | `I### <hash>` or `Del? <hash> YES/NO` | Deletes selected token hash when confirmed | Short SELECT toggles delete prompt; long SELECT deletes if confirmed |
| NFC: `NFC Add Token` | `LAYER_NFC_MENU` -> `LAYER_NFC_ADD` | `Tap tag...` and scan state | Adds new token hash | Two-scan confirm; write happens immediately during NFC op via `addTokenHash(hash)` |
| NFC: `NFC Del Token` | `LAYER_NFC_MENU` -> `LAYER_NFC_DEL` | `Tap tag...` and scan state | Deletes token hash | Two-scan confirm; write happens immediately during NFC op via `deleteTokenHash(hash)` |
| NFC: `Pack NFC Table` | `LAYER_NFC_MENU` | Shows “Packing!”/Saving overlays | De-dup and compact token table | Immediate operation: `packTokenHashTable()` |
| NFC: `Del ALL Tokens` | `LAYER_NFC_MENU` (modal prompt) | `RemoveAll? Sel=Y YES/NO` | Clears all token hashes | Long SELECT with YES runs `removeAllTokens()` |
| Top: `Re-init EEPROM` | `LAYER_TOP` -> `LAYER_REINIT` | Confirmation and hold countdown | Full EEPROM reset to defaults | Short SELECT enters prompt, UP/DOWN toggles YES/NO, then hold SELECT for 10s to run `reinitializeToDefaults(true)` |
