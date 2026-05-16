# Main UI State Machine (Kiosk User Perspective)

This describes the primary runtime UI flow implemented in `KioskBehavior::Ui::uiUpdate()`.

![UI Runtime Operation](diagrams/kiosk_ui_runtime_state_machine.png)

## Core user journey

1. `UI_BOOT`
- Shows sales counters.
- Moves to `UI_WELCOME` on front-button press or after 60s.

2. `UI_WELCOME`
- Main attract screen.
- Normal flow: press/release front button -> `UI_PAY_PROMPT`.
- Override flow: press/release with override still active -> `UI_OVERRIDE`.
- Dark mode entry flow: press in override, then release after switching to normal -> `UI_DARK_MODE_IN`.
- If dispense is not OK (and override is not active), transitions to `UI_OUT_OF_SERVICE`.

3. `UI_PAY_PROMPT` (normal mode)
- Waits up to 15s for payment.
- On success -> one of `UI_PAY_COIN_OK`, `UI_PAY_APP_OK`, `UI_PAY_TOKEN_OK`.
- On invalid token -> `UI_PAY_TOKEN_BAD`.
- On failed payment -> `UI_PAY_FAILED`.
- Timeout -> `UI_WELCOME`.
- If override becomes active, bypasses to `UI_OVERRIDE`.

4. Payment result states
- `UI_PAY_COIN_OK`, `UI_PAY_APP_OK`, `UI_PAY_TOKEN_OK`: after 2s -> `UI_DISP_RDY`.
- `UI_PAY_TOKEN_BAD`: front press or 3s -> `UI_WELCOME`.
- `UI_PAY_FAILED`: front press -> `UI_PAY_PROMPT`, or 15s -> `UI_WELCOME`.

5. `UI_OVERRIDE`
- Short acknowledgement screen for override dispensing.
- Increments override counter once.
- After 2s -> `UI_DISP_RDY`.

6. Dispense states
- `UI_DISP_RDY`: front/rear dispense press -> `UI_DISPENSING`, or 60s timeout -> `UI_TIMEOUT`.
- `UI_DISPENSING`: when dispense completes -> `UI_DISP_DONE`.
- `UI_DISP_DONE`: front press or 2s -> `UI_WELCOME`.
- `UI_TIMEOUT`: after 3s -> `UI_WELCOME`.

7. `UI_OUT_OF_SERVICE`
- Shown when dispensing is unavailable.
- Returns to `UI_WELCOME` when dispense recovers, or immediately if override is enabled.

## Dark mode branch

1. `UI_DARK_MODE_IN`
- 60s entry countdown, then 2s transition.
- Front press during countdown skips directly to the final phase.
- Once fully dark, front press -> `UI_DARK_MODE_OUT`.

2. `UI_DARK_MODE_OUT`
- Hold front button for 8s -> reset event.
- Releasing before 8s cancels exit and returns to fully-dark `UI_DARK_MODE_IN`.

## Error behavior

- From most non-dark states, active error forces transition to `UI_ERROR`.
- `UI_ERROR` self-refreshes in a 1s flashing loop until reset/recovery logic outside this state changes conditions.
