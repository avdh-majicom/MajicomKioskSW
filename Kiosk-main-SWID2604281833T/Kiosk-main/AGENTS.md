AGENTS.md

Purpose
- This file provides guidance for automated agents working in this repository.
- Keep instructions concise and actionable.

Required Checks (read before doing any work)
- Confirm current branch and git status.
- Identify the target board and toolchain before compiling or uploading.
- Before building, refresh the `ver:YYMMDDHHMMSS` timestamp in any module being built.
- When running via Codex, build and upload must be executed outside of the sandbox.
- Avoid modifying credentials or secrets; do not store tokens in repo files.

Working Guidelines
- Prefer small, focused changes.
- Run the minimal commands needed to validate a change.
- Summarize changes and list any follow-up steps.
- Auto-request approval when needed for commands that require escalated permissions.

Project Notes
- Arduino sketch: Kiosk.ino
- Target board: Arduino MEGA 2560 (arduino:avr:mega)
- Build artifacts go in the build/ subdirectory when compiling/exporting.
- Upload to /dev/ttyACM0 outside of the sandbox when flashing the target board.
- Primary modules: KioskHydraulics.*, KioskIO.*
