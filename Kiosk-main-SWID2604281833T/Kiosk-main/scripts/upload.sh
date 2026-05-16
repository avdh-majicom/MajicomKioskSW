#!/usr/bin/env bash
# Note: uploads from sandboxed environments require unsandboxed device access.
# Intentionally does not modify SWID/build timestamps; upload should be read-only.
set -euo pipefail

FQBN=${FQBN:-arduino:avr:mega}
INPUT_DIR=${INPUT_DIR:-build}
SKETCH=${SKETCH:-Kiosk.ino}

if [ -z "${PORT:-}" ]; then
  echo "Error: PORT is not set. Example: PORT=/dev/ttyACM0 scripts/upload.sh" >&2
  echo "If you're running in a sandbox, re-run with unsandboxed device access." >&2
  exit 1
fi

echo "Using PORT=${PORT}"
arduino-cli upload -p "$PORT" --fqbn "$FQBN" --input-dir "$INPUT_DIR" "$SKETCH"
