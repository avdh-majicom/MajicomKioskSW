#!/usr/bin/env bash
set -euo pipefail

PORT=${PORT:-COM3}
BAUD=${BAUD:-250000}

if command -v picocom >/dev/null 2>&1; then
  picocom -b "$BAUD" "$PORT"
elif command -v screen >/dev/null 2>&1; then
  screen "$PORT" "$BAUD"
else
  echo "No serial monitor found. Install picocom or screen." >&2
  exit 1
fi
