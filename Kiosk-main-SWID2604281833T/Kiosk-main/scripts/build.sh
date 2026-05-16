#!/usr/bin/env bash
set -euo pipefail

FQBN=${FQBN:-arduino:avr:mega}
BUILD_DIR=${BUILD_DIR:-build}
BUILD_PATH=${BUILD_PATH:-build/arduino-build}
SKETCH=${SKETCH:-Kiosk.ino}
SOURCE=${SOURCE:-KioskBuildInfo.h}

# Build-time only: refresh SWID stamp right before compile.
scripts/refresh_build_stamp.sh

mkdir -p "$BUILD_DIR" "$BUILD_PATH"
arduino-cli compile \
  --fqbn "$FQBN" \
  --output-dir "$BUILD_DIR" \
  --build-path "$BUILD_PATH" \
  "$SKETCH"
