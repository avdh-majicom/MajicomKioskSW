#!/usr/bin/env bash
set -euo pipefail

SOURCE=${SOURCE:-KioskBuildInfo.h}
STAMP="$(date +%y%m%d%H%M)"

if grep -q 'KIOSK_SWID' "$SOURCE"; then
  sed -i -E "s/(KIOSK_SWID[[:space:]]+\")SWID:[0-9]{10}([A-Za-z])(\")/\\1SWID:${STAMP}\\2\\3/" "$SOURCE"
else
  echo "Warning: KIOSK_SWID not found in ${SOURCE}" >&2
fi
