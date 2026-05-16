#!/usr/bin/env bash
set -euo pipefail

# Timestamp is updated by build.sh (build phase) only.
scripts/build.sh
scripts/upload.sh
