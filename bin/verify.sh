#!/bin/sh

set -eu

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
ARDUINO_JSON="$ROOT_DIR/.vscode/arduino.json"

if ! command -v jq >/dev/null 2>&1; then
  echo "jq is not installed. Please install it like: sudo apt install jq" >&2
  exit 1
fi

if [ ! -f "$ARDUINO_JSON" ]; then
  echo "$ARDUINO_JSON not found. Please create the file with a valid 'board' entry." >&2
  exit 1
fi

FQBN=$(jq -er '.board | select(type == "string" and length > 0)' "$ARDUINO_JSON")
CONFIGURATION=$(jq -r '.configuration // empty' "$ARDUINO_JSON")
if [ -n "$CONFIGURATION" ]; then
  FQBN="$FQBN:$CONFIGURATION"
fi
INO_FILE="$(basename "$ROOT_DIR").ino"

echo "Compiling with FQBN: $FQBN"
echo "Using sketch file: $INO_FILE"

arduino-cli compile \
  --fqbn "$FQBN" \
  --build-path "$ROOT_DIR/Build" \
  --jobs 0 \
  "$ROOT_DIR/$INO_FILE"
