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
  echo "$ARDUINO_JSON not found. Please create the file with valid 'board' and 'port' entries." >&2
  exit 1
fi

FQBN=$(jq -er '.board | select(type == "string" and length > 0)' "$ARDUINO_JSON")
CONFIGURATION=$(jq -r '.configuration // empty' "$ARDUINO_JSON")
if [ -n "$CONFIGURATION" ]; then
  FQBN="$FQBN:$CONFIGURATION"
fi
PORT=$(jq -er '.port | select(type == "string" and length > 0)' "$ARDUINO_JSON")
INO_FILE="$(basename "$ROOT_DIR").ino"

echo "Using port: $PORT"
echo "Uploading with FQBN: $FQBN, sketch: $INO_FILE"

arduino-cli compile \
  --fqbn "$FQBN" \
  --build-path "$ROOT_DIR/Build" \
  --upload \
  --port "$PORT" \
  --verbose \
  "$ROOT_DIR/$INO_FILE"
