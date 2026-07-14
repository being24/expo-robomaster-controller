#!/bin/sh

set -eu

ESP32_BOARD_URL="https://espressif.github.io/arduino-esp32/package_esp32_index.json"
ESP32_CORE="esp32:esp32@3.2.1"

arduino-cli config set board_manager.additional_urls "$ESP32_BOARD_URL"
arduino-cli core update-index
arduino-cli core install "$ESP32_CORE"

arduino-cli lib install "M5Unified@0.2.18"
arduino-cli lib install "ArduinoJson@7.4.2"
arduino-cli lib install "ESP32-TWAI-CAN@1.0.1"
