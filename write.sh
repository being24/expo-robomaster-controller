#!/bin/sh

arduino-cli compile --fqbn esp32:esp32:m5stack_stickc_plus2 --build-path ./Build --upload --port /dev/ttyACM0 --verbose expo-robomaster-controller.ino