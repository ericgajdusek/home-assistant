# ESP32-P4 Smart Room Controller

A local-first smart home system built on the ESP32-P4 featuring:

- Touchscreen UI (LVGL)
- Voice activation (AFE + Wake Word)
- Smart blinds (stepper motor via ESP-NOW)
- Smart lighting (ESP-NOW + MOSFET LED control)
- Local MQTT + Server
- Cloud sync (Firebase)

## Hardware
- ESP32-P4 Touch Display
- ESP32-S3 nodes
- NEMA-17 stepper motor
- MOSFET LED drivers
- Light, motion, and temperature sensors

## Firmware
Built using ESP-IDF and FreeRTOS.

## Status
Work in progress – currently implementing voice wake word + LVGL UI + ESP-NOW control mesh.
