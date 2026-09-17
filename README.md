# Haptic Awareness Headband

A wearable headband that provides spatial/environmental awareness through haptic
(vibration) feedback, combining IMU/magnetometer heading data with directional
audio input from a mic array.

> TODO: replace this paragraph with the actual functional description — what
> the device senses, how it decides what to alert on, and how haptic output is
> mapped to that (e.g. direction, urgency).

## Hardware

- **MCU**: Waveshare ESP32-S3-Zero (ESP32-S3FH4R2, 4MB flash, 2MB PSRAM)
- **Connectivity**: single USB-C, native USB-Serial-JTAG (no separate UART bridge chip)
- **Sensors**: IMU/magnetometer (I2C), mic array (I2S)
- Enters download/flash mode via BOOT + RESET buttons

## Toolchain

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) v6.1 (not Arduino), via the
  official Espressif VS Code extension
- Installed via ESP-IDF Installation Manager (EIM) through Homebrew
- Target: `esp32s3`

## Repo structure

```
haptic-awareness-device/
└── firmware/           ESP-IDF project (see firmware/README.md for build details)
    ├── main/            Application entry point
    ├── CMakeLists.txt
    └── ...
```

## Building & flashing

1. Open `firmware/` in VS Code with the ESP-IDF extension installed.
2. Set target: `esp32s3` (Cmd+Shift+P → "ESP-IDF: Set Espressif Device Target").
3. Select the serial port (Cmd+Shift+P → "ESP-IDF: Select Port to Use").
4. Build: "ESP-IDF: Build your Project".
5. Flash & monitor: "ESP-IDF: Flash your Project", then "ESP-IDF: Monitor your Device".

## Status

- [x] Repo + ESP-IDF project scaffolded
- [ ] I2C driver for IMU/magnetometer
- [ ] I2S setup for mic array
- [ ] Haptic actuator output
- [ ] Sensor fusion / alert logic
