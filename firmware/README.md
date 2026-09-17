# Firmware

ESP-IDF project for the [Haptic Awareness Headband](../README.md), targeting the
Waveshare ESP32-S3-Zero (ESP32-S3FH4R2, 4MB flash, 2MB PSRAM).

See the top-level README for project overview and hardware details.

## Target

- `esp32s3`
- Flashed and monitored over the chip's native USB-Serial-JTAG — no separate UART
  bridge required, single USB-C connection
- Enter download mode: hold BOOT, tap RESET, release BOOT

## Building

Requires ESP-IDF v6.1, installed via the official Espressif ESP-IDF VS Code extension
(EIM through Homebrew).

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Or via the VS Code extension's Build / Flash / Monitor commands from the command palette.

## Structure

```
firmware/
├── CMakeLists.txt      Top-level build config
├── main/                Application entry point
│   ├── CMakeLists.txt
│   └── ...
└── sdkconfig             Generated on first build — board-specific config (not committed)
```

## Troubleshooting

- **Flash upload failure**: run `idf.py -p PORT monitor` and reboot the board to check
  for output; if nothing appears, confirm the port and that the board is in download
  mode (BOOT+RESET).
- **Baud rate too high**: lower it under Serial flasher config in `idf.py menuconfig`.
