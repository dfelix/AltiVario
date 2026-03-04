AltiVario
=========

Open-source variometer for paragliding and hang gliding. Two hardware variants sharing common libraries.

## Versions

### AltiVario Legacy (ATmega328P + BMP085)
Refactored version of the original vario.ino with bug fixes:
- Ring buffer averaging (fixes out-of-bounds array access)
- `unsigned long` timestamps (fixes float precision loss at ~4.6 hours)
- Division-by-zero guard in linear regression
- Missing `break` statements in switch/case (audio)
- Zero-heap NMEA formatting via `snprintf()` (eliminates String class fragmentation)
- Watchdog timer and I2C bus recovery

### AltiVario ESP32 (ESP32-S3 + MS5611)
Full-featured variometer with FreeRTOS multi-task architecture:
- **MS5611** barometric sensor at 50Hz with CRC4 validation
- **BLE telemetry** (NimBLE UART service) — compatible with XCTrack/XCSoar
- **OLED display** (SSD1306 128x64) — 3 pages: Flight, Stats, Settings
- **GPS** (u-blox NEO-6M) with airborne mode, glide ratio computation
- **IGC flight logging** to MicroSD card with 512-byte write buffer
- **WiFi configuration portal** — captive portal for settings and log download
- **Dual audio** — piezo (LEDC PWM) and DAC speaker (MAX98357A via I2S)
- **Auto flight detection** — launch/landing state machine
- **NVS-backed configuration** — persists across reboots

## Repository Structure

```
AltiVario/
├── libraries/                    # Shared Arduino libraries
│   ├── AltiVario_Vario/          # VarioComputer, AveragingFilter, pressureToAltitude()
│   ├── AltiVario_NMEA/           # LK8EX1 + Flymaster F1 formatting (zero-heap)
│   └── AltiVario_Audio/          # AudioPatternEngine (tone params from vario rate)
├── AltiVario_Legacy/             # ATmega328P sketch
└── AltiVario_ESP32/              # ESP32-S3 sketch with FreeRTOS tasks
    └── data/                     # WiFi portal web assets
```

## ESP32-S3 Pin Assignments

| Function | Pin | Notes |
|----------|-----|-------|
| I2C SDA | GPIO 8 | MS5611 + SSD1306 |
| I2C SCL | GPIO 9 | |
| GPS RX | GPIO 17 | NEO-6M TX |
| GPS TX | GPIO 18 | NEO-6M RX |
| SD CS/MOSI/SCK/MISO | GPIO 10/11/12/13 | SPI |
| Piezo | GPIO 38 | LEDC PWM |
| I2S DAC | GPIO 4/5/6 | BCLK/LRCLK/DOUT |
| Button | GPIO 0 | Boot button |
| Battery ADC | GPIO 1 | Voltage divider |

## Dependencies

**Legacy:** Wire, BMP085, Tone, AltiVario_* shared libs

**ESP32-S3:** Wire, MS5611, NimBLE-Arduino, U8g2, TinyGPS++, SD, ESPAsyncWebServer, AsyncTCP, Preferences, AltiVario_* shared libs

## Setup

1. Symlink shared libraries to your Arduino libraries folder:
   ```bash
   ln -s $(pwd)/libraries/AltiVario_Vario ~/Arduino/libraries/
   ln -s $(pwd)/libraries/AltiVario_NMEA ~/Arduino/libraries/
   ln -s $(pwd)/libraries/AltiVario_Audio ~/Arduino/libraries/
   ```

2. Install board support:
   - Legacy: Arduino AVR Boards
   - ESP32: ESP32 Arduino Core (Espressif)

3. Install library dependencies via Arduino Library Manager.

4. Open the appropriate `.ino` sketch and upload.

## Power Budget (ESP32-S3 flight mode)

~200-250 mA total. 2000 mAh LiPo = ~8-10 hours, 3000 mAh = ~12-15 hours.
