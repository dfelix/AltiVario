# AltiVario — Known Issues & Future Work

## Warnings (non-blocking, noted from code review)

### ESP-IDF API Deprecations
- [ ] `audio_esp32.cpp` uses legacy I2S API (`driver/i2s.h`, `i2s_driver_install`, etc.) — deprecated in ESP-IDF 5.x, migrate to `driver/i2s_std.h`
- [ ] `battery_esp32.cpp` uses `ADC_11db` — deprecated in ESP-IDF 5.x, replace with `ADC_ATTEN_DB_12`

### Minor Code Quality
- [ ] `AltiVario_Vario.h` uses `static const float` in header — duplicated per translation unit, consider `constexpr` or `extern const`
- [ ] `VarioComputer` heap-allocates 400 bytes on AVR (~20% of RAM) with no `new` failure check
- [ ] `audio_esp32.cpp` `playDAC()` blocks the audio task for the full tone duration — consider DMA-based non-blocking playback
- [ ] `audio_esp32.cpp` `_ledcChannel` parameter is stored but unused (new Arduino-ESP32 3.x API auto-assigns channels)
- [ ] `ble_telemetry.cpp` `new ServerCallbacks` is never freed — leaks if `begin()` called twice
- [ ] `igc_logger.cpp` uses `extern GPSManager gps` — tight coupling to global variable name
- [ ] `display_manager.cpp` `begin()` ignores `sdaPin`/`sclPin`/`addr` parameters (U8g2 constructor doesn't use them)
- [ ] NMEA lib uses `long` for pressure — matches `int32_t` on ESP32/AVR but not portable to 64-bit platforms
- [ ] `sensor_bmp085.cpp` hardcodes SCL recovery pin to A5 (correct for ATmega328P only)

