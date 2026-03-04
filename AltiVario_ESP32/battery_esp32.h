#ifndef BATTERY_ESP32_H
#define BATTERY_ESP32_H

#include <Arduino.h>

// ESP32 battery monitor with multisampling and calibration
class BatteryESP32 {
public:
    BatteryESP32();

    void begin(uint8_t adcPin, float dividerRatio = 2.0f, uint8_t samples = 16);

    // Read battery voltage (returns volts)
    float readVoltage();

    // Get battery percentage (linear approximation)
    uint8_t percentage(float fullV = 4.2f, float emptyV = 3.0f);

    float lastVoltage() const { return _lastV; }

private:
    uint8_t _pin;
    float _dividerRatio;
    uint8_t _samples;
    float _lastV;
};

#endif // BATTERY_ESP32_H
