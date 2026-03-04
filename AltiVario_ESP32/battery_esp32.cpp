#include "battery_esp32.h"

BatteryESP32::BatteryESP32()
    : _pin(0)
    , _dividerRatio(2.0f)
    , _samples(16)
    , _lastV(0.0f)
{
}

void BatteryESP32::begin(uint8_t adcPin, float dividerRatio, uint8_t samples) {
    _pin = adcPin;
    _dividerRatio = dividerRatio;
    _samples = samples;
    analogSetAttenuation(ADC_11db);
    pinMode(_pin, INPUT);
}

float BatteryESP32::readVoltage() {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < _samples; i++) {
        sum += analogReadMilliVolts(_pin);
    }
    float adcMv = (float)sum / _samples;
    _lastV = (adcMv / 1000.0f) * _dividerRatio;
    return _lastV;
}

uint8_t BatteryESP32::percentage(float fullV, float emptyV) {
    if (_lastV >= fullV) return 100;
    if (_lastV <= emptyV) return 0;
    return (uint8_t)((_lastV - emptyV) / (fullV - emptyV) * 100.0f);
}
