#include "battery_avr.h"

BatteryAVR::BatteryAVR() {}

long BatteryAVR::readVcc_mV() {
    // Read 1.1V internal reference against AVcc
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
    delay(2);  // Wait for Vref to settle
    ADCSRA |= _BV(ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    long result = ADCL;
    result |= ADCH << 8;
    if (result == 0) return 0;  // avoid division by zero
    return 1126400L / result;
}

float BatteryAVR::readVcc_V() {
    return (float)readVcc_mV() / 1000.0f;
}

int BatteryAVR::percentage(long vcc_mV, long min_mV, long max_mV) {
    return constrain(map(vcc_mV, min_mV, max_mV, 0, 100), 0, 100);
}
