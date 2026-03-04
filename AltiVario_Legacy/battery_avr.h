#ifndef BATTERY_AVR_H
#define BATTERY_AVR_H

#include <Arduino.h>

// AVR battery voltage reader using internal bandgap reference
class BatteryAVR {
public:
    BatteryAVR();

    // Read Vcc in millivolts using bandgap reference
    long readVcc_mV();

    // Read Vcc in volts
    float readVcc_V();

    // Map voltage to percentage (linear approximation)
    int percentage(long vcc_mV, long min_mV = 2500, long max_mV = 4300);
};

#endif // BATTERY_AVR_H
