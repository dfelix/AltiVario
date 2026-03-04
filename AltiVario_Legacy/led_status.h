#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <Arduino.h>

// Bicolor LED status indicator
class LEDStatus {
public:
    LEDStatus();

    void begin(uint8_t redPin, uint8_t greenPin);

    // Set LED based on battery voltage
    void updateBattery(float voltage, float goodV, float warnV);

    // Direct control
    void setGreen();
    void setRed();
    void setOrange();  // both on
    void off();

private:
    uint8_t _redPin;
    uint8_t _greenPin;
};

#endif // LED_STATUS_H
