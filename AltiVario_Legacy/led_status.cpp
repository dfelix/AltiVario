#include "led_status.h"

LEDStatus::LEDStatus()
    : _redPin(0)
    , _greenPin(0)
{
}

void LEDStatus::begin(uint8_t redPin, uint8_t greenPin) {
    _redPin = redPin;
    _greenPin = greenPin;
    pinMode(_redPin, OUTPUT);
    pinMode(_greenPin, OUTPUT);
    setOrange();  // initial state while booting
}

void LEDStatus::updateBattery(float voltage, float goodV, float warnV) {
    if (voltage > goodV) {
        setGreen();
    } else if (voltage > warnV) {
        setOrange();
    } else {
        setRed();
    }
}

void LEDStatus::setGreen() {
    digitalWrite(_greenPin, HIGH);
    digitalWrite(_redPin, LOW);
}

void LEDStatus::setRed() {
    digitalWrite(_greenPin, LOW);
    digitalWrite(_redPin, HIGH);
}

void LEDStatus::setOrange() {
    digitalWrite(_greenPin, HIGH);
    digitalWrite(_redPin, HIGH);
}

void LEDStatus::off() {
    digitalWrite(_greenPin, LOW);
    digitalWrite(_redPin, LOW);
}
