#include "button_handler.h"

ButtonHandler::ButtonHandler()
    : _pin(0)
    , _activeLow(true)
    , _pressed(false)
    , _pressTime(0)
    , _releaseTime(0)
    , _eventPending(false)
    , _wasPressed(false)
    , _lastDebounce(0)
{
}

void IRAM_ATTR ButtonHandler::isrHandler(void* arg) {
    ButtonHandler* self = static_cast<ButtonHandler*>(arg);
    bool state = digitalRead(self->_pin);
    if (self->_activeLow) state = !state;

    unsigned long now = millis();
    if ((now - self->_lastDebounce) < DEBOUNCE_MS) return;
    self->_lastDebounce = now;

    if (state && !self->_pressed) {
        // Press
        self->_pressed = true;
        self->_pressTime = now;
    } else if (!state && self->_pressed) {
        // Release
        self->_pressed = false;
        self->_releaseTime = now;
        self->_eventPending = true;
    }
}

void ButtonHandler::begin(uint8_t pin, bool activeLow) {
    _pin = pin;
    _activeLow = activeLow;

    if (_activeLow) {
        pinMode(_pin, INPUT_PULLUP);
    } else {
        pinMode(_pin, INPUT_PULLDOWN);
    }

    attachInterruptArg(digitalPinToInterrupt(_pin), isrHandler, this, CHANGE);
}

ButtonEvent ButtonHandler::getEvent() {
    if (!_eventPending) return BTN_NONE;
    _eventPending = false;

    unsigned long duration = _releaseTime - _pressTime;

    if (duration >= VERY_LONG_PRESS_MS) return BTN_VERY_LONG_PRESS;
    if (duration >= LONG_PRESS_MS) return BTN_LONG_PRESS;
    return BTN_SHORT_PRESS;
}

bool ButtonHandler::isPressed() const {
    return _pressed;
}
