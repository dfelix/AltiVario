#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <Arduino.h>

// Button event types
enum ButtonEvent {
    BTN_NONE,
    BTN_SHORT_PRESS,
    BTN_LONG_PRESS,     // >1 second
    BTN_VERY_LONG_PRESS // >3 seconds
};

// GPIO ISR-based button handler with debounce
class ButtonHandler {
public:
    ButtonHandler();

    void begin(uint8_t pin, bool activeLow = true);

    // Call from main loop or task to get pending event
    ButtonEvent getEvent();

    // Check if button is currently held
    bool isPressed() const;

private:
    uint8_t _pin;
    bool _activeLow;
    volatile bool _pressed;
    volatile unsigned long _pressTime;
    volatile unsigned long _releaseTime;
    volatile bool _eventPending;
    bool _wasPressed;
    unsigned long _lastDebounce;

    static const unsigned long DEBOUNCE_MS = 50;
    static const unsigned long LONG_PRESS_MS = 1000;
    static const unsigned long VERY_LONG_PRESS_MS = 3000;

    static void IRAM_ATTR isrHandler(void* arg);
};

#endif // BUTTON_HANDLER_H
