#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <U8g2lib.h>
#include "flight_data.h"

// Display pages
enum DisplayPage {
    PAGE_FLIGHT = 0,   // Main flight: altitude, vario, speed, glide
    PAGE_STATS  = 1,   // Flight stats: duration, max alt, max climb/sink
    PAGE_SETTINGS = 2  // Settings: battery, GPS sats, BLE, audio mode
};

// OLED display manager using U8g2 page buffer mode
class DisplayManager {
public:
    DisplayManager();

    void begin(uint8_t sdaPin, uint8_t sclPin, uint8_t addr);

    // Update display with current page and flight data
    void update(uint8_t page, const FlightData& fd);

    // Show splash screen
    void showSplash(const char* name, const char* version);

    // Show temporary message
    void showMessage(const char* msg);

    // Brightness control
    void setBrightness(uint8_t brightness);

    // Turn display on/off
    void setEnabled(bool on);

private:
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C* _u8g2;
    unsigned long _messageUntil;
    char _message[32];

    void drawFlightPage(const FlightData& fd);
    void drawStatsPage(const FlightData& fd);
    void drawSettingsPage(const FlightData& fd);
    void drawStatusBar(const FlightData& fd);

    // Stats tracking
    float _maxAlt;
    float _maxClimb;
    float _maxSink;
};

#endif // DISPLAY_MANAGER_H
