#ifndef GPS_MANAGER_H
#define GPS_MANAGER_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

// GPS manager wrapping TinyGPS++ on HardwareSerial (UART1)
class GPSManager {
public:
    GPSManager();

    // Initialize GPS UART
    void begin(uint8_t rxPin, uint8_t txPin, uint32_t baud = 9600);

    // Feed UART data to parser — call frequently
    void update();

    // True once after each new valid fix
    bool hasNewFix();

    // Position
    float latitude() const;
    float longitude() const;
    float altitudeMSL() const;

    // Motion
    float speedKmh() const;
    float heading() const;

    // Quality
    uint8_t satellites() const;
    bool hasFix() const;

    // Time (UTC)
    uint8_t hour() const;
    uint8_t minute() const;
    uint8_t second() const;
    uint16_t year() const;
    uint8_t month() const;
    uint8_t day() const;
    bool timeValid() const;

    // Configure u-blox NEO-6M for airborne mode
    void configureUblox();

    // Power save mode
    void enablePowerSave(bool enable);

private:
    TinyGPSPlus _gps;
    HardwareSerial* _serial;
    bool _newFix;
    uint32_t _lastFixAge;
};

#endif // GPS_MANAGER_H
