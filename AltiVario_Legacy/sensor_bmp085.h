#ifndef SENSOR_BMP085_H
#define SENSOR_BMP085_H

#include <Arduino.h>
#include <Wire.h>
#include <BMP085.h>

// BMP085 sensor wrapper with error handling and I2C bus recovery
class SensorBMP085 {
public:
    SensorBMP085();

    // Initialize sensor. Returns true on success.
    bool begin(float ref_pressure_pa = 101325.0f);

    // Read pressure into output. Returns true on success.
    bool readPressure(long& pressure_pa);

    // Read temperature into output. Returns true on success.
    bool readTemperature(long& temperature_tenths_c);

    // Attempt I2C bus recovery (clock out stuck slave)
    static void recoverI2CBus();

    bool isOk() const { return _ok; }

private:
    BMP085 _bmp;
    bool _ok;
    uint8_t _errorCount;
    static const uint8_t MAX_ERRORS = 5;
};

#endif // SENSOR_BMP085_H
