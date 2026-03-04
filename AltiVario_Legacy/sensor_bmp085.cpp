#include "sensor_bmp085.h"
#include "config.h"

SensorBMP085::SensorBMP085()
    : _ok(false)
    , _errorCount(0)
{
}

bool SensorBMP085::begin(float ref_pressure_pa) {
    Wire.begin();

    // Probe I2C address 0x77 (BMP085 default)
    Wire.beginTransmission(0x77);
    if (Wire.endTransmission() != 0) {
        _ok = false;
        return false;
    }

    _bmp.init(BMP085_MODE, (long)ref_pressure_pa, false);
    _ok = true;
    _errorCount = 0;
    return true;
}

bool SensorBMP085::readPressure(long& pressure_pa) {
    if (!_ok) return false;

    long p = 0;
    _bmp.calcTruePressure(&p);

    // Basic sanity check: pressure should be 30000–120000 Pa
    if (p < 30000 || p > 120000) {
        _errorCount++;
        if (_errorCount >= MAX_ERRORS) {
            _ok = false;
            recoverI2CBus();
        }
        return false;
    }

    _errorCount = 0;
    pressure_pa = p;
    return true;
}

bool SensorBMP085::readTemperature(long& temperature_tenths_c) {
    if (!_ok) return false;
    _bmp.getTemperature(&temperature_tenths_c);
    return true;
}

void SensorBMP085::recoverI2CBus() {
    // Toggle SCL to release stuck SDA line
    Wire.end();
    pinMode(A5, OUTPUT);  // SCL on ATmega328P
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(A5, HIGH);
        delayMicroseconds(5);
        digitalWrite(A5, LOW);
        delayMicroseconds(5);
    }
    digitalWrite(A5, HIGH);
    Wire.begin();
}
