#ifndef SENSOR_MS5611_H
#define SENSOR_MS5611_H

#include <Arduino.h>
#include <Wire.h>

// Full MS5611 driver with CRC4 validation, non-blocking reads, 50Hz
class SensorMS5611 {
public:
    SensorMS5611(uint8_t address = 0x77);

    // Initialize sensor, read PROM, verify CRC. Returns true on success.
    bool begin(TwoWire& wire = Wire);

    // Start an asynchronous conversion. Call readResult() after conversion time.
    void startPressureConversion();
    void startTemperatureConversion();

    // Read result from ADC. Returns true on success.
    bool readADC(uint32_t& result);

    // Perform full compensated read (blocking ~20ms)
    // Returns compensated pressure in Pa and temperature in 0.01°C
    bool read(int32_t& pressure_pa, int32_t& temperature_hundredths);

    // Non-blocking state machine: call repeatedly at >100Hz
    // Returns true when a new pressure/temp reading is available
    bool update();

    // Latest readings (valid after update() returns true)
    int32_t getPressure() const { return _pressure; }
    int32_t getTemperature() const { return _temperature; }
    bool isOk() const { return _ok; }

    // I2C bus recovery
    static void recoverI2CBus(uint8_t sdaPin, uint8_t sclPin);

private:
    TwoWire* _wire;
    uint8_t _addr;
    uint16_t _prom[8];
    bool _ok;

    // Non-blocking state machine
    enum ConvState { CONV_IDLE, CONV_TEMP_WAIT, CONV_PRESS_WAIT };
    ConvState _convState;
    unsigned long _convStart;
    uint32_t _rawTemp;
    uint32_t _rawPress;
    int32_t _pressure;
    int32_t _temperature;

    bool readPROM();
    bool verifyCRC4();
    void sendCommand(uint8_t cmd);
    void compensate(uint32_t D1, uint32_t D2, int32_t& pressure, int32_t& temperature);

    static const uint8_t CMD_RESET      = 0x1E;
    static const uint8_t CMD_D1_4096    = 0x48;  // pressure, OSR=4096
    static const uint8_t CMD_D2_4096    = 0x58;  // temperature, OSR=4096
    static const uint8_t CMD_ADC_READ   = 0x00;
    static const uint8_t CMD_PROM_BASE  = 0xA0;
    static const unsigned long CONV_TIME_MS = 10;  // 9.04ms for OSR=4096
};

#endif // SENSOR_MS5611_H
