#include "sensor_ms5611.h"

SensorMS5611::SensorMS5611(uint8_t address)
    : _wire(nullptr)
    , _addr(address)
    , _ok(false)
    , _convState(CONV_IDLE)
    , _convStart(0)
    , _rawTemp(0)
    , _rawPress(0)
    , _pressure(101325)
    , _temperature(2000)
{
    memset(_prom, 0, sizeof(_prom));
}

bool SensorMS5611::begin(TwoWire& wire) {
    _wire = &wire;

    // Probe device
    _wire->beginTransmission(_addr);
    if (_wire->endTransmission() != 0) {
        _ok = false;
        return false;
    }

    // Reset
    sendCommand(CMD_RESET);
    delay(5);

    // Read calibration PROM
    if (!readPROM()) {
        _ok = false;
        return false;
    }

    // Verify CRC
    if (!verifyCRC4()) {
        _ok = false;
        return false;
    }

    _ok = true;
    _convState = CONV_IDLE;
    return true;
}

void SensorMS5611::sendCommand(uint8_t cmd) {
    _wire->beginTransmission(_addr);
    _wire->write(cmd);
    _wire->endTransmission();
}

bool SensorMS5611::readPROM() {
    for (uint8_t i = 0; i < 8; i++) {
        sendCommand(CMD_PROM_BASE + (i * 2));
        if (_wire->requestFrom(_addr, (uint8_t)2) != 2) return false;
        _prom[i] = ((uint16_t)_wire->read() << 8) | _wire->read();
    }
    return true;
}

bool SensorMS5611::verifyCRC4() {
    uint16_t crc_read = _prom[7] & 0x000F;
    uint16_t prom_copy[8];
    memcpy(prom_copy, _prom, sizeof(prom_copy));

    prom_copy[7] &= 0xFF00;
    uint16_t n_rem = 0;

    for (uint8_t cnt = 0; cnt < 16; cnt++) {
        if (cnt & 1) {
            n_rem ^= prom_copy[cnt >> 1] & 0x00FF;
        } else {
            n_rem ^= prom_copy[cnt >> 1] >> 8;
        }
        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem <<= 1;
            }
        }
    }
    n_rem = (n_rem >> 12) & 0x000F;
    return (n_rem == crc_read);
}

bool SensorMS5611::readADC(uint32_t& result) {
    sendCommand(CMD_ADC_READ);
    if (_wire->requestFrom(_addr, (uint8_t)3) != 3) return false;
    result = ((uint32_t)_wire->read() << 16) |
             ((uint32_t)_wire->read() << 8) |
             _wire->read();
    return true;
}

void SensorMS5611::compensate(uint32_t D1, uint32_t D2,
                               int32_t& pressure, int32_t& temperature) {
    int64_t dT = (int64_t)D2 - ((int64_t)_prom[5] << 8);
    temperature = 2000 + (int32_t)((dT * (int64_t)_prom[6]) >> 23);

    int64_t OFF  = ((int64_t)_prom[2] << 16) + (((int64_t)_prom[4] * dT) >> 7);
    int64_t SENS = ((int64_t)_prom[1] << 15) + (((int64_t)_prom[3] * dT) >> 8);

    // Second order compensation
    if (temperature < 2000) {
        int64_t T2 = (dT * dT) >> 31;
        int64_t t_diff = temperature - 2000;
        int64_t OFF2  = 5 * t_diff * t_diff / 2;
        int64_t SENS2 = 5 * t_diff * t_diff / 4;

        if (temperature < -1500) {
            int64_t t_diff2 = temperature + 1500;
            OFF2  += 7 * t_diff2 * t_diff2;
            SENS2 += 11 * t_diff2 * t_diff2 / 2;
        }

        temperature -= (int32_t)T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    pressure = (int32_t)(((int64_t)D1 * SENS / 2097152 - OFF) / 32768);
}

bool SensorMS5611::read(int32_t& pressure_pa, int32_t& temperature_hundredths) {
    if (!_ok) return false;

    // Temperature conversion
    sendCommand(CMD_D2_4096);
    delay(CONV_TIME_MS);
    uint32_t D2;
    if (!readADC(D2)) return false;

    // Pressure conversion
    sendCommand(CMD_D1_4096);
    delay(CONV_TIME_MS);
    uint32_t D1;
    if (!readADC(D1)) return false;

    compensate(D1, D2, pressure_pa, temperature_hundredths);
    return true;
}

bool SensorMS5611::update() {
    if (!_ok) return false;
    unsigned long now = millis();

    switch (_convState) {
        case CONV_IDLE:
            sendCommand(CMD_D2_4096);
            _convStart = now;
            _convState = CONV_TEMP_WAIT;
            return false;

        case CONV_TEMP_WAIT:
            if ((now - _convStart) < CONV_TIME_MS) return false;
            if (!readADC(_rawTemp)) {
                _convState = CONV_IDLE;
                return false;
            }
            sendCommand(CMD_D1_4096);
            _convStart = now;
            _convState = CONV_PRESS_WAIT;
            return false;

        case CONV_PRESS_WAIT:
            if ((now - _convStart) < CONV_TIME_MS) return false;
            if (!readADC(_rawPress)) {
                _convState = CONV_IDLE;
                return false;
            }
            compensate(_rawPress, _rawTemp, _pressure, _temperature);
            _convState = CONV_IDLE;
            return true;  // new reading available
    }
    return false;
}

void SensorMS5611::recoverI2CBus(uint8_t sdaPin, uint8_t sclPin) {
    pinMode(sclPin, OUTPUT);
    pinMode(sdaPin, INPUT_PULLUP);
    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(sclPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(sclPin, LOW);
        delayMicroseconds(5);
    }
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
}
