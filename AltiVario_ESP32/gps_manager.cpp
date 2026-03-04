#include "gps_manager.h"

GPSManager::GPSManager()
    : _serial(nullptr)
    , _newFix(false)
    , _lastFixAge(0xFFFFFFFF)
{
}

void GPSManager::begin(uint8_t rxPin, uint8_t txPin, uint32_t baud) {
    _serial = &Serial1;
    _serial->begin(baud, SERIAL_8N1, rxPin, txPin);

    // Allow GPS time to start
    delay(100);

    // Configure for airborne mode
    configureUblox();
}

void GPSManager::update() {
    if (!_serial) return;

    while (_serial->available()) {
        char c = _serial->read();
        _gps.encode(c);
    }

    // Detect new fix by checking if location age changed
    if (_gps.location.isUpdated()) {
        _newFix = true;
    }
}

bool GPSManager::hasNewFix() {
    if (_newFix) {
        _newFix = false;
        return true;
    }
    return false;
}

float GPSManager::latitude() const {
    return _gps.location.isValid() ? (float)_gps.location.lat() : 0;
}

float GPSManager::longitude() const {
    return _gps.location.isValid() ? (float)_gps.location.lng() : 0;
}

float GPSManager::altitudeMSL() const {
    return _gps.altitude.isValid() ? (float)_gps.altitude.meters() : 0;
}

float GPSManager::speedKmh() const {
    return _gps.speed.isValid() ? (float)_gps.speed.kmph() : 0;
}

float GPSManager::heading() const {
    return _gps.course.isValid() ? (float)_gps.course.deg() : 0;
}

uint8_t GPSManager::satellites() const {
    return _gps.satellites.isValid() ? (uint8_t)_gps.satellites.value() : 0;
}

bool GPSManager::hasFix() const {
    return _gps.location.isValid() && _gps.location.age() < 3000;
}

uint8_t GPSManager::hour() const { return _gps.time.hour(); }
uint8_t GPSManager::minute() const { return _gps.time.minute(); }
uint8_t GPSManager::second() const { return _gps.time.second(); }
uint16_t GPSManager::year() const { return _gps.date.year(); }
uint8_t GPSManager::month() const { return _gps.date.month(); }
uint8_t GPSManager::day() const { return _gps.date.day(); }
bool GPSManager::timeValid() const { return _gps.time.isValid(); }

void GPSManager::configureUblox() {
    if (!_serial) return;

    // UBX-CFG-NAV5: Set to Airborne <2g> dynamic model (6)
    // This improves tracking at high altitude and with fast changes
    const uint8_t ubxNav5[] = {
        0xB5, 0x62,  // header
        0x06, 0x24,  // CFG-NAV5
        0x24, 0x00,  // length 36
        0xFF, 0xFF,  // mask: apply all
        0x06,        // dynModel: Airborne <2g>
        0x03,        // fixMode: auto 2D/3D
        0x00, 0x00, 0x00, 0x00,  // fixedAlt
        0x10, 0x27, 0x00, 0x00,  // fixedAltVar
        0x05,        // minElev: 5 degrees
        0x00,        // drLimit
        0xFA, 0x00,  // pDop: 25.0
        0xFA, 0x00,  // tDop: 25.0
        0x64, 0x00,  // pAcc: 100m
        0x2C, 0x01,  // tAcc: 300m
        0x00,        // staticHoldThresh
        0x00,        // dgnssTimeout
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        // checksum calculated below
    };

    // Calculate UBX checksum
    uint8_t ckA = 0, ckB = 0;
    for (size_t i = 2; i < sizeof(ubxNav5); i++) {
        ckA += ubxNav5[i];
        ckB += ckA;
    }

    _serial->write(ubxNav5, sizeof(ubxNav5));
    _serial->write(ckA);
    _serial->write(ckB);
    _serial->flush();
}

void GPSManager::enablePowerSave(bool enable) {
    if (!_serial) return;

    // UBX-CFG-RXM: Power Save Mode
    uint8_t ubxRxm[] = {
        0xB5, 0x62,
        0x06, 0x11,  // CFG-RXM
        0x02, 0x00,
        0x08,        // reserved
        0x00,        // lpMode: 0=continuous, 1=power save
    };
    ubxRxm[7] = enable ? 0x01 : 0x00;

    uint8_t ckA = 0, ckB = 0;
    for (size_t i = 2; i < sizeof(ubxRxm); i++) {
        ckA += ubxRxm[i];
        ckB += ckA;
    }

    _serial->write(ubxRxm, sizeof(ubxRxm));
    _serial->write(ckA);
    _serial->write(ckB);
    _serial->flush();
}
