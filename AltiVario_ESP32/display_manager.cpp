#include "display_manager.h"
#include "config.h"

DisplayManager::DisplayManager()
    : _u8g2(nullptr)
    , _messageUntil(0)
    , _maxAlt(0)
    , _maxClimb(0)
    , _maxSink(0)
{
    memset(_message, 0, sizeof(_message));
}

void DisplayManager::begin(uint8_t sdaPin, uint8_t sclPin, uint8_t addr) {
    _u8g2 = new U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
    _u8g2->begin();
    _u8g2->setFont(u8g2_font_6x10_tr);
}

void DisplayManager::update(uint8_t page, const FlightData& fd) {
    if (!_u8g2) return;

    // Show temporary message if active
    unsigned long now = millis();
    if (_messageUntil > 0 && now < _messageUntil) {
        _u8g2->clearBuffer();
        _u8g2->setFont(u8g2_font_9x15B_tr);
        int w = _u8g2->getStrWidth(_message);
        _u8g2->drawStr((128 - w) / 2, 38, _message);
        _u8g2->sendBuffer();
        return;
    }
    _messageUntil = 0;

    // Update stats
    if (fd.altitude > _maxAlt) _maxAlt = fd.altitude;
    if (fd.vario > _maxClimb) _maxClimb = fd.vario;
    if (fd.vario < _maxSink) _maxSink = fd.vario;

    _u8g2->clearBuffer();

    switch (page) {
        case PAGE_FLIGHT:   drawFlightPage(fd); break;
        case PAGE_STATS:    drawStatsPage(fd); break;
        case PAGE_SETTINGS: drawSettingsPage(fd); break;
    }

    drawStatusBar(fd);
    _u8g2->sendBuffer();
}

void DisplayManager::drawFlightPage(const FlightData& fd) {
    char buf[20];

    // Altitude (large)
    _u8g2->setFont(u8g2_font_logisoso22_tn);
    snprintf(buf, sizeof(buf), "%d", (int)fd.altitude);
    _u8g2->drawStr(0, 28, buf);
    _u8g2->setFont(u8g2_font_6x10_tr);
    _u8g2->drawStr(100, 28, "m");

    // Vario
    _u8g2->setFont(u8g2_font_logisoso16_tn);
    dtostrf(fd.vario, 4, 1, buf);
    _u8g2->drawStr(0, 50, buf);
    _u8g2->setFont(u8g2_font_6x10_tr);
    _u8g2->drawStr(80, 50, "m/s");

    // Glide ratio (right side)
    if (fd.glideRatio > 0 && fd.glideRatio < 100) {
        snprintf(buf, sizeof(buf), "L/D %.0f", fd.glideRatio);
        _u8g2->drawStr(80, 16, buf);
    }

    // Ground speed
    if (fd.gpsFix) {
        snprintf(buf, sizeof(buf), "%dkm/h", (int)fd.gpsSpeed);
        _u8g2->drawStr(80, 40, buf);
    }
}

void DisplayManager::drawStatsPage(const FlightData& fd) {
    char buf[24];

    _u8g2->setFont(u8g2_font_6x10_tr);
    _u8g2->drawStr(0, 10, "-- Flight Stats --");

    // Flight time
    unsigned long dur = fd.flightDurationMs;
    uint16_t mins = (dur / 60000) % 60;
    uint16_t hrs = dur / 3600000;
    uint16_t secs = (dur / 1000) % 60;
    snprintf(buf, sizeof(buf), "Time: %02d:%02d:%02d", hrs, mins, secs);
    _u8g2->drawStr(0, 24, buf);

    // Max altitude
    snprintf(buf, sizeof(buf), "Max Alt: %dm", (int)_maxAlt);
    _u8g2->drawStr(0, 36, buf);

    // Max climb
    dtostrf(_maxClimb, 4, 1, buf);
    char line[24];
    snprintf(line, sizeof(line), "Max Up: %s m/s", buf);
    _u8g2->drawStr(0, 48, line);

    // Max sink
    dtostrf(_maxSink, 4, 1, buf);
    snprintf(line, sizeof(line), "Max Dn: %s m/s", buf);
    _u8g2->drawStr(0, 60, line);
}

void DisplayManager::drawSettingsPage(const FlightData& fd) {
    char buf[24];

    _u8g2->setFont(u8g2_font_6x10_tr);
    _u8g2->drawStr(0, 10, "-- Settings --");

    // Battery
    snprintf(buf, sizeof(buf), "Batt: %.1fV %d%%", fd.batteryV, fd.batteryPct);
    _u8g2->drawStr(0, 24, buf);

    // GPS
    snprintf(buf, sizeof(buf), "GPS: %d sats %s", fd.gpsSats, fd.gpsFix ? "FIX" : "---");
    _u8g2->drawStr(0, 36, buf);

    // Temperature
    snprintf(buf, sizeof(buf), "Temp: %.1fC", fd.temperature / 100.0f);
    _u8g2->drawStr(0, 48, buf);

    // Pressure
    snprintf(buf, sizeof(buf), "QNH: %dhPa", (int)(fd.pressure / 100));
    _u8g2->drawStr(0, 60, buf);
}

void DisplayManager::drawStatusBar(const FlightData& fd) {
    // Top-right status icons
    uint8_t x = 128;

    _u8g2->setFont(u8g2_font_4x6_tr);

    // Battery icon
    x -= 16;
    char pct[5];
    snprintf(pct, sizeof(pct), "%d%%", fd.batteryPct);
    _u8g2->drawStr(x, 6, pct);

    // GPS satellite count
    if (fd.gpsFix) {
        x -= 12;
        char sat[4];
        snprintf(sat, sizeof(sat), "%d", fd.gpsSats);
        _u8g2->drawStr(x, 6, sat);
    }
}

void DisplayManager::showSplash(const char* name, const char* version) {
    if (!_u8g2) return;
    _u8g2->clearBuffer();
    _u8g2->setFont(u8g2_font_9x15B_tr);
    int w = _u8g2->getStrWidth(name);
    _u8g2->drawStr((128 - w) / 2, 30, name);
    _u8g2->setFont(u8g2_font_6x10_tr);
    w = _u8g2->getStrWidth(version);
    _u8g2->drawStr((128 - w) / 2, 48, version);
    _u8g2->sendBuffer();
}

void DisplayManager::showMessage(const char* msg) {
    strncpy(_message, msg, sizeof(_message) - 1);
    _message[sizeof(_message) - 1] = '\0';
    _messageUntil = millis() + 2000;
}

void DisplayManager::setBrightness(uint8_t brightness) {
    if (_u8g2) _u8g2->setContrast(brightness);
}

void DisplayManager::setEnabled(bool on) {
    if (!_u8g2) return;
    if (on) {
        _u8g2->setPowerSave(0);
    } else {
        _u8g2->setPowerSave(1);
    }
}
