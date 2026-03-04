#include "igc_logger.h"
#include "gps_manager.h"

// External GPS reference for time
extern GPSManager gps;

IGCLogger::IGCLogger()
    : _recording(false)
    , _sdReady(false)
    , _csPin(0)
    , _bufPos(0)
    , _recordCount(0)
{
    memset(_filename, 0, sizeof(_filename));
    memset(_buf, 0, sizeof(_buf));
}

bool IGCLogger::begin(uint8_t csPin) {
    _csPin = csPin;
    if (!SD.begin(csPin)) {
        _sdReady = false;
        return false;
    }
    _sdReady = true;
    return true;
}

bool IGCLogger::remount() {
    SD.end();
    delay(100);
    return begin(_csPin);
}

void IGCLogger::generateFilename() {
    // IGC filename format: YYYY-MM-DD_HHmm.igc
    if (gps.timeValid()) {
        snprintf(_filename, sizeof(_filename), "/%04d-%02d-%02d_%02d%02d.igc",
                 gps.year(), gps.month(), gps.day(),
                 gps.hour(), gps.minute());
    } else {
        // Fallback: use millis
        snprintf(_filename, sizeof(_filename), "/flight_%lu.igc", millis() / 1000);
    }
}

bool IGCLogger::startFlight(float lat, float lon, float altGPS) {
    if (!_sdReady || _recording) return false;

    generateFilename();
    _file = SD.open(_filename, FILE_WRITE);
    if (!_file) {
        // Try remount
        if (!remount()) return false;
        _file = SD.open(_filename, FILE_WRITE);
        if (!_file) return false;
    }

    _recording = true;
    _bufPos = 0;
    _recordCount = 0;

    writeHRecord();
    return true;
}

void IGCLogger::writeHRecord() {
    char line[80];
    int len;

    // A-record: manufacturer
    len = snprintf(line, sizeof(line), "AXAV AltiVario ESP32\r\n");
    writeToBuffer(line, len);

    // H-records: header
    if (gps.timeValid()) {
        len = snprintf(line, sizeof(line), "HFDTE%02d%02d%02d\r\n",
                       gps.day(), gps.month(), gps.year() % 100);
        writeToBuffer(line, len);
    }

    len = snprintf(line, sizeof(line), "HFPLTPILOTINCHARGE:\r\n");
    writeToBuffer(line, len);

    len = snprintf(line, sizeof(line), "HFGTYGLIDERTYPE:\r\n");
    writeToBuffer(line, len);

    len = snprintf(line, sizeof(line), "HFDTM100GPSDATUM:WGS-84\r\n");
    writeToBuffer(line, len);

    len = snprintf(line, sizeof(line), "HFFTYFRTYPE:AltiVario,ESP32-S3\r\n");
    writeToBuffer(line, len);

    len = snprintf(line, sizeof(line), "HFGPSGPS:u-blox,NEO-6M,16,50000\r\n");
    writeToBuffer(line, len);

    len = snprintf(line, sizeof(line), "HFPRSPRESSALTSENSOR:Measurement Specialties,MS5611,50000\r\n");
    writeToBuffer(line, len);

    // I-record: fix extension indices
    // We include pressure altitude as an extension
    len = snprintf(line, sizeof(line), "I013638FXA\r\n");
    writeToBuffer(line, len);

    flushBuffer();
}

void IGCLogger::formatLatLon(float lat, float lon, char* latStr, char* lonStr) {
    // IGC format: DDMMmmmN/S  DDDMMmmmE/W
    char ns = lat >= 0 ? 'N' : 'S';
    char ew = lon >= 0 ? 'E' : 'W';
    lat = fabsf(lat);
    lon = fabsf(lon);

    int latDeg = (int)lat;
    float latMin = (lat - latDeg) * 60.0f;
    int latMinInt = (int)latMin;
    int latMinFrac = (int)((latMin - latMinInt) * 1000);

    int lonDeg = (int)lon;
    float lonMin = (lon - lonDeg) * 60.0f;
    int lonMinInt = (int)lonMin;
    int lonMinFrac = (int)((lonMin - lonMinInt) * 1000);

    snprintf(latStr, 9, "%02d%02d%03d%c", latDeg, latMinInt, latMinFrac, ns);
    snprintf(lonStr, 10, "%03d%02d%03d%c", lonDeg, lonMinInt, lonMinFrac, ew);
}

void IGCLogger::writeRecord(float lat, float lon, float altGPS,
                             float altBaro, int32_t pressure) {
    if (!_recording) return;

    char line[64];
    char latStr[9], lonStr[10];
    formatLatLon(lat, lon, latStr, lonStr);

    int altG = (int)altGPS;
    int altB = (int)altBaro;

    // B-record: Bhhmmss DDMMmmmN DDDMMmmmE V PPPPP GGGGG
    // V = validity (A=3D, V=2D)
    int len = snprintf(line, sizeof(line), "B%02d%02d%02d%s%s%c%05d%05d\r\n",
                       gps.hour(), gps.minute(), gps.second(),
                       latStr, lonStr,
                       'A',  // 3D fix
                       altB < 0 ? 0 : altB,
                       altG < 0 ? 0 : altG);

    writeToBuffer(line, len);
    _recordCount++;

    // Flush every ~16 records (roughly 16 seconds)
    if (_recordCount % 16 == 0) {
        flushBuffer();
    }
}

void IGCLogger::stopFlight() {
    if (!_recording) return;

    // G-record (security — simplified)
    char line[32];
    int len = snprintf(line, sizeof(line), "G00000000000000\r\n");
    writeToBuffer(line, len);

    flushBuffer();
    _file.close();
    _recording = false;
}

void IGCLogger::writeToBuffer(const char* str, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        _buf[_bufPos++] = str[i];
        if (_bufPos >= BUF_SIZE) {
            flushBuffer();
        }
    }
}

void IGCLogger::flushBuffer() {
    if (_bufPos > 0 && _file) {
        _file.write((uint8_t*)_buf, _bufPos);
        _file.flush();
        _bufPos = 0;
    }
}

uint8_t IGCLogger::listFiles(char filenames[][32], uint8_t maxFiles) {
    if (!_sdReady) return 0;

    File root = SD.open("/");
    if (!root) return 0;

    uint8_t count = 0;
    File entry;
    while ((entry = root.openNextFile()) && count < maxFiles) {
        const char* name = entry.name();
        size_t len = strlen(name);
        if (len > 4 && strcmp(name + len - 4, ".igc") == 0) {
            strncpy(filenames[count], name, 31);
            filenames[count][31] = '\0';
            count++;
        }
        entry.close();
    }
    root.close();
    return count;
}
