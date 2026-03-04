#ifndef IGC_LOGGER_H
#define IGC_LOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>

// IGC flight recorder — writes to SD card in IGC format
// Uses 512-byte write buffer to minimize SD card writes
class IGCLogger {
public:
    IGCLogger();

    // Initialize SD card. Returns true on success.
    bool begin(uint8_t csPin);

    // Start a new flight recording
    bool startFlight(float lat, float lon, float altGPS);

    // Write a B-record (fix record) at 1Hz
    void writeRecord(float lat, float lon, float altGPS,
                     float altBaro, int32_t pressure);

    // End the flight recording
    void stopFlight();

    bool isRecording() const { return _recording; }
    bool isSDReady() const { return _sdReady; }

    // Get current filename for WiFi download
    const char* currentFilename() const { return _filename; }

    // List IGC files for WiFi portal
    uint8_t listFiles(char filenames[][32], uint8_t maxFiles);

    // Remount SD card (error recovery)
    bool remount();

private:
    File _file;
    bool _recording;
    bool _sdReady;
    uint8_t _csPin;
    char _filename[32];

    // Write buffer (512 bytes to match SD sector size)
    static const uint16_t BUF_SIZE = 512;
    char _buf[BUF_SIZE];
    uint16_t _bufPos;

    uint16_t _recordCount;

    void flushBuffer();
    void writeToBuffer(const char* str, uint8_t len);
    void writeHRecord();
    void formatLatLon(float lat, float lon, char* latStr, char* lonStr);
    void generateFilename();
};

#endif // IGC_LOGGER_H
