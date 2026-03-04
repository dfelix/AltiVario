#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <Arduino.h>
#include <Preferences.h>

// Runtime configuration backed by NVS (Non-Volatile Storage)
struct RuntimeConfig {
    float climbThreshold;
    float sinkThreshold;
    uint8_t audioMode;       // 0=piezo, 1=DAC
    uint8_t volume;          // 0-100
    uint8_t bleEnabled;
    uint8_t gpsEnabled;
    uint8_t sdEnabled;
    uint8_t displayBrightness; // 0-255
    float qnh;               // sea level pressure in Pa
};

class ConfigManager {
public:
    ConfigManager();

    // Load from NVS (or set defaults if first boot)
    void begin();

    // Save current config to NVS
    void save();

    // Reset to factory defaults
    void resetDefaults();

    // Access config
    RuntimeConfig& config() { return _cfg; }
    const RuntimeConfig& config() const { return _cfg; }

private:
    Preferences _prefs;
    RuntimeConfig _cfg;
    void loadDefaults();
};

#endif // CONFIG_MANAGER_H
