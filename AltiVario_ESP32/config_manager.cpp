#include "config_manager.h"
#include "config.h"

ConfigManager::ConfigManager() {}

void ConfigManager::loadDefaults() {
    _cfg.climbThreshold = DEFAULT_CLIMB_THRESHOLD;
    _cfg.sinkThreshold = DEFAULT_SINK_THRESHOLD;
    _cfg.audioMode = DEFAULT_AUDIO_MODE;
    _cfg.volume = DEFAULT_VOLUME;
    _cfg.bleEnabled = 1;
    _cfg.gpsEnabled = 1;
    _cfg.sdEnabled = 1;
    _cfg.displayBrightness = 255;
    _cfg.qnh = 101325.0f;
}

void ConfigManager::begin() {
    loadDefaults();
    _prefs.begin("altivario", false);

    if (!_prefs.isKey("init")) {
        // First boot — save defaults
        save();
        _prefs.putBool("init", true);
        return;
    }

    _cfg.climbThreshold    = _prefs.getFloat("climbThr", _cfg.climbThreshold);
    _cfg.sinkThreshold     = _prefs.getFloat("sinkThr", _cfg.sinkThreshold);
    _cfg.audioMode         = _prefs.getUChar("audioMode", _cfg.audioMode);
    _cfg.volume            = _prefs.getUChar("volume", _cfg.volume);
    _cfg.bleEnabled        = _prefs.getUChar("bleOn", _cfg.bleEnabled);
    _cfg.gpsEnabled        = _prefs.getUChar("gpsOn", _cfg.gpsEnabled);
    _cfg.sdEnabled         = _prefs.getUChar("sdOn", _cfg.sdEnabled);
    _cfg.displayBrightness = _prefs.getUChar("dispBrt", _cfg.displayBrightness);
    _cfg.qnh               = _prefs.getFloat("qnh", _cfg.qnh);
}

void ConfigManager::save() {
    _prefs.putFloat("climbThr", _cfg.climbThreshold);
    _prefs.putFloat("sinkThr", _cfg.sinkThreshold);
    _prefs.putUChar("audioMode", _cfg.audioMode);
    _prefs.putUChar("volume", _cfg.volume);
    _prefs.putUChar("bleOn", _cfg.bleEnabled);
    _prefs.putUChar("gpsOn", _cfg.gpsEnabled);
    _prefs.putUChar("sdOn", _cfg.sdEnabled);
    _prefs.putUChar("dispBrt", _cfg.displayBrightness);
    _prefs.putFloat("qnh", _cfg.qnh);
}

void ConfigManager::resetDefaults() {
    loadDefaults();
    save();
}
