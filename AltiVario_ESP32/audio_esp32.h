#ifndef AUDIO_ESP32_H
#define AUDIO_ESP32_H

#include <Arduino.h>
#include <AltiVario_Audio.h>

// ESP32 audio driver: LEDC PWM for piezo, I2S DAC for speaker
class AudioESP32 {
public:
    AudioESP32();

    // Initialize piezo on LEDC channel
    void beginPiezo(uint8_t pin, uint8_t ledcChannel = 0);

    // Initialize I2S DAC (MAX98357A)
    void beginDAC(uint8_t bclkPin, uint8_t lrclkPin, uint8_t doutPin);

    // Play tone from ToneParams (uses current mode)
    void play(const ToneParams& tp);

    // Stop all output
    void stop();

    // Set audio mode (0=piezo, 1=DAC)
    void setMode(uint8_t mode) { _mode = mode; }
    uint8_t mode() const { return _mode; }

    // Set volume (0-100)
    void setVolume(uint8_t vol) { _volume = vol; }
    uint8_t volume() const { return _volume; }

private:
    uint8_t _piezoPin;
    uint8_t _ledcChannel;
    uint8_t _mode;
    uint8_t _volume;
    bool _piezoReady;
    bool _dacReady;

    void playPiezo(uint16_t freq, uint16_t duration);
    void stopPiezo();
    void playDAC(uint16_t freq, uint16_t duration);
    void stopDAC();

    // I2S DAC pins
    uint8_t _bclkPin;
    uint8_t _lrclkPin;
    uint8_t _doutPin;
};

#endif // AUDIO_ESP32_H
