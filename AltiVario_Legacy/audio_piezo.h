#ifndef AUDIO_PIEZO_H
#define AUDIO_PIEZO_H

#include <Arduino.h>
#include <Tone.h>
#include <AltiVario_Audio.h>

// Piezo audio driver for ATmega328P
// Supports single or dual piezo configuration
class AudioPiezo {
public:
    AudioPiezo();

    // Initialize with volume level (0=mute, 1=single, 2=dual)
    void begin(uint8_t pin1, uint8_t pin2, uint8_t volume);

    // Play a tone based on ToneParams from AudioPatternEngine
    void play(const ToneParams& tp);

    // Stop all sound
    void stop();

    // Play startup melody
    void playWelcome();

    uint8_t volume() const { return _volume; }

private:
    Tone _tone1;
    Tone _tone2;
    uint8_t _pin2;
    uint8_t _volume;
};

#endif // AUDIO_PIEZO_H
