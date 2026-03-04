#include "audio_piezo.h"

AudioPiezo::AudioPiezo()
    : _pin2(0)
    , _volume(0)
{
}

void AudioPiezo::begin(uint8_t pin1, uint8_t pin2, uint8_t volume) {
    _pin2 = pin2;
    _volume = volume;

    if (_volume >= 1) {
        _tone1.begin(pin1);
        if (_volume >= 2) {
            _tone2.begin(pin2);
        } else {
            // Single piezo: hold pin2 low
            pinMode(pin2, OUTPUT);
            digitalWrite(pin2, LOW);
        }
    }
}

void AudioPiezo::play(const ToneParams& tp) {
    if (_volume == 0 || tp.frequency == 0) return;

    _tone1.play(tp.frequency, tp.duration);
    if (_volume >= 2) {
        // Slight frequency offset for louder dual-piezo sound
        _tone2.play(tp.frequency + 6, tp.duration);
    }
}

void AudioPiezo::stop() {
    if (_volume >= 1) _tone1.stop();
    if (_volume >= 2) _tone2.stop();
}

void AudioPiezo::playWelcome() {
    if (_volume == 0) return;

    for (int f = 300; f <= 1500; f += 100) {
        _tone1.play(f, 200);
        if (_volume >= 2) _tone2.play(f + 5, 200);
        delay(100);
    }
    for (int f = 1500; f >= 100; f -= 100) {
        _tone1.play(f, 200);
        if (_volume >= 2) _tone2.play(f + 5, 200);
        delay(100);
    }
}
