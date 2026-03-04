#include "AltiVario_Audio.h"

static const uint16_t FREQ_MIN  = 200;
static const uint16_t FREQ_MAX  = 4000;
static const uint16_t DUR_MIN   = 50;
static const uint16_t DUR_MAX   = 500;
static const uint16_t PER_MIN   = 100;
static const uint16_t PER_MAX   = 1000;

AudioPatternEngine::AudioPatternEngine()
    : _climbThreshold(0.5f)
    , _sinkThreshold(-1.1f)
    , _lastBeepMs(0)
{
}

uint16_t AudioPatternEngine::clampFreq(int32_t freq) {
    if (freq < FREQ_MIN) return FREQ_MIN;
    if (freq > FREQ_MAX) return FREQ_MAX;
    return (uint16_t)freq;
}

uint16_t AudioPatternEngine::clampDuration(int32_t dur) {
    if (dur < DUR_MIN) return DUR_MIN;
    if (dur > DUR_MAX) return DUR_MAX;
    return (uint16_t)dur;
}

uint16_t AudioPatternEngine::clampPeriod(int32_t per) {
    if (per < PER_MIN) return PER_MIN;
    if (per > PER_MAX) return PER_MAX;
    return (uint16_t)per;
}

ToneParams AudioPatternEngine::compute(float vario) const {
    ToneParams tp = {0, 0, 0};

    if (vario > _climbThreshold && vario <= 10.0f) {
        // Normal climb: frequency rises, period shortens with stronger lift
        // Ported from original: freq = 1400 + 200*vario, dur = 420 - vario*(20+vario)
        // period = 550 - vario*(30+vario)
        tp.frequency = clampFreq((int32_t)(1400.0f + 200.0f * vario));
        tp.duration  = clampDuration((int32_t)(420.0f - vario * (20.0f + vario)));
        tp.period    = clampPeriod((int32_t)(550.0f - vario * (30.0f + vario)));
    }
    else if (vario > 10.0f) {
        // Strong climb: max urgency
        tp.frequency = clampFreq(3450);
        tp.duration  = 120;
        tp.period    = 160;
    }
    else if (vario < _sinkThreshold) {
        // Sink alarm: low continuous tone
        tp.frequency = clampFreq(300);
        tp.duration  = 340;
        tp.period    = 200;
    }
    // else: dead band — no tone (frequency = 0)

    return tp;
}

bool AudioPatternEngine::shouldBeep(unsigned long now_ms, uint16_t period_ms) const {
    // Rollover-safe: unsigned subtraction handles wrap-around correctly
    return (now_ms - _lastBeepMs) >= period_ms;
}

void AudioPatternEngine::markBeep(unsigned long now_ms) {
    _lastBeepMs = now_ms;
}
