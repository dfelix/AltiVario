#ifndef ALTIVARIO_AUDIO_H
#define ALTIVARIO_AUDIO_H

#include <Arduino.h>

// Output from the audio pattern engine
struct ToneParams {
    uint16_t frequency;   // Hz (0 = silence)
    uint16_t duration;    // ms (tone on time)
    uint16_t period;      // ms (full cycle: on + off)
};

// Audio pattern engine: computes tone parameters from vario rate
// Platform-independent — does not produce sound, only computes parameters
class AudioPatternEngine {
public:
    AudioPatternEngine();

    // Configure thresholds (m/s)
    void setClimbThreshold(float threshold) { _climbThreshold = threshold; }
    void setSinkThreshold(float threshold) { _sinkThreshold = threshold; }
    float climbThreshold() const { return _climbThreshold; }
    float sinkThreshold() const { return _sinkThreshold; }

    // Compute tone parameters for given vario rate (m/s)
    ToneParams compute(float vario) const;

    // Check if enough time has elapsed for a new beep cycle (rollover-safe)
    // period_ms: the period from the last ToneParams
    bool shouldBeep(unsigned long now_ms, uint16_t period_ms) const;

    // Call when a beep cycle starts
    void markBeep(unsigned long now_ms);

private:
    float _climbThreshold;
    float _sinkThreshold;
    unsigned long _lastBeepMs;

    static uint16_t clampFreq(int32_t freq);
    static uint16_t clampDuration(int32_t dur);
    static uint16_t clampPeriod(int32_t per);
};

#endif // ALTIVARIO_AUDIO_H
