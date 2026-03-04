#include "AltiVario_Vario.h"

float pressureToAltitude(float pressure_pa, float ref_pressure_pa) {
    if (ref_pressure_pa <= 0.0f || pressure_pa <= 0.0f) return 0.0f;
    return BARO_CONST_A * (1.0f - powf(pressure_pa / ref_pressure_pa, BARO_CONST_EXP));
}

// ─── VarioComputer ─────────────────────────────────────────────────

VarioComputer::VarioComputer(uint8_t window_size, uint8_t max_size)
    : _maxSize(max_size)
    , _window(window_size)
    , _head(0)
    , _count(0)
    , _vario(0.0f)
{
    if (_window > _maxSize) _window = _maxSize;
    _alt = new float[_maxSize];
    _tim = new unsigned long[_maxSize];
    memset(_alt, 0, sizeof(float) * _maxSize);
    memset(_tim, 0, sizeof(unsigned long) * _maxSize);
}

VarioComputer::~VarioComputer() {
    delete[] _alt;
    delete[] _tim;
}

void VarioComputer::update(float altitude, unsigned long timestamp_ms) {
    // Write into ring buffer at head position
    _alt[_head] = altitude;
    _tim[_head] = timestamp_ms;
    _head = (_head + 1) % _maxSize;
    if (_count < _maxSize) _count++;

    // Need at least window samples for valid regression
    uint8_t n = (_count < _window) ? _count : _window;
    if (n < 2) {
        _vario = 0.0f;
        return;
    }

    // Linear regression over the last n samples
    // Oldest sample index in ring buffer
    uint8_t oldest = (_head + _maxSize - n) % _maxSize;
    unsigned long t0 = _tim[oldest];

    float sumT = 0, sumA = 0, sumTT = 0, sumTA = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t idx = (oldest + i) % _maxSize;
        float t = (float)(_tim[idx] - t0);  // ms relative to oldest
        float a = _alt[idx];
        sumT  += t;
        sumA  += a;
        sumTT += t * t;
        sumTA += t * a;
    }

    float denom = (float)n * sumTT - sumT * sumT;
    if (fabsf(denom) < 1e-6f) {
        // Division by zero guard — all timestamps identical
        _vario = 0.0f;
        return;
    }

    // Slope in m/ms, convert to m/s (* 1000)
    _vario = 1000.0f * ((float)n * sumTA - sumT * sumA) / denom;
}

void VarioComputer::reset() {
    _head = 0;
    _count = 0;
    _vario = 0.0f;
    memset(_alt, 0, sizeof(float) * _maxSize);
    memset(_tim, 0, sizeof(unsigned long) * _maxSize);
}
