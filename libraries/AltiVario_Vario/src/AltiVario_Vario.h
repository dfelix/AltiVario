#ifndef ALTIVARIO_VARIO_H
#define ALTIVARIO_VARIO_H

#include <Arduino.h>
#include <math.h>

// Barometric formula constants
static const float BARO_CONST_A = 44330.0f;
static const float BARO_CONST_EXP = 0.190295f;
static const float SEA_LEVEL_PRESSURE_PA = 101325.0f;

// Convert pressure (Pa) to altitude (m) using the barometric formula
float pressureToAltitude(float pressure_pa, float ref_pressure_pa = SEA_LEVEL_PRESSURE_PA);

// ─── AveragingFilter ───────────────────────────────────────────────
// Moving average filter with ring buffer (fixes OOB bug from original)

template <uint8_t SIZE>
class AveragingFilter {
public:
    AveragingFilter() : _sum(0), _index(0), _count(0) {
        memset(_buf, 0, sizeof(_buf));
    }

    long update(long input) {
        _sum -= _buf[_index];
        _buf[_index] = input;
        _sum += input;
        _index = (_index + 1) % SIZE;
        if (_count < SIZE) _count++;
        return _sum / _count;  // correct average during startup
    }

    void reset() {
        memset(_buf, 0, sizeof(_buf));
        _sum = 0;
        _index = 0;
        _count = 0;
    }

    uint8_t count() const { return _count; }
    bool isFull() const { return _count >= SIZE; }

private:
    long _buf[SIZE];
    long _sum;
    uint8_t _index;
    uint8_t _count;
};

// ─── VarioComputer ─────────────────────────────────────────────────
// Linear regression vario using ring buffer with unsigned long timestamps

class VarioComputer {
public:
    VarioComputer(uint8_t window_size = 40, uint8_t max_size = 50);
    ~VarioComputer();

    // Call every loop iteration with current altitude and millis() timestamp
    void update(float altitude, unsigned long timestamp_ms);

    // Get computed vario rate in m/s
    float getVario() const { return _vario; }

    // True once enough samples have been collected
    bool isValid() const { return _count >= _window; }

    void reset();

private:
    float* _alt;
    unsigned long* _tim;
    uint8_t _maxSize;
    uint8_t _window;
    uint8_t _head;     // ring buffer write position
    uint8_t _count;    // samples collected so far
    float _vario;
};

#endif // ALTIVARIO_VARIO_H
