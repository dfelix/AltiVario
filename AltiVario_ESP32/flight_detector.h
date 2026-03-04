#ifndef FLIGHT_DETECTOR_H
#define FLIGHT_DETECTOR_H

#include <Arduino.h>

enum FlightState {
    FLIGHT_GROUND,
    FLIGHT_LAUNCHING,
    FLIGHT_IN_FLIGHT,
    FLIGHT_LANDING
};

// Automatic flight state detection based on vario and ground speed
class FlightDetector {
public:
    FlightDetector();

    void reset();

    // Update with current vario (m/s), ground speed (km/h), and timestamp
    void update(float vario, float groundSpeedKmh, unsigned long now_ms);

    FlightState state() const { return _state; }
    bool stateChanged() const { return _stateChanged; }

    unsigned long flightStartMs() const { return _flightStartMs; }
    unsigned long flightDurationMs(unsigned long now_ms) const;

private:
    FlightState _state;
    bool _stateChanged;
    unsigned long _stateEntryMs;
    unsigned long _flightStartMs;

    // Thresholds
    static const float LAUNCH_VARIO_THRESH;      // m/s sustained climb
    static const float LAUNCH_SPEED_THRESH;       // km/h ground speed
    static const unsigned long LAUNCH_HOLD_MS;    // sustain time to confirm
    static const float LAND_SPEED_THRESH;         // km/h below this = stopped
    static const float LAND_VARIO_THRESH;         // m/s nearly zero vario
    static const unsigned long LAND_HOLD_MS;      // sustain time to confirm landing

    bool _launchCondMet;
    bool _landCondMet;
    unsigned long _condStartMs;
};

#endif // FLIGHT_DETECTOR_H
