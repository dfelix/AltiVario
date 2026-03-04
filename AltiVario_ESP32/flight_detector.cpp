#include "flight_detector.h"

const float FlightDetector::LAUNCH_VARIO_THRESH = 0.5f;
const float FlightDetector::LAUNCH_SPEED_THRESH = 10.0f;
const unsigned long FlightDetector::LAUNCH_HOLD_MS = 5000;
const float FlightDetector::LAND_SPEED_THRESH = 3.0f;
const float FlightDetector::LAND_VARIO_THRESH = 0.3f;
const unsigned long FlightDetector::LAND_HOLD_MS = 60000;  // 1 minute

FlightDetector::FlightDetector()
    : _state(FLIGHT_GROUND)
    , _stateChanged(false)
    , _stateEntryMs(0)
    , _flightStartMs(0)
    , _launchCondMet(false)
    , _landCondMet(false)
    , _condStartMs(0)
{
}

void FlightDetector::reset() {
    _state = FLIGHT_GROUND;
    _stateChanged = false;
    _stateEntryMs = 0;
    _flightStartMs = 0;
    _launchCondMet = false;
    _landCondMet = false;
    _condStartMs = 0;
}

void FlightDetector::update(float vario, float groundSpeedKmh, unsigned long now_ms) {
    _stateChanged = false;
    FlightState newState = _state;

    switch (_state) {
        case FLIGHT_GROUND: {
            // Detect launch: sustained climb or ground speed
            bool launchCond = (vario > LAUNCH_VARIO_THRESH) ||
                              (groundSpeedKmh > LAUNCH_SPEED_THRESH);
            if (launchCond && !_launchCondMet) {
                _launchCondMet = true;
                _condStartMs = now_ms;
            } else if (!launchCond) {
                _launchCondMet = false;
            }

            if (_launchCondMet && (now_ms - _condStartMs) >= LAUNCH_HOLD_MS) {
                newState = FLIGHT_LAUNCHING;
            }
            break;
        }

        case FLIGHT_LAUNCHING:
            // Transition immediately to in-flight
            newState = FLIGHT_IN_FLIGHT;
            _flightStartMs = now_ms;
            _landCondMet = false;
            break;

        case FLIGHT_IN_FLIGHT: {
            // Detect landing: low speed and low vario for extended period
            bool landCond = (groundSpeedKmh < LAND_SPEED_THRESH) &&
                            (fabsf(vario) < LAND_VARIO_THRESH);
            if (landCond && !_landCondMet) {
                _landCondMet = true;
                _condStartMs = now_ms;
            } else if (!landCond) {
                _landCondMet = false;
            }

            if (_landCondMet && (now_ms - _condStartMs) >= LAND_HOLD_MS) {
                newState = FLIGHT_LANDING;
            }
            break;
        }

        case FLIGHT_LANDING:
            // Transition to ground
            newState = FLIGHT_GROUND;
            _launchCondMet = false;
            break;
    }

    if (newState != _state) {
        _state = newState;
        _stateEntryMs = now_ms;
        _stateChanged = true;
    }
}

unsigned long FlightDetector::flightDurationMs(unsigned long now_ms) const {
    if (_state == FLIGHT_IN_FLIGHT && _flightStartMs > 0) {
        return now_ms - _flightStartMs;
    }
    return 0;
}
