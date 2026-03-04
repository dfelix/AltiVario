#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

#include <Arduino.h>
#include "flight_detector.h"

// Shared flight data structure, protected by mutex in FreeRTOS tasks
struct FlightData {
    // Sensor data
    float altitude;           // m
    float vario;              // m/s
    int32_t pressure;         // Pa
    int32_t temperature;      // 0.01 °C
    bool sensorValid;

    // GPS data
    float gpsLat;
    float gpsLon;
    float gpsAlt;             // m MSL
    float gpsSpeed;           // km/h
    float gpsHeading;         // degrees
    float glideRatio;
    uint8_t gpsSats;
    bool gpsFix;

    // Battery
    float batteryV;
    uint8_t batteryPct;

    // Flight state
    FlightState flightState;
    unsigned long flightStartMs;
    unsigned long flightDurationMs;

    // Display
    uint8_t displayPage;
};

#endif // FLIGHT_DATA_H
