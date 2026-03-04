#ifndef ALTIVARIO_NMEA_H
#define ALTIVARIO_NMEA_H

#include <Arduino.h>

// Data structure for passing vario data to NMEA formatters
struct NMEAVarioData {
    long pressure_pa;        // raw pressure in Pa
    float altitude_m;        // altitude in meters
    float vario_cms;         // vario in cm/s
    int temperature_c;       // temperature in degrees C
    float battery_v;         // battery voltage
    int battery_pct;         // battery percentage (0-100), -1 if unavailable
};

// Compute NMEA XOR checksum over a string (excluding $ and *)
uint8_t computeNMEAChecksum(const char* sentence);

// Format LK8EX1 sentence into caller-provided buffer
// Returns length written (excluding null terminator), 0 on error
// Buffer must be at least 80 bytes
// Output: $LK8EX1,pressure,altitude,vario,temperature,battery,*XX\r\n
uint8_t formatLK8EX1(char* buf, uint8_t bufSize, const NMEAVarioData& data);

// Format Flymaster VARIO sentence into caller-provided buffer
// Returns length written (excluding null terminator), 0 on error
// Buffer must be at least 80 bytes
// Output: $VARIO,fPressure,fVario,Bat1Volts,Bat2Volts,BatBank,Temp1,Temp2*XX\r\n
uint8_t formatFlymasterF1(char* buf, uint8_t bufSize, const NMEAVarioData& data);

#endif // ALTIVARIO_NMEA_H
