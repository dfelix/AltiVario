#include "AltiVario_NMEA.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t computeNMEAChecksum(const char* sentence) {
    uint8_t cs = 0;
    while (*sentence) {
        cs ^= (uint8_t)*sentence;
        sentence++;
    }
    return cs;
}

uint8_t formatLK8EX1(char* buf, uint8_t bufSize, const NMEAVarioData& data) {
    if (!buf || bufSize < 80) return 0;

    // Build payload: LK8EX1,pressure,altitude,vario,temperature,battery,
    char payload[64];
    char alt_str[8];
    char var_str[8];

    dtostrf(data.altitude_m, 0, 0, alt_str);
    dtostrf(data.vario_cms, 0, 0, var_str);

    int batt;
    if (data.battery_pct >= 0) {
        batt = 1000 + constrain(data.battery_pct, 0, 100);
    } else {
        batt = 999;  // not available
    }

    snprintf(payload, sizeof(payload), "LK8EX1,%ld,%s,%s,%d,%d,",
             data.pressure_pa, alt_str, var_str,
             data.temperature_c, batt);

    uint8_t cs = computeNMEAChecksum(payload);
    int len = snprintf(buf, bufSize, "$%s*%02X\r\n", payload, cs);
    return (len > 0 && len < bufSize) ? (uint8_t)len : 0;
}

uint8_t formatFlymasterF1(char* buf, uint8_t bufSize, const NMEAVarioData& data) {
    if (!buf || bufSize < 80) return 0;

    // Build payload: VARIO,fPressure,fVario,Bat1Volts,Bat2Volts,BatBank,Temp1,Temp2
    char payload[64];
    char press_str[12];
    char vario_str[8];
    char batt_str[6];

    // Pressure in hPa (integer)
    dtostrf((float)data.pressure_pa / 100.0f, 0, 0, press_str);
    // Vario in dm/s (from cm/s)
    dtostrf(data.vario_cms / 10.0f, 2, 2, vario_str);
    // Battery voltage
    dtostrf(data.battery_v, 2, 2, batt_str);

    snprintf(payload, sizeof(payload), "VARIO,%s,%s,%s,0,1,%d,0",
             press_str, vario_str, batt_str, data.temperature_c);

    uint8_t cs = computeNMEAChecksum(payload);
    int len = snprintf(buf, bufSize, "$%s*%02X\r\n", payload, cs);
    return (len > 0 && len < bufSize) ? (uint8_t)len : 0;
}
