/*
 * AltiVario Legacy — ATmega328P + BMP085 Variometer
 *
 * Refactored from original vario.ino with bug fixes:
 * - Ring buffer averaging (fixes OOB array access)
 * - unsigned long timestamps (fixes float precision loss)
 * - Division-by-zero guard in vario regression
 * - Missing break statements in switch/case (audio)
 * - snprintf NMEA formatting (eliminates String heap fragmentation)
 * - Startup validity check (no false vario readings)
 * - Millis rollover-safe timing
 *
 * References:
 *   http://taturno.com/2011/10/30/variometro-la-rivincita/
 *   http://www.daqq.eu/index.php?show=prj_sanity_nullifier
 *   http://code.google.com/p/tinkerit/wiki/SecretVoltmeter
 */

#include <Wire.h>
#include <avr/wdt.h>

#include <AltiVario_Vario.h>
#include <AltiVario_NMEA.h>
#include <AltiVario_Audio.h>

#include "config.h"
#include "sensor_bmp085.h"
#include "audio_piezo.h"
#include "battery_avr.h"
#include "led_status.h"

// ─── State ─────────────────────────────────────────────────────────
enum SystemState {
    STATE_INIT,
    STATE_RUNNING,
    STATE_SENSOR_FAIL
};

static SystemState state = STATE_INIT;

// ─── Objects ───────────────────────────────────────────────────────
static SensorBMP085 sensor;
static AudioPiezo audio;
static BatteryAVR battery;
static LEDStatus led;

static VarioComputer vario(VARIO_SAMPLES, VARIO_MAX_SAMPLES);
static AveragingFilter<PRESSURE_FILTER_SIZE> pressureFilter;
static AudioPatternEngine audioEngine;

// ─── NMEA state ────────────────────────────────────────────────────
static float nmeaAlt = 0;
static float nmeaOldAlt = 0;
static float nmeaVarioCms = 0;

// ─── Timing (all rollover-safe) ────────────────────────────────────
static unsigned long lastTempCheck = 0;
static unsigned long lastBattCheck = 0;
static unsigned long lastNmeaOut = 0;
static unsigned long lastNmeaVario = 0;
static unsigned long lastBeep = 0;
static uint16_t currentPeriod = 500;  // current beep period

// ─── Sensor data ───────────────────────────────────────────────────
static long temperature_tenths = 0;
static float batteryV = 0;
static long batteryMv = 0;

// ─── Setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(UART_SPEED);

    // Initialize LED first for visual feedback
    led.begin(PIN_LED_RED, PIN_LED_GREEN);

    // Initialize audio
    audio.begin(PIN_SPEAKER1, PIN_SPEAKER2, VOLUME);

    // NMEA LED
    pinMode(PIN_NMEA_LED, OUTPUT);

    // Configure audio engine thresholds
    audioEngine.setClimbThreshold(CLIMB_THRESHOLD);
    audioEngine.setSinkThreshold(SINK_THRESHOLD);

    // Initialize sensor
    if (sensor.begin()) {
        state = STATE_RUNNING;
        // audio.playWelcome();  // uncomment to enable startup melody
    } else {
        state = STATE_SENSOR_FAIL;
        led.setRed();
    }

    // Initialize timers
    unsigned long now = millis();
    lastTempCheck = now;
    lastBattCheck = now;
    lastNmeaOut = now;
    lastNmeaVario = now;
    lastBeep = now;

    // Enable watchdog (2 second timeout)
    wdt_enable(WDTO_2S);
}

// ─── Main Loop ─────────────────────────────────────────────────────
void loop() {
    wdt_reset();
    unsigned long now = millis();

    if (state == STATE_SENSOR_FAIL) {
        // Try to recover sensor every 2 seconds
        if ((now - lastTempCheck) >= 2000) {
            lastTempCheck = now;
            SensorBMP085::recoverI2CBus();
            if (sensor.begin()) {
                state = STATE_RUNNING;
                vario.reset();
                pressureFilter.reset();
            }
        }
        return;
    }

    // ── Read pressure ──────────────────────────────────────────────
    long pressure = 0;
    if (!sensor.readPressure(pressure)) {
        if (!sensor.isOk()) {
            state = STATE_SENSOR_FAIL;
            led.setRed();
        }
        return;
    }

    // ── Compute altitude + vario ───────────────────────────────────
    float altitude = pressureToAltitude((float)pressure);
    vario.update(altitude, now);

    long avgPressure = pressureFilter.update(pressure);
    float avgAlt = pressureToAltitude((float)avgPressure);

    // ── NMEA vario (1 Hz, from averaged altitude) ──────────────────
    if ((now - lastNmeaVario) >= 1000) {
        nmeaVarioCms = (avgAlt - nmeaOldAlt) * 100.0f;
        nmeaOldAlt = avgAlt;
        nmeaAlt = avgAlt;
        lastNmeaVario = now;
    }

    // ── Audio ──────────────────────────────────────────────────────
    if (vario.isValid()) {
        float v = vario.getVario();
        ToneParams tp = audioEngine.compute(v);

        if (tp.frequency > 0) {
            if ((now - lastBeep) >= currentPeriod) {
                audio.play(tp);
                currentPeriod = tp.period;
                lastBeep = now;
            }
        }
    }

    // ── Temperature (periodic) ─────────────────────────────────────
    if ((now - lastTempCheck) >= TEMP_CHECK_MS) {
        sensor.readTemperature(temperature_tenths);
        lastTempCheck = now;
    }

    // ── Battery (periodic) ─────────────────────────────────────────
    if ((now - lastBattCheck) >= BATT_CHECK_MS) {
        batteryMv = battery.readVcc_mV();
        batteryV = (float)batteryMv / 1000.0f;
        led.updateBattery(batteryV, BATT_GOOD_V, BATT_WARN_V);
        lastBattCheck = now;
    }

    // ── NMEA output (periodic) ─────────────────────────────────────
    if ((now - lastNmeaOut) >= NMEA_INTERVAL_MS) {
        NMEAVarioData data;
        data.pressure_pa = avgPressure;
        data.altitude_m = nmeaAlt;
        data.vario_cms = nmeaVarioCms;
        data.temperature_c = (int)(temperature_tenths / 10);
        data.battery_v = batteryV;
        data.battery_pct = battery.percentage(batteryMv, BATT_ADC_MIN_MV, BATT_ADC_MAX_MV);

        char buf[80];
        bool sent = false;

        if (NMEA_PROTOCOL == 1 || NMEA_PROTOCOL == 0) {
            if (formatLK8EX1(buf, sizeof(buf), data)) {
                Serial.print(buf);
                sent = true;
            }
        }
        if (NMEA_PROTOCOL == 2 || NMEA_PROTOCOL == 0) {
            if (formatFlymasterF1(buf, sizeof(buf), data)) {
                Serial.print(buf);
                sent = true;
            }
        }

        if (sent) {
            // Toggle NMEA LED
            digitalWrite(PIN_NMEA_LED, !digitalRead(PIN_NMEA_LED));
        }

        lastNmeaOut = now;
    }
}
