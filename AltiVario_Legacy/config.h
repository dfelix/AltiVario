#ifndef CONFIG_H
#define CONFIG_H

// ─── Pin Assignments ───────────────────────────────────────────────
#define PIN_LED_RED        3
#define PIN_LED_GREEN      4
#define PIN_SPEAKER1       8
#define PIN_SPEAKER2       2
#define PIN_NMEA_LED       6

// ─── Serial ────────────────────────────────────────────────────────
#define UART_SPEED         9600

// ─── NMEA Protocol ─────────────────────────────────────────────────
// 1 = LK8EX1, 2 = FlymasterF1, 0 = both
#define NMEA_PROTOCOL      2
#define NMEA_OUT_PER_SEC   3

// ─── Vario ─────────────────────────────────────────────────────────
#define VARIO_SAMPLES      40    // regression window
#define VARIO_MAX_SAMPLES  50    // ring buffer size
#define PRESSURE_FILTER_SIZE 25  // averaging filter size
#define CLIMB_THRESHOLD    0.5f  // m/s to start climb beep
#define SINK_THRESHOLD    -1.1f  // m/s to start sink alarm

// ─── Audio ─────────────────────────────────────────────────────────
// 0 = mute, 1 = single piezo, 2 = dual piezo (louder)
#define VOLUME             2

// ─── Timing ────────────────────────────────────────────────────────
#define TEMP_CHECK_MS      1000
#define BATT_CHECK_MS      10000
#define NMEA_INTERVAL_MS   (1000 / NMEA_OUT_PER_SEC)

// ─── Battery ───────────────────────────────────────────────────────
#define BATT_GOOD_V        3.60f
#define BATT_WARN_V        3.40f
#define BATT_ADC_MIN_MV    2500
#define BATT_ADC_MAX_MV    4300

// ─── Sensor ────────────────────────────────────────────────────────
#define BMP085_MODE        MODE_ULTRA_HIGHRES

#endif // CONFIG_H
