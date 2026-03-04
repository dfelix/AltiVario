#ifndef CONFIG_H
#define CONFIG_H

// ─── Pin Assignments (ESP32-S3) ────────────────────────────────────
// I2C (shared bus: MS5611 @ 0x77, SSD1306 @ 0x3C)
#define PIN_I2C_SDA        8
#define PIN_I2C_SCL        9

// GPS UART1
#define PIN_GPS_RX         17   // ← NEO-6M TX
#define PIN_GPS_TX         18   // → NEO-6M RX

// SD Card SPI
#define PIN_SD_CS          10
#define PIN_SD_MOSI        11
#define PIN_SD_SCK         12
#define PIN_SD_MISO        13

// Audio
#define PIN_PIEZO          38   // LEDC PWM
#define PIN_I2S_BCLK       4   // MAX98357A
#define PIN_I2S_LRCLK      5
#define PIN_I2S_DOUT       6

// UI
#define PIN_BUTTON         0    // Boot button, internal pull-up
#define PIN_BATTERY_ADC    1    // ADC1_CH0, voltage divider

// ─── I2C Addresses ─────────────────────────────────────────────────
#define ADDR_MS5611        0x77
#define ADDR_SSD1306       0x3C

// ─── Vario Settings (defaults, overridable via NVS) ────────────────
#define DEFAULT_CLIMB_THRESHOLD  0.5f
#define DEFAULT_SINK_THRESHOLD  -1.1f
#define DEFAULT_VARIO_SAMPLES    40
#define DEFAULT_VARIO_MAX_SAMPLES 50
#define DEFAULT_PRESSURE_FILTER  25

// ─── Audio ─────────────────────────────────────────────────────────
#define AUDIO_MODE_PIEZO   0
#define AUDIO_MODE_DAC     1
#define DEFAULT_AUDIO_MODE AUDIO_MODE_PIEZO
#define DEFAULT_VOLUME     80   // 0-100

// ─── NMEA / BLE ────────────────────────────────────────────────────
#define BLE_DEVICE_NAME    "AltiVario"
#define BLE_UPDATE_HZ      3

// ─── GPS ───────────────────────────────────────────────────────────
#define GPS_BAUD           9600

// ─── Display ───────────────────────────────────────────────────────
#define DISPLAY_UPDATE_HZ  5
#define DISPLAY_DIM_SEC    60    // auto-dim after N seconds
#define DISPLAY_OFF_SEC    300   // auto-off after N seconds

// ─── Battery ───────────────────────────────────────────────────────
#define BATT_DIVIDER_RATIO 2.0f  // voltage divider ratio
#define BATT_WARN_V        3.4f
#define BATT_STOP_LOG_V    3.2f
#define BATT_SLEEP_V       3.0f
#define BATT_FULL_V        4.2f
#define BATT_EMPTY_V       3.0f
#define BATT_SAMPLES       16

// ─── SD Card / IGC ─────────────────────────────────────────────────
#define IGC_WRITE_BUF_SIZE 512
#define IGC_RECORD_HZ      1

// ─── WiFi Portal ───────────────────────────────────────────────────
#define WIFI_AP_SSID       "AltiVario"
#define WIFI_AP_PASS       ""     // open AP
#define WIFI_PORTAL_PORT   80

// ─── Task Priorities ───────────────────────────────────────────────
#define TASK_PRI_SENSOR    5
#define TASK_PRI_AUDIO     4
#define TASK_PRI_BLE       3
#define TASK_PRI_GPS       3
#define TASK_PRI_DISPLAY   2
#define TASK_PRI_WIFI      2
#define TASK_PRI_IGC       1
#define TASK_PRI_BATTERY   1

// ─── Task Stack Sizes ──────────────────────────────────────────────
#define STACK_SENSOR       4096
#define STACK_AUDIO        2048
#define STACK_BLE          4096
#define STACK_GPS          4096
#define STACK_DISPLAY      4096
#define STACK_IGC          4096
#define STACK_WIFI         8192
#define STACK_BATTERY      2048

#endif // CONFIG_H
