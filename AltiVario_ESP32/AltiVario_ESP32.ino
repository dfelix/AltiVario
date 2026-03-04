/*
 * AltiVario ESP32 — ESP32-S3 + MS5611 Variometer
 *
 * FreeRTOS-based multi-task architecture:
 *   Core 0: WiFi/BLE stack, taskBLE, taskWiFi
 *   Core 1: taskSensor, taskAudio, taskGPS, taskDisplay, taskIGCLog, taskBattery
 *
 * Inter-task communication via FlightData struct protected by mutex,
 * FreeRTOS queues, and task notifications.
 */

#include <Wire.h>
#include <AltiVario_Vario.h>
#include <AltiVario_NMEA.h>
#include <AltiVario_Audio.h>

#include "config.h"
#include "config_manager.h"
#include "flight_data.h"
#include "sensor_ms5611.h"
#include "audio_esp32.h"
#include "ble_telemetry.h"
#include "battery_esp32.h"
#include "button_handler.h"
#include "gps_manager.h"
#include "display_manager.h"
#include "igc_logger.h"
#include "wifi_portal.h"

static FlightData flightData;
static SemaphoreHandle_t flightDataMutex;

// ─── Objects ──────────────────────────────────────────────────────
static ConfigManager configMgr;
static SensorMS5611 sensor(ADDR_MS5611);
static AudioESP32 audioDriver;
static AudioPatternEngine audioEngine;
static BLETelemetry ble;
static BatteryESP32 battery;
static ButtonHandler button;
static GPSManager gps;
static DisplayManager display;
static FlightDetector flightDetector;
static IGCLogger igcLogger;
static WiFiPortal wifiPortal;

static VarioComputer varioComputer(DEFAULT_VARIO_SAMPLES, DEFAULT_VARIO_MAX_SAMPLES);
static AveragingFilter<DEFAULT_PRESSURE_FILTER> pressureFilter;

// ─── Task handles ─────────────────────────────────────────────────
static TaskHandle_t hTaskSensor  = nullptr;
static TaskHandle_t hTaskAudio   = nullptr;
static TaskHandle_t hTaskBLE     = nullptr;
static TaskHandle_t hTaskGPS     = nullptr;
static TaskHandle_t hTaskDisplay = nullptr;
static TaskHandle_t hTaskIGC     = nullptr;
static TaskHandle_t hTaskBattery = nullptr;
static TaskHandle_t hTaskWiFi    = nullptr;

// ─── Queues ───────────────────────────────────────────────────────
static QueueHandle_t audioQueue;  // ToneParams from sensor→audio

// ─── Forward declarations ─────────────────────────────────────────
void taskSensor(void* param);
void taskAudio(void* param);
void taskBLE(void* param);
void taskGPS(void* param);
void taskDisplay(void* param);
void taskIGCLog(void* param);
void taskBattery(void* param);

// ─── Setup ────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("[AltiVario] Starting...");

    // I2C
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    // Load config from NVS
    configMgr.begin();
    RuntimeConfig& cfg = configMgr.config();

    // Mutex and queues
    flightDataMutex = xSemaphoreCreateMutex();
    audioQueue = xQueueCreate(1, sizeof(ToneParams));
    memset(&flightData, 0, sizeof(flightData));

    // Audio engine
    audioEngine.setClimbThreshold(cfg.climbThreshold);
    audioEngine.setSinkThreshold(cfg.sinkThreshold);

    // Sensor
    if (!sensor.begin(Wire)) {
        Serial.println("[Sensor] MS5611 init FAILED");
    } else {
        Serial.println("[Sensor] MS5611 OK");
    }

    // Audio
    audioDriver.beginPiezo(PIN_PIEZO);
    audioDriver.setVolume(cfg.volume);
    audioDriver.setMode(cfg.audioMode);

    // BLE
    if (cfg.bleEnabled) {
        ble.begin(BLE_DEVICE_NAME);
        Serial.println("[BLE] Started");
    }

    // Battery
    battery.begin(PIN_BATTERY_ADC, BATT_DIVIDER_RATIO, BATT_SAMPLES);

    // Button
    button.begin(PIN_BUTTON, true);

    // GPS
    if (cfg.gpsEnabled) {
        gps.begin(PIN_GPS_RX, PIN_GPS_TX, GPS_BAUD);
        Serial.println("[GPS] Started");
    }

    // Display
    display.begin(PIN_I2C_SDA, PIN_I2C_SCL, ADDR_SSD1306);
    display.showSplash("AltiVario", "v2.0");
    delay(1000);

    // SD Card / IGC
    if (cfg.sdEnabled) {
        igcLogger.begin(PIN_SD_CS);
    }

    // Flight detector
    flightDetector.reset();

    // ── Create FreeRTOS tasks ─────────────────────────────────────

    // Core 1: Application tasks
    xTaskCreatePinnedToCore(taskSensor,  "sensor",  STACK_SENSOR,  nullptr, TASK_PRI_SENSOR,  &hTaskSensor,  1);
    xTaskCreatePinnedToCore(taskAudio,   "audio",   STACK_AUDIO,   nullptr, TASK_PRI_AUDIO,   &hTaskAudio,   1);
    xTaskCreatePinnedToCore(taskGPS,     "gps",     STACK_GPS,     nullptr, TASK_PRI_GPS,     &hTaskGPS,     1);
    xTaskCreatePinnedToCore(taskDisplay, "display", STACK_DISPLAY, nullptr, TASK_PRI_DISPLAY, &hTaskDisplay, 1);
    xTaskCreatePinnedToCore(taskIGCLog,  "igc",     STACK_IGC,     nullptr, TASK_PRI_IGC,     &hTaskIGC,     1);
    xTaskCreatePinnedToCore(taskBattery, "batt",    STACK_BATTERY, nullptr, TASK_PRI_BATTERY, &hTaskBattery, 1);

    // Core 0: Protocol tasks
    xTaskCreatePinnedToCore(taskBLE, "ble", STACK_BLE, nullptr, TASK_PRI_BLE, &hTaskBLE, 0);

    Serial.println("[AltiVario] All tasks started");
}

void loop() {
    // Handle button events in Arduino loop (runs on Core 1)
    ButtonEvent evt = button.getEvent();
    if (evt != BTN_NONE) {
        if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(10))) {
            switch (evt) {
                case BTN_SHORT_PRESS:
                    flightData.displayPage = (flightData.displayPage + 1) % 3;
                    break;
                case BTN_LONG_PRESS:
                    // Toggle WiFi portal
                    if (!wifiPortal.isRunning()) {
                        wifiPortal.begin(&configMgr, &igcLogger);
                        display.showMessage("WiFi AP On");
                    } else {
                        wifiPortal.stop();
                        display.showMessage("WiFi Off");
                    }
                    break;
                case BTN_VERY_LONG_PRESS:
                    // Reset config
                    configMgr.resetDefaults();
                    display.showMessage("Config Reset");
                    break;
                default:
                    break;
            }
            xSemaphoreGive(flightDataMutex);
        }
        // Notify display task
        if (hTaskDisplay) xTaskNotifyGive(hTaskDisplay);
    }

    // WiFi portal processing
    if (wifiPortal.isRunning()) {
        wifiPortal.handle();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

// ─── Sensor Task (50 Hz) ─────────────────────────────────────────
void taskSensor(void* param) {
    TickType_t lastWake = xTaskGetTickCount();

    while (true) {
        if (sensor.update()) {
            int32_t press = sensor.getPressure();
            int32_t temp = sensor.getTemperature();
            unsigned long now = millis();

            float alt = pressureToAltitude((float)press, configMgr.config().qnh);
            varioComputer.update(alt, now);
            long avgPress = pressureFilter.update(press);

            // Update flight data
            if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(5))) {
                flightData.altitude = alt;
                flightData.vario = varioComputer.getVario();
                flightData.pressure = avgPress;
                flightData.temperature = temp;
                flightData.sensorValid = varioComputer.isValid();

                // Flight detection
                flightDetector.update(flightData.vario, flightData.gpsSpeed, now);
                flightData.flightState = flightDetector.state();

                xSemaphoreGive(flightDataMutex);
            }

            // Send tone to audio task
            if (varioComputer.isValid()) {
                ToneParams tp = audioEngine.compute(varioComputer.getVario());
                xQueueOverwrite(audioQueue, &tp);
            }

            // Notify IGC logger on flight state change
            if (flightDetector.stateChanged() && hTaskIGC) {
                xTaskNotifyGive(hTaskIGC);
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));  // 50 Hz
    }
}

// ─── Audio Task (event-driven) ────────────────────────────────────
void taskAudio(void* param) {
    ToneParams tp = {0, 0, 0};
    ToneParams currentTp = {0, 0, 0};
    unsigned long lastBeep = 0;

    while (true) {
        // Check for new tone parameters
        if (xQueueReceive(audioQueue, &tp, pdMS_TO_TICKS(10))) {
            currentTp = tp;
        }

        unsigned long now = millis();
        if (currentTp.frequency > 0 && currentTp.period > 0) {
            if ((now - lastBeep) >= currentTp.period) {
                audioDriver.play(currentTp);
                lastBeep = now;

                // Wait for tone duration, then stop
                vTaskDelay(pdMS_TO_TICKS(currentTp.duration));
                audioDriver.stop();
            }
        } else {
            audioDriver.stop();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── BLE Task (3 Hz) ─────────────────────────────────────────────
void taskBLE(void* param) {
    TickType_t lastWake = xTaskGetTickCount();
    char nmeaBuf[80];

    while (true) {
        if (ble.isConnected()) {
            NMEAVarioData data;
            if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(10))) {
                data.pressure_pa = flightData.pressure;
                data.altitude_m = flightData.altitude;
                data.vario_cms = flightData.vario * 100.0f;
                data.temperature_c = flightData.temperature / 100;
                data.battery_v = flightData.batteryV;
                data.battery_pct = flightData.batteryPct;
                xSemaphoreGive(flightDataMutex);
            }

            if (formatLK8EX1(nmeaBuf, sizeof(nmeaBuf), data)) {
                ble.send(nmeaBuf);
            }
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000 / BLE_UPDATE_HZ));
    }
}

// ─── GPS Task (continuous) ────────────────────────────────────────
void taskGPS(void* param) {
    while (true) {
        gps.update();

        if (gps.hasNewFix()) {
            if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(10))) {
                flightData.gpsLat = gps.latitude();
                flightData.gpsLon = gps.longitude();
                flightData.gpsAlt = gps.altitudeMSL();
                flightData.gpsSpeed = gps.speedKmh();
                flightData.gpsHeading = gps.heading();
                flightData.gpsSats = gps.satellites();
                flightData.gpsFix = gps.hasFix();

                // Compute glide ratio
                if (flightData.vario < -0.1f && flightData.gpsSpeed > 5.0f) {
                    float hSpeed = flightData.gpsSpeed / 3.6f;  // m/s
                    flightData.glideRatio = hSpeed / fabsf(flightData.vario);
                } else {
                    flightData.glideRatio = 0;
                }

                xSemaphoreGive(flightDataMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── Display Task (5 Hz) ─────────────────────────────────────────
void taskDisplay(void* param) {
    while (true) {
        // Wait for periodic update or button notification
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / DISPLAY_UPDATE_HZ));

        FlightData fd;
        if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(10))) {
            fd = flightData;
            xSemaphoreGive(flightDataMutex);
        }

        display.update(fd.displayPage, fd);
    }
}

// ─── IGC Logger Task (1 Hz, event-driven) ─────────────────────────
void taskIGCLog(void* param) {
    while (true) {
        // Wait for notification or periodic check
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000 / IGC_RECORD_HZ));

        FlightData fd;
        if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(10))) {
            fd = flightData;
            xSemaphoreGive(flightDataMutex);
        }

        // Start logging when flight begins
        if (fd.flightState == FLIGHT_IN_FLIGHT && !igcLogger.isRecording()) {
            if (fd.gpsFix) {
                igcLogger.startFlight(fd.gpsLat, fd.gpsLon, fd.gpsAlt);
            }
        }

        // Write B-record if recording
        if (igcLogger.isRecording() && fd.gpsFix) {
            igcLogger.writeRecord(fd.gpsLat, fd.gpsLon, fd.gpsAlt,
                                  fd.altitude, fd.pressure);
        }

        // Stop logging when landing
        if (fd.flightState == FLIGHT_GROUND && igcLogger.isRecording()) {
            igcLogger.stopFlight();
        }

        // Low battery protection
        if (fd.batteryV > 0 && fd.batteryV < BATT_STOP_LOG_V && igcLogger.isRecording()) {
            igcLogger.stopFlight();
        }
    }
}

// ─── Battery Task (0.1 Hz) ───────────────────────────────────────
void taskBattery(void* param) {
    while (true) {
        float v = battery.readVoltage();
        uint8_t pct = battery.percentage(BATT_FULL_V, BATT_EMPTY_V);

        if (xSemaphoreTake(flightDataMutex, pdMS_TO_TICKS(10))) {
            flightData.batteryV = v;
            flightData.batteryPct = pct;
            xSemaphoreGive(flightDataMutex);
        }

        // Deep sleep on critical battery
        if (v > 0.5f && v < BATT_SLEEP_V) {
            Serial.println("[BATT] Critical! Entering deep sleep.");
            if (igcLogger.isRecording()) {
                igcLogger.stopFlight();
            }
            esp_deep_sleep_start();
        }

        vTaskDelay(pdMS_TO_TICKS(10000));  // 0.1 Hz
    }
}
