#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include <Arduino.h>
#include <NimBLEDevice.h>

// BLE Serial (UART service) for XCTrack/XCSoar compatibility
// Uses Nordic UART Service (NUS) UUIDs
class BLETelemetry {
public:
    BLETelemetry();

    // Initialize BLE with device name
    void begin(const char* deviceName = "AltiVario");

    // Send NMEA sentence over BLE (max ~20 bytes per notification)
    void send(const char* data, uint8_t len);

    // Send null-terminated string
    void send(const char* str);

    bool isConnected() const;
    bool isEnabled() const { return _enabled; }
    void setEnabled(bool en) { _enabled = en; }

private:
    NimBLEServer* _server;
    NimBLECharacteristic* _txChar;
    bool _enabled;
    volatile bool _connected;

    // Nordic UART Service UUIDs
    static const char* SERVICE_UUID;
    static const char* TX_CHAR_UUID;
    static const char* RX_CHAR_UUID;

    class ServerCallbacks : public NimBLEServerCallbacks {
    public:
        ServerCallbacks(BLETelemetry* parent) : _parent(parent) {}
        void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override;
        void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override;
    private:
        BLETelemetry* _parent;
    };
    friend class ServerCallbacks;
};

#endif // BLE_TELEMETRY_H
