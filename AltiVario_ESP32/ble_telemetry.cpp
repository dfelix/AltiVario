#include "ble_telemetry.h"

// Nordic UART Service UUIDs (standard for BLE serial)
const char* BLETelemetry::SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char* BLETelemetry::TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const char* BLETelemetry::RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

BLETelemetry::BLETelemetry()
    : _server(nullptr)
    , _txChar(nullptr)
    , _enabled(true)
    , _connected(false)
{
}

void BLETelemetry::ServerCallbacks::onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) {
    _parent->_connected = true;
    // Allow multiple connections
    NimBLEDevice::startAdvertising();
}

void BLETelemetry::ServerCallbacks::onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) {
    _parent->_connected = false;
    NimBLEDevice::startAdvertising();
}

void BLETelemetry::begin(const char* deviceName) {
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    _server = NimBLEDevice::createServer();
    _server->setCallbacks(new ServerCallbacks(this));

    NimBLEService* service = _server->createService(SERVICE_UUID);

    _txChar = service->createCharacteristic(
        TX_CHAR_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // RX characteristic (for receiving commands from phone)
    service->createCharacteristic(
        RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );

    service->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();
}

void BLETelemetry::send(const char* data, uint8_t len) {
    if (!_enabled || !_connected || !_txChar || len == 0) return;

    // BLE MTU is typically 20 bytes for notifications, send in chunks
    const uint8_t MTU = 20;
    uint8_t offset = 0;
    while (offset < len) {
        uint8_t chunk = min((uint8_t)(len - offset), MTU);
        _txChar->setValue((const uint8_t*)(data + offset), chunk);
        _txChar->notify();
        offset += chunk;
    }
}

void BLETelemetry::send(const char* str) {
    send(str, strlen(str));
}

bool BLETelemetry::isConnected() const {
    return _connected;
}
