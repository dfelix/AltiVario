#ifndef WIFI_PORTAL_H
#define WIFI_PORTAL_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

class ConfigManager;
class IGCLogger;

// WiFi captive portal for configuration and IGC file download
class WiFiPortal {
public:
    WiFiPortal();
    ~WiFiPortal();

    // Start AP and web server
    void begin(ConfigManager* config, IGCLogger* logger);

    // Stop AP and server
    void stop();

    // Must be called in loop for DNS handling
    void handle();

    bool isRunning() const { return _running; }

private:
    AsyncWebServer* _server;
    DNSServer* _dns;
    ConfigManager* _config;
    IGCLogger* _logger;
    bool _running;

    void setupRoutes();
    void handleGetConfig(AsyncWebServerRequest* request);
    void handleSetConfig(AsyncWebServerRequest* request);
    void handleListFiles(AsyncWebServerRequest* request);
    void handleDownloadFile(AsyncWebServerRequest* request);
};

#endif // WIFI_PORTAL_H
