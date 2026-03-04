#include "wifi_portal.h"
#include "config_manager.h"
#include "igc_logger.h"
#include "config.h"
#include <SD.h>

WiFiPortal::WiFiPortal()
    : _server(nullptr)
    , _dns(nullptr)
    , _config(nullptr)
    , _logger(nullptr)
    , _running(false)
{
}

WiFiPortal::~WiFiPortal() {
    stop();
}

void WiFiPortal::begin(ConfigManager* config, IGCLogger* logger) {
    if (_running) return;

    _config = config;
    _logger = logger;

    // Start AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);

    // DNS for captive portal
    _dns = new DNSServer();
    _dns->start(53, "*", WiFi.softAPIP());

    // Web server
    _server = new AsyncWebServer(WIFI_PORTAL_PORT);
    setupRoutes();
    _server->begin();

    _running = true;
    Serial.printf("[WiFi] AP started: %s IP: %s\n", WIFI_AP_SSID,
                  WiFi.softAPIP().toString().c_str());
}

void WiFiPortal::stop() {
    if (!_running) return;

    if (_server) {
        _server->end();
        delete _server;
        _server = nullptr;
    }
    if (_dns) {
        _dns->stop();
        delete _dns;
        _dns = nullptr;
    }

    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    _running = false;
}

void WiFiPortal::handle() {
    if (_dns) _dns->processNextRequest();
}

void WiFiPortal::setupRoutes() {
    // Serve static files from SPIFFS/LittleFS or embedded
    _server->on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
        request->send(200, "text/html",
            "<!DOCTYPE html><html><head>"
            "<meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>AltiVario</title>"
            "<style>"
            "body{font-family:sans-serif;max-width:600px;margin:0 auto;padding:16px;background:#1a1a2e;color:#eee}"
            "h1{color:#0ff}h2{color:#0af;border-bottom:1px solid #333;padding-bottom:8px}"
            "input,select{width:100%;padding:8px;margin:4px 0 12px;background:#16213e;color:#eee;border:1px solid #444;border-radius:4px}"
            "button{background:#0af;color:#000;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;margin:4px}"
            "button:hover{background:#0cf}"
            ".file{display:flex;justify-content:space-between;padding:8px;background:#16213e;margin:4px 0;border-radius:4px}"
            ".file a{color:#0ff;text-decoration:none}"
            "</style></head><body>"
            "<h1>AltiVario</h1>"
            "<h2>Configuration</h2>"
            "<div id='cfg'></div>"
            "<h2>Flight Logs</h2>"
            "<div id='files'>Loading...</div>"
            "<script>"
            "fetch('/api/config').then(r=>r.json()).then(c=>{"
            "let h='';"
            "for(let k in c)h+=`<label>${k}</label><input id='${k}' value='${c[k]}'>`;"
            "h+='<button onclick=\"save()\">Save</button>';"
            "document.getElementById('cfg').innerHTML=h;});"
            "function save(){"
            "let p=new URLSearchParams();document.querySelectorAll('#cfg input').forEach(i=>p.append(i.id,i.value));"
            "fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p.toString()}).then(()=>alert('Saved!'));}"
            "fetch('/api/files').then(r=>r.json()).then(f=>{"
            "let h=f.length?'':'No files found';"
            "f.forEach(n=>h+=`<div class='file'><a href='/api/download?f=${n}'>${n}</a></div>`);"
            "document.getElementById('files').innerHTML=h;});"
            "</script></body></html>"
        );
    });

    // API: Get config
    _server->on("/api/config", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleGetConfig(request);
    });

    // API: Set config
    _server->on("/api/config", HTTP_POST, [this](AsyncWebServerRequest* request) {
        handleSetConfig(request);
    });

    // API: List IGC files
    _server->on("/api/files", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleListFiles(request);
    });

    // API: Download IGC file
    _server->on("/api/download", HTTP_GET, [this](AsyncWebServerRequest* request) {
        handleDownloadFile(request);
    });

    // Captive portal redirect
    _server->onNotFound([](AsyncWebServerRequest* request) {
        request->redirect("/");
    });
}

void WiFiPortal::handleGetConfig(AsyncWebServerRequest* request) {
    if (!_config) { request->send(500); return; }

    const RuntimeConfig& cfg = _config->config();
    char json[256];
    snprintf(json, sizeof(json),
        "{\"climbThreshold\":%.2f,\"sinkThreshold\":%.2f,"
        "\"audioMode\":%d,\"volume\":%d,"
        "\"bleEnabled\":%d,\"gpsEnabled\":%d,\"sdEnabled\":%d,"
        "\"qnh\":%.0f}",
        cfg.climbThreshold, cfg.sinkThreshold,
        cfg.audioMode, cfg.volume,
        cfg.bleEnabled, cfg.gpsEnabled, cfg.sdEnabled,
        cfg.qnh);

    request->send(200, "application/json", json);
}

void WiFiPortal::handleSetConfig(AsyncWebServerRequest* request) {
    if (!_config) { request->send(500); return; }

    RuntimeConfig& cfg = _config->config();

    if (request->hasParam("climbThreshold", true))
        cfg.climbThreshold = request->getParam("climbThreshold", true)->value().toFloat();
    if (request->hasParam("sinkThreshold", true))
        cfg.sinkThreshold = request->getParam("sinkThreshold", true)->value().toFloat();
    if (request->hasParam("audioMode", true))
        cfg.audioMode = request->getParam("audioMode", true)->value().toInt();
    if (request->hasParam("volume", true))
        cfg.volume = request->getParam("volume", true)->value().toInt();
    if (request->hasParam("bleEnabled", true))
        cfg.bleEnabled = request->getParam("bleEnabled", true)->value().toInt();
    if (request->hasParam("gpsEnabled", true))
        cfg.gpsEnabled = request->getParam("gpsEnabled", true)->value().toInt();
    if (request->hasParam("sdEnabled", true))
        cfg.sdEnabled = request->getParam("sdEnabled", true)->value().toInt();
    if (request->hasParam("qnh", true))
        cfg.qnh = request->getParam("qnh", true)->value().toFloat();

    _config->save();
    request->send(200, "application/json", "{\"ok\":true}");
}

void WiFiPortal::handleListFiles(AsyncWebServerRequest* request) {
    if (!_logger) {
        request->send(200, "application/json", "[]");
        return;
    }

    char files[20][32];
    uint8_t count = _logger->listFiles(files, 20);

    String json = "[";
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) json += ",";
        json += "\"";
        json += files[i];
        json += "\"";
    }
    json += "]";

    request->send(200, "application/json", json);
}

void WiFiPortal::handleDownloadFile(AsyncWebServerRequest* request) {
    if (!request->hasParam("f")) {
        request->send(400, "text/plain", "Missing filename");
        return;
    }

    String name = request->getParam("f")->value();

    // Sanitize: reject path traversal attempts
    if (name.indexOf("..") >= 0 || name.indexOf("/") >= 0 || name.indexOf("\\") >= 0) {
        request->send(403, "text/plain", "Invalid filename");
        return;
    }

    // Only allow .igc files
    if (!name.endsWith(".igc")) {
        request->send(403, "text/plain", "Only IGC files can be downloaded");
        return;
    }

    String filename = "/" + name;
    if (!SD.exists(filename)) {
        request->send(404, "text/plain", "File not found");
        return;
    }

    request->send(SD, filename, "application/octet-stream");
}
