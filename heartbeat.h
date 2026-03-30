#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "wifi_manager.h"
#include "diagnostics.h"
#include "http_sender.h"
#include "ct_sensor.h"

#define HEARTBEAT_INTERVAL_MS 60000   // 1 minute
#define HEARTBEAT_TIMEOUT_MS 5000
#define MAX_ERROR_LOG 50              // Ring buffer of last 50 errors
#define ERROR_LOG_FILE "/errors.json"
#define ERROR_SAVE_INTERVAL_MS 300000 // Save errors to SPIFFS every 5 min

struct ErrorEntry {
    String timestamp;
    String message;
};

static ErrorEntry _errorLog[MAX_ERROR_LOG];
static int _errorHead = 0;
static int _errorCount = 0;
static unsigned long _lastHeartbeat = 0;
static unsigned long _lastErrorSave = 0;
static String _heartbeatUrl;
static String _bootReasonStr;
static bool _errorsDirty = false;

// Forward declaration
int getRSSIMin();
int getRSSIAvg();

// Load errors from SPIFFS on boot
void loadErrorsFromFlash() {
    if (!SPIFFS.exists(ERROR_LOG_FILE)) return;

    File f = SPIFFS.open(ERROR_LOG_FILE, "r");
    if (!f) return;

    String content = f.readString();
    f.close();

    JsonDocument doc;
    if (deserializeJson(doc, content)) return;

    JsonArray arr = doc.as<JsonArray>();
    for (JsonVariant v : arr) {
        if (_errorCount >= MAX_ERROR_LOG) break;
        _errorLog[_errorHead].timestamp = v["t"].as<String>();
        _errorLog[_errorHead].message = v["m"].as<String>();
        _errorHead = (_errorHead + 1) % MAX_ERROR_LOG;
        _errorCount++;
    }
    Serial.printf("[ERR] Loaded %d errors from flash\n", _errorCount);
}

// Save errors to SPIFFS
void saveErrorsToFlash() {
    if (!_errorsDirty || _errorCount == 0) return;

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    int start = (_errorCount >= MAX_ERROR_LOG) ? _errorHead : 0;
    int count = min(_errorCount, MAX_ERROR_LOG);
    for (int i = 0; i < count; i++) {
        int idx = (start + i) % MAX_ERROR_LOG;
        JsonObject e = arr.add<JsonObject>();
        e["t"] = _errorLog[idx].timestamp;
        e["m"] = _errorLog[idx].message;
    }

    File f = SPIFFS.open(ERROR_LOG_FILE, "w");
    if (f) {
        serializeJson(doc, f);
        f.close();
        _errorsDirty = false;
    }
}

// Call this from anywhere to log an error
void logError(const String& message) {
    _errorLog[_errorHead].timestamp = getUTCTimestamp();
    _errorLog[_errorHead].message = message;
    _errorHead = (_errorHead + 1) % MAX_ERROR_LOG;
    if (_errorCount < MAX_ERROR_LOG) _errorCount++;
    _errorsDirty = true;

    Serial.printf("[ERR] %s\n", message.c_str());
}

void initHeartbeat(const String& serverUrl) {
    // Derive heartbeat URL from data URL
    // e.g., "http://77.42.75.92/api/data" -> "http://77.42.75.92/api/firmware/heartbeat"
    int apiIdx = serverUrl.indexOf("/api/");
    if (apiIdx >= 0) {
        _heartbeatUrl = serverUrl.substring(0, apiIdx) + "/api/firmware/heartbeat";
    } else {
        _heartbeatUrl = serverUrl + "/firmware/heartbeat";
    }

    // Capture boot reason
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON:   _bootReasonStr = "power_on"; break;
        case ESP_RST_SW:        _bootReasonStr = "software"; break;
        case ESP_RST_PANIC:     _bootReasonStr = "panic"; break;
        case ESP_RST_INT_WDT:   _bootReasonStr = "int_watchdog"; break;
        case ESP_RST_TASK_WDT:  _bootReasonStr = "task_watchdog"; break;
        case ESP_RST_WDT:       _bootReasonStr = "watchdog"; break;
        case ESP_RST_DEEPSLEEP: _bootReasonStr = "deep_sleep"; break;
        default:                _bootReasonStr = "unknown"; break;
    }

    // Load persisted errors from SPIFFS
    loadErrorsFromFlash();

    Serial.printf("[HB] Heartbeat initialized -> %s\n", _heartbeatUrl.c_str());
    if (reason != ESP_RST_POWERON && reason != ESP_RST_SW) {
        logError("Unexpected boot: " + _bootReasonStr);
    }
}

void sendHeartbeat() {
    if (_heartbeatUrl.length() == 0) return;

    JsonDocument doc;
    doc["device_id"] = getDeviceId();
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["wifi_rssi"] = WiFi.RSSI();
    doc["wifi_rssi_min"] = getRSSIMin();
    doc["wifi_rssi_avg"] = getRSSIAvg();
    doc["wifi_ssid"] = WiFi.SSID();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["uptime_seconds"] = getUptimeSeconds();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["boot_reason"] = _bootReasonStr;

    // Telemetry stats
    doc["queue_depth"] = getQueueSize();
    doc["total_sent"] = getTotalSent();
    doc["total_failed"] = getTotalFailed();
    doc["total_dropped"] = getTotalDropped();
    doc["ct_calibrated"] = isCTCalibrated();

    // Error log (last N errors, oldest first)
    JsonArray errors = doc["errors"].to<JsonArray>();
    if (_errorCount > 0) {
        int start;
        int count;
        if (_errorCount >= MAX_ERROR_LOG) {
            start = _errorHead;  // oldest is at head (it wraps)
            count = MAX_ERROR_LOG;
        } else {
            start = 0;
            count = _errorCount;
        }
        for (int i = 0; i < count; i++) {
            int idx = (start + i) % MAX_ERROR_LOG;
            JsonObject e = errors.add<JsonObject>();
            e["timestamp"] = _errorLog[idx].timestamp;
            e["message"] = _errorLog[idx].message;
        }
    }

    String json;
    serializeJson(doc, json);

    HTTPClient http;
    http.begin(_heartbeatUrl);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(HEARTBEAT_TIMEOUT_MS);

    int httpCode = http.POST(json);

    if (httpCode == 200) {
        Serial.printf("[HB] OK | heap:%u | RSSI:%d | Q:%d | S:%d | E:%d\n",
            ESP.getFreeHeap(), WiFi.RSSI(), getQueueSize(), getTotalSent(), _errorCount);
    } else if (httpCode > 0) {
        // Don't log as error — endpoint might not exist yet
        Serial.printf("[HB] HTTP %d (endpoint may not exist yet)\n", httpCode);
    } else {
        logError("Heartbeat failed: " + http.errorToString(httpCode));
    }

    http.end();
}

void heartbeatLoop() {
    unsigned long now = millis();
    if (now - _lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        _lastHeartbeat = now;
        sendHeartbeat();
    }

    // Periodic error save to SPIFFS
    if (now - _lastErrorSave >= ERROR_SAVE_INTERVAL_MS) {
        _lastErrorSave = now;
        saveErrorsToFlash();
    }
}

int getErrorCount() { return _errorCount; }
