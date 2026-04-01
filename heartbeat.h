#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "config.h"
#include "wifi_manager.h"
#include "diagnostics.h"
#include "http_sender.h"
#include "ct_sensor.h"

// ============================================================================
// Heartbeat — Thread-safe error log, rate-limited commands, bounded storage
// ============================================================================

#define ERROR_LOG_FILE "/errors.json"

// Forward declarations
void calibrateCTZero();
void setHTTPDebug(bool on);
void forceOTACheck();

struct ErrorEntry {
    String timestamp;
    String message;
};

static portMUX_TYPE _errMux = portMUX_INITIALIZER_UNLOCKED;
static ErrorEntry _errorLog[MAX_ERROR_LOG];
static int _errorHead = 0;
static int _errorCount = 0;
static unsigned long _lastHeartbeat = 0;
static unsigned long _lastErrorSave = 0;
static String _heartbeatUrl;
static String _hbBootReason;
static bool _errorsDirty = false;
static unsigned long _lastCommandTime = 0;  // Rate limiting

int getRSSIMin();
int getRSSIAvg();

void loadErrorsFromFlash() {
    if (!LittleFS.exists(ERROR_LOG_FILE)) return;

    File f = LittleFS.open(ERROR_LOG_FILE, "r");
    if (!f) return;
    String content = f.readString();
    f.close();

    JsonDocument doc;
    if (deserializeJson(doc, content)) {
        LittleFS.remove(ERROR_LOG_FILE);
        return;
    }

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

void saveErrorsToFlash() {
    if (!_errorsDirty || _errorCount == 0) return;

    // Step 1: Copy error data under spinlock (only simple variable access)
    String snapTs[MAX_ERROR_LOG];
    String snapMsg[MAX_ERROR_LOG];
    int snapCount = 0;

    portENTER_CRITICAL(&_errMux);
    int start = (_errorCount >= MAX_ERROR_LOG) ? _errorHead : 0;
    snapCount = min(_errorCount, (int)MAX_ERROR_LOG);
    for (int i = 0; i < snapCount; i++) {
        int idx = (start + i) % MAX_ERROR_LOG;
        snapTs[i] = _errorLog[idx].timestamp;
        snapMsg[i] = _errorLog[idx].message;
    }
    portEXIT_CRITICAL(&_errMux);

    // Step 2: Build JSON OUTSIDE spinlock (allocations safe here)
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < snapCount; i++) {
        JsonObject e = arr.add<JsonObject>();
        e["t"] = snapTs[i];
        e["m"] = snapMsg[i];
    }

    // Step 3: Write file OUTSIDE spinlock (LittleFS needs interrupts)
    File f = LittleFS.open("/errors.tmp", "w");
    if (f) {
        serializeJson(doc, f);
        f.close();
        if (LittleFS.exists(ERROR_LOG_FILE)) LittleFS.remove(ERROR_LOG_FILE);
        LittleFS.rename("/errors.tmp", ERROR_LOG_FILE);
        _errorsDirty = false;
    }
}

// Thread-safe error logging — callable from any core
void logError(const String& message) {
    // Get timestamp OUTSIDE critical section (getLocalTime needs env locks)
    String ts = getUTCTimestamp();

    portENTER_CRITICAL(&_errMux);
    _errorLog[_errorHead].timestamp = ts;
    _errorLog[_errorHead].message = message;
    _errorHead = (_errorHead + 1) % MAX_ERROR_LOG;
    if (_errorCount < MAX_ERROR_LOG) _errorCount++;
    _errorsDirty = true;
    portEXIT_CRITICAL(&_errMux);

    Serial.printf("[ERR] %s\n", message.c_str());
}

void initHeartbeat(const String& serverUrl) {
    // Bypass Cloudflare — direct IP
    _heartbeatUrl = "http://46.224.90.187/api/firmware/heartbeat";

    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON:   _hbBootReason = "power_on"; break;
        case ESP_RST_SW:        _hbBootReason = "software"; break;
        case ESP_RST_PANIC:     _hbBootReason = "panic"; break;
        case ESP_RST_INT_WDT:   _hbBootReason = "int_watchdog"; break;
        case ESP_RST_TASK_WDT:  _hbBootReason = "task_watchdog"; break;
        case ESP_RST_WDT:       _hbBootReason = "watchdog"; break;
        case ESP_RST_BROWNOUT:  _hbBootReason = "brownout"; break;
        default:                _hbBootReason = "unknown"; break;
    }

    loadErrorsFromFlash();
    Serial.printf("[HB] -> %s\n", _heartbeatUrl.c_str());

    if (reason != ESP_RST_POWERON && reason != ESP_RST_SW) {
        logError("Unexpected boot: " + _hbBootReason);
    }
}

void processCommands(const String& responseBody) {
    if (responseBody.length() == 0) return;

    JsonDocument doc;
    if (deserializeJson(doc, responseBody)) return;

    JsonArray commands = doc["commands"];
    if (commands.isNull() || commands.size() == 0) return;

    // Rate limit: max 1 command batch per 30 seconds
    unsigned long now = millis();
    if (now - _lastCommandTime < 30000) {
        Serial.println("[CMD] Rate limited — ignoring commands");
        return;
    }
    _lastCommandTime = now;

    // Limit command array size to prevent DoS
    int cmdCount = min((int)commands.size(), 10);
    Serial.printf("[CMD] Processing %d command(s)\n", cmdCount);

    int processed = 0;
    for (JsonObject cmd : commands) {
        if (processed >= cmdCount) break;
        processed++;

        String action = cmd["action"] | "";
        if (action.length() == 0) continue;

        Serial.printf("[CMD] -> %s\n", action.c_str());

        if (action == "set_interval") {
            int val = cmd["value"] | -1;
            if (val >= 1 && val <= 60) {
                Preferences p; p.begin("lscfg", false);
                p.putInt("interval", val); p.end();
                Serial.printf("[CMD] Interval: %ds (reboot to apply)\n", val);
            }
        } else if (action == "set_voltage") {
            float val = cmd["value"] | -1.0f;
            if (val >= 100 && val <= 250) {
                Preferences p; p.begin("lscfg", false);
                p.putFloat("volt", val); p.end();
                Serial.printf("[CMD] Voltage: %.0fV (reboot to apply)\n", val);
            }
        } else if (action == "recalibrate") {
            calibrateCTZero();
        } else if (action == "reboot") {
            Serial.println("[CMD] Reboot requested");
            delay(500);
            ESP.restart();
        } else if (action == "debug_on") {
            setHTTPDebug(true);
        } else if (action == "debug_off") {
            setHTTPDebug(false);
        } else if (action == "update_firmware") {
            forceOTACheck();
        } else if (action == "factory_reset") {
            Serial.println("[CMD] Factory reset!");
            Preferences p; p.begin("lscfg", false);
            p.clear(); p.end();
            delay(500);
            ESP.restart();
        } else {
            Serial.printf("[CMD] Unknown: %s\n", action.c_str());
        }
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
    doc["min_heap"] = ESP.getMinFreeHeap();
    doc["uptime_seconds"] = getUptimeSeconds();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["boot_reason"] = _hbBootReason;
    doc["queue_depth"] = getQueueSize();
    doc["total_sent"] = getTotalSent();
    doc["total_failed"] = getTotalFailed();
    doc["total_dropped"] = getTotalDropped();
    doc["ct_calibrated"] = isCTCalibrated();

    // Errors — copy under lock, build JSON outside
    String errTimestamps[MAX_ERROR_LOG];
    String errMessages[MAX_ERROR_LOG];
    int errCopyCount = 0;

    portENTER_CRITICAL(&_errMux);
    if (_errorCount > 0) {
        int start = (_errorCount >= MAX_ERROR_LOG) ? _errorHead : 0;
        errCopyCount = min(_errorCount, (int)MAX_ERROR_LOG);
        for (int i = 0; i < errCopyCount; i++) {
            int idx = (start + i) % MAX_ERROR_LOG;
            errTimestamps[i] = _errorLog[idx].timestamp;
            errMessages[i] = _errorLog[idx].message;
        }
    }
    portEXIT_CRITICAL(&_errMux);

    JsonArray errors = doc["errors"].to<JsonArray>();
    for (int i = 0; i < errCopyCount; i++) {
        JsonObject e = errors.add<JsonObject>();
        e["timestamp"] = errTimestamps[i];
        e["message"] = errMessages[i];
    }

    String json;
    serializeJson(doc, json);

    HTTPClient http;
    http.begin(_heartbeatUrl);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(HEARTBEAT_TIMEOUT_MS);
    int httpCode = http.POST(json);

    if (httpCode == 200) {
        String response = http.getString();
        Serial.printf("[HB] OK | heap:%u | Q:%d | S:%u\n",
            ESP.getFreeHeap(), getQueueSize(), getTotalSent());
        processCommands(response);
    } else if (httpCode > 0) {
        Serial.printf("[HB] HTTP %d\n", httpCode);
    } else {
        logError("HB fail: " + http.errorToString(httpCode));
    }

    http.end();  // Always cleanup
}

void heartbeatLoop() {
    unsigned long now = millis();
    if (now - _lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        _lastHeartbeat = now;
        sendHeartbeat();
    }

    if (now - _lastErrorSave >= ERROR_SAVE_INTERVAL_MS) {
        _lastErrorSave = now;
        saveErrorsToFlash();
    }
}

int getErrorCount() { return _errorCount; }
