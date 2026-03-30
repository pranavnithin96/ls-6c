#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "ct_sensor.h"

// Forward declaration — defined in heartbeat.h
void logError(const String& message);

#define MAX_BUFFER_SIZE 300
#define HTTP_TIMEOUT_MS 5000
#define MAX_CONSECUTIVE_FAILURES 10
#define MAX_BACKOFF_MS 30000
#define BUFFER_SAVE_INTERVAL_MS 300000  // Save to flash every 5 minutes
#define BUFFER_FILE "/buffer.json"

struct BufferedReading {
    String json;
    bool used;
};

static BufferedReading _sendBuffer[MAX_BUFFER_SIZE];
static int _bufferHead = 0;
static int _bufferTail = 0;
static int _bufferCount = 0;
static String _httpServerUrl;
static int _totalSent = 0;
static int _totalFailed = 0;
static int _totalDropped = 0;
static int _consecutiveFailures = 0;
static unsigned long _lastSendAttempt = 0;
static int _backoffMs = 1000;
static bool _httpDebug = false;
static unsigned long _lastBufferSave = 0;
static bool _spiffsReady = false;

void setHTTPDebug(bool on) { _httpDebug = on; }
bool getHTTPDebug() { return _httpDebug; }

// Load buffered readings from SPIFFS (survives power loss)
void loadBufferFromFlash() {
    if (!_spiffsReady) return;
    if (!SPIFFS.exists(BUFFER_FILE)) return;

    File f = SPIFFS.open(BUFFER_FILE, "r");
    if (!f) return;

    String content = f.readString();
    f.close();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, content);
    if (err) {
        Serial.printf("[BUF] Failed to parse buffer file: %s\n", err.c_str());
        SPIFFS.remove(BUFFER_FILE);
        return;
    }

    JsonArray arr = doc.as<JsonArray>();
    int loaded = 0;
    for (JsonVariant v : arr) {
        if (_bufferCount >= MAX_BUFFER_SIZE) break;
        String json;
        serializeJson(v, json);
        _sendBuffer[_bufferHead].json = json;
        _sendBuffer[_bufferHead].used = true;
        _bufferHead = (_bufferHead + 1) % MAX_BUFFER_SIZE;
        _bufferCount++;
        loaded++;
    }

    SPIFFS.remove(BUFFER_FILE);
    if (loaded > 0) {
        Serial.printf("[BUF] Loaded %d readings from flash\n", loaded);
    }
}

// Save current buffer to SPIFFS
void saveBufferToFlash() {
    if (!_spiffsReady || _bufferCount == 0) return;

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    int idx = _bufferTail;
    for (int i = 0; i < _bufferCount; i++) {
        if (_sendBuffer[idx].used) {
            JsonDocument item;
            deserializeJson(item, _sendBuffer[idx].json);
            arr.add(item);
        }
        idx = (idx + 1) % MAX_BUFFER_SIZE;
    }

    File f = SPIFFS.open(BUFFER_FILE, "w");
    if (!f) {
        Serial.println("[BUF] Failed to open buffer file for writing");
        return;
    }

    serializeJson(doc, f);
    f.close();
    Serial.printf("[BUF] Saved %d readings to flash\n", _bufferCount);
}

void initHTTPSender(const String& serverUrl) {
    _httpServerUrl = serverUrl;
    for (int i = 0; i < MAX_BUFFER_SIZE; i++) _sendBuffer[i].used = false;
    _bufferCount = 0;
    _bufferHead = 0;
    _bufferTail = 0;

    // Init SPIFFS for persistent buffer
    if (SPIFFS.begin(true)) {
        _spiffsReady = true;
        loadBufferFromFlash();
    } else {
        Serial.println("[BUF] SPIFFS init failed — no persistence");
    }

    Serial.printf("[HTTP] Sender initialized, server: %s\n", serverUrl.c_str());
}

void queueReading(const String& deviceId, const String& location, const String& timezone,
                  float gridVoltage, CTReading readings[NUM_CT_CHANNELS], const String& timestamp) {

    JsonDocument doc;
    doc["device_id"] = deviceId;
    doc["timestamp"] = timestamp;
    doc["location"] = location;
    doc["timezone"] = timezone;

    JsonObject cts = doc["readings"]["cts"].to<JsonObject>();
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        char key[8];
        snprintf(key, sizeof(key), "ct_%d", i + 1);
        JsonObject ct = cts[key].to<JsonObject>();

        ct["real_power_w"] = serialized(String(readings[i].watts, 1));
        ct["amps"] = serialized(String(readings[i].amps, 3));
        ct["pf"] = serialized(String(readings[i].pf, 3));
    }
    doc["readings"]["voltage_rms"] = serialized(String(gridVoltage, 1));

    String json;
    serializeJson(doc, json);

    if (_bufferCount >= MAX_BUFFER_SIZE) {
        _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
        _bufferCount--;
        _totalDropped++;
    }

    _sendBuffer[_bufferHead].json = json;
    _sendBuffer[_bufferHead].used = true;
    _bufferHead = (_bufferHead + 1) % MAX_BUFFER_SIZE;
    _bufferCount++;
}

void processSendQueue() {
    if (_bufferCount == 0) return;

    unsigned long now = millis();
    if (_consecutiveFailures > 0 && (now - _lastSendAttempt) < (unsigned long)_backoffMs) return;

    if (!_sendBuffer[_bufferTail].used) return;

    String& json = _sendBuffer[_bufferTail].json;

    if (_httpDebug) {
        Serial.printf("[HTTP] POST %s\n", _httpServerUrl.c_str());
        Serial.printf("[HTTP] Body: %s\n", json.c_str());
    }

    HTTPClient http;
    http.begin(_httpServerUrl);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(HTTP_TIMEOUT_MS);

    int httpCode = http.POST(json);
    _lastSendAttempt = now;

    if (httpCode == 200) {
        _totalSent++;
        _consecutiveFailures = 0;
        _backoffMs = 1000;
        _sendBuffer[_bufferTail].used = false;
        _sendBuffer[_bufferTail].json = String();
        _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
        _bufferCount--;
    } else {
        _totalFailed++;
        _consecutiveFailures++;
        _backoffMs = min(_backoffMs * 2, MAX_BACKOFF_MS);

        if (httpCode > 0) {
            String errMsg = "HTTP " + String(httpCode) + ": " + http.getString();
            Serial.printf("[HTTP] %s\n", errMsg.c_str());
            if (_consecutiveFailures == 1) logError(errMsg);  // log first failure
        } else {
            String errMsg = "HTTP error: " + http.errorToString(httpCode);
            Serial.printf("[HTTP] %s\n", errMsg.c_str());
            if (_consecutiveFailures == 1) logError(errMsg);
        }

        if (_consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
            _totalDropped++;
            _sendBuffer[_bufferTail].used = false;
            _sendBuffer[_bufferTail].json = String();
            _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
            _bufferCount--;
            _consecutiveFailures = 0;
            logError("Dropped reading after " + String(MAX_CONSECUTIVE_FAILURES) + " retries");
        }
    }

    http.end();

    // Periodic buffer save to flash (every 5 minutes)
    if (_spiffsReady && (now - _lastBufferSave >= BUFFER_SAVE_INTERVAL_MS)) {
        _lastBufferSave = now;
        if (_bufferCount > 0) {
            saveBufferToFlash();
        } else if (SPIFFS.exists(BUFFER_FILE)) {
            SPIFFS.remove(BUFFER_FILE);
        }
    }
}

int getQueueSize() { return _bufferCount; }
int getTotalSent() { return _totalSent; }
int getTotalFailed() { return _totalFailed; }
int getTotalDropped() { return _totalDropped; }
