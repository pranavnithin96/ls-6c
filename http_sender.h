#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "ct_sensor.h"

#define MAX_BUFFER_SIZE 120
#define HTTP_TIMEOUT_MS 5000
#define MAX_CONSECUTIVE_FAILURES 10
#define MAX_BACKOFF_MS 30000

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

void setHTTPDebug(bool on) { _httpDebug = on; }
bool getHTTPDebug() { return _httpDebug; }

void initHTTPSender(const String& serverUrl) {
    _httpServerUrl = serverUrl;
    for (int i = 0; i < MAX_BUFFER_SIZE; i++) _sendBuffer[i].used = false;
    _bufferCount = 0;
    _bufferHead = 0;
    _bufferTail = 0;
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

        ct["amps"] = serialized(String(readings[i].amps, 3));
        ct["watts"] = serialized(String(readings[i].watts, 1));
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
        _sendBuffer[_bufferTail].json = String();  // free memory
        _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
        _bufferCount--;
    } else {
        _totalFailed++;
        _consecutiveFailures++;
        _backoffMs = min(_backoffMs * 2, MAX_BACKOFF_MS);

        if (httpCode > 0) {
            Serial.printf("[HTTP] Error %d: %s\n", httpCode, http.getString().c_str());
        } else {
            Serial.printf("[HTTP] Error: %s\n", http.errorToString(httpCode).c_str());
        }

        if (_consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
            _totalDropped++;
            _sendBuffer[_bufferTail].used = false;
            _sendBuffer[_bufferTail].json = String();
            _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
            _bufferCount--;
            _consecutiveFailures = 0;
            Serial.println("[HTTP] Dropped after max retries");
        }
    }

    http.end();
}

int getQueueSize() { return _bufferCount; }
int getTotalSent() { return _totalSent; }
int getTotalFailed() { return _totalFailed; }
int getTotalDropped() { return _totalDropped; }
