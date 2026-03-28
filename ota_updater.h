#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "led_status.h"

#define OTA_CHECK_INTERVAL_MS 3600000  // 1 hour
#define FIRMWARE_VERSION "1.0.0"

static String _otaDeviceId;
static String _otaBaseUrl;
static unsigned long _lastOTACheck = 0;
static bool _otaInProgress = false;

void initOTAUpdater(const String& deviceId, const String& serverBaseUrl) {
    _otaDeviceId = deviceId;
    // Use test server for firmware updates
    _otaBaseUrl = "https://test.linesights.com/api/firmware/";
    (void)serverBaseUrl;  // OTA uses dedicated endpoint
    Serial.printf("[OTA] Updater initialized, version: %s\n", FIRMWARE_VERSION);
}

String getCurrentVersion() { return FIRMWARE_VERSION; }
bool isUpdateInProgress() { return _otaInProgress; }

void checkForUpdate() {
    if (_otaInProgress) return;

    String checkUrl = _otaBaseUrl + "check?device_id=" + _otaDeviceId + "&current_version=" + FIRMWARE_VERSION;
    Serial.printf("[OTA] Checking for updates: %s\n", checkUrl.c_str());

    HTTPClient http;
    http.begin(checkUrl);
    http.setTimeout(10000);
    int httpCode = http.GET();

    if (httpCode != 200) {
        Serial.printf("[OTA] Check failed: HTTP %d\n", httpCode);
        http.end();
        return;
    }

    String payload = http.getString();
    http.end();

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        Serial.printf("[OTA] JSON parse error: %s\n", err.c_str());
        return;
    }

    bool updateAvailable = doc["update_available"] | false;
    if (!updateAvailable) {
        Serial.println("[OTA] Firmware is up to date");
        return;
    }

    String newVersion = doc["version"] | "unknown";
    String downloadUrl = doc["url"] | "";
    String releaseNotes = doc["release_notes"] | "";

    Serial.printf("[OTA] Update available: %s -> %s\n", FIRMWARE_VERSION, newVersion.c_str());
    if (releaseNotes.length() > 0) Serial.printf("[OTA] Notes: %s\n", releaseNotes.c_str());

    if (downloadUrl.length() == 0) {
        Serial.println("[OTA] No download URL provided");
        return;
    }

    // Start update
    _otaInProgress = true;
    setLEDState(LED_OTA_UPDATING);
    Serial.printf("[OTA] Downloading firmware from: %s\n", downloadUrl.c_str());

    HTTPClient dlHttp;
    dlHttp.begin(downloadUrl);
    dlHttp.setTimeout(30000);
    int dlCode = dlHttp.GET();

    if (dlCode != 200) {
        Serial.printf("[OTA] Download failed: HTTP %d\n", dlCode);
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    int contentLength = dlHttp.getSize();
    if (contentLength <= 0) {
        Serial.println("[OTA] Invalid content length");
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    Serial.printf("[OTA] Firmware size: %d bytes\n", contentLength);

    if (!Update.begin(contentLength)) {
        Serial.printf("[OTA] Not enough space: %s\n", Update.errorString());
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    WiFiClient* stream = dlHttp.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    Serial.printf("[OTA] Written: %d / %d bytes\n", written, contentLength);

    if (!Update.end()) {
        Serial.printf("[OTA] Update failed: %s\n", Update.errorString());
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    dlHttp.end();

    if (Update.isFinished()) {
        Serial.printf("[OTA] Update successful! Rebooting to %s...\n", newVersion.c_str());
        delay(1000);
        ESP.restart();
    } else {
        Serial.println("[OTA] Update not finished");
        _otaInProgress = false;
        setLEDState(LED_ERROR);
    }
}

void otaLoop() {
    if (_otaInProgress) return;

    unsigned long now = millis();
    if (now - _lastOTACheck >= OTA_CHECK_INTERVAL_MS) {
        _lastOTACheck = now;
        checkForUpdate();
    }
}
