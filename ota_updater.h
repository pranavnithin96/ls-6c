#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>
#include "led_status.h"

#define OTA_CHECK_INTERVAL_MS 3600000  // 1 hour
#define MAX_CRASH_COUNT 3              // Rollback after this many consecutive crashes
// FIRMWARE_VERSION is defined in Line_Sight.ino / main.cpp

static String _otaDeviceId;
static String _otaBaseUrl;
static unsigned long _lastOTACheck = 0;
static bool _otaInProgress = false;
static bool _otaForceCheck = false;

void forceOTACheck() { _otaForceCheck = true; }

// Forward declaration
void logError(const String& message);

// Check if we should rollback (called early in setup)
void checkFirmwareRollback() {
    Preferences otaPrefs;
    otaPrefs.begin("otastate", false);

    int crashCount = otaPrefs.getInt("crashes", 0);
    bool justUpdated = otaPrefs.getBool("updated", false);

    esp_reset_reason_t reason = esp_reset_reason();
    bool wasCrash = (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
                     reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT);

    if (wasCrash && justUpdated) {
        crashCount++;
        otaPrefs.putInt("crashes", crashCount);
        Serial.printf("[OTA] Post-update crash #%d/%d\n", crashCount, MAX_CRASH_COUNT);

        if (crashCount >= MAX_CRASH_COUNT) {
            Serial.println("[OTA] Too many crashes — rolling back to previous firmware!");
            otaPrefs.putBool("updated", false);
            otaPrefs.putInt("crashes", 0);
            otaPrefs.end();

            // Rollback to previous OTA partition
            const esp_partition_t* prev = esp_ota_get_last_invalid_partition();
            if (prev != NULL) {
                esp_ota_set_boot_partition(prev);
                Serial.println("[OTA] Rollback set. Rebooting...");
                delay(1000);
                ESP.restart();
            } else {
                Serial.println("[OTA] No previous partition to rollback to");
            }
            return;
        }
    } else if (!wasCrash) {
        // Stable boot — clear crash counter and mark firmware as good
        if (justUpdated) {
            Serial.println("[OTA] Firmware stable after update — marking as good");
            otaPrefs.putBool("updated", false);
            otaPrefs.putInt("crashes", 0);

            // Mark current partition as valid (prevents automatic rollback)
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }

    otaPrefs.end();
}

void initOTAUpdater(const String& deviceId, const String& serverBaseUrl) {
    _otaDeviceId = deviceId;
    String baseUrl = serverBaseUrl;
    int apiIdx = baseUrl.indexOf("/api/");
    if (apiIdx >= 0) {
        _otaBaseUrl = baseUrl.substring(0, apiIdx) + "/api/firmware/";
    } else {
        _otaBaseUrl = "http://77.42.75.92/api/firmware/";
    }
    Serial.printf("[OTA] Updater initialized, version: %s\n", FIRMWARE_VERSION);
}

String getCurrentVersion() { return FIRMWARE_VERSION; }
bool isUpdateInProgress() { return _otaInProgress; }

void checkForUpdate() {
    if (_otaInProgress) return;

    String checkUrl = _otaBaseUrl + "check?device_id=" + _otaDeviceId + "&current_version=" + FIRMWARE_VERSION;
    Serial.printf("[OTA] Checking: %s\n", checkUrl.c_str());

    HTTPClient http;
    http.begin(checkUrl);
    http.setTimeout(10000);
    int httpCode = http.GET();

    if (httpCode != 200) {
        Serial.printf("[OTA] Check failed: HTTP %d\n", httpCode);
        http.end();
        return;
    }

    // Read response — handle both chunked and content-length responses
    String payload = "";
    WiFiClient* checkStream = http.getStreamPtr();
    if (checkStream && checkStream->available()) {
        payload = checkStream->readString();
    }
    if (payload.length() == 0) {
        payload = http.getString();
    }
    http.end();

    Serial.printf("[OTA] Response: HTTP %d, body=%d bytes\n", httpCode, payload.length());
    if (payload.length() > 0 && payload.length() < 500) {
        Serial.printf("[OTA] Body: %s\n", payload.c_str());
    }

    if (payload.length() == 0) {
        Serial.println("[OTA] Empty response — check server");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        Serial.printf("[OTA] JSON error: %s\n", err.c_str());
        return;
    }

    bool updateAvailable = doc["update_available"] | false;
    if (!updateAvailable) {
        Serial.println("[OTA] Up to date");
        return;
    }

    String newVersion = doc["version"] | "unknown";
    String downloadUrl = doc["url"] | "";
    String releaseNotes = doc["release_notes"] | "";

    Serial.printf("[OTA] Update: %s -> %s\n", FIRMWARE_VERSION, newVersion.c_str());
    if (releaseNotes.length() > 0) Serial.printf("[OTA] Notes: %s\n", releaseNotes.c_str());

    if (downloadUrl.length() == 0) {
        Serial.println("[OTA] No download URL");
        return;
    }

    _otaInProgress = true;
    setLEDState(LED_OTA_UPDATING);
    Serial.printf("[OTA] Downloading: %s\n", downloadUrl.c_str());

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

    Serial.printf("[OTA] Size: %d bytes\n", contentLength);

    if (!Update.begin(contentLength)) {
        Serial.printf("[OTA] No space: %s\n", Update.errorString());
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    WiFiClient* stream = dlHttp.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    Serial.printf("[OTA] Written: %d / %d bytes\n", written, contentLength);

    if (!Update.end()) {
        Serial.printf("[OTA] Failed: %s\n", Update.errorString());
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    dlHttp.end();

    if (Update.isFinished()) {
        // Mark that we just updated — crash counter will track stability
        Preferences otaPrefs;
        otaPrefs.begin("otastate", false);
        otaPrefs.putBool("updated", true);
        otaPrefs.putInt("crashes", 0);
        otaPrefs.end();

        Serial.printf("[OTA] Success! Rebooting to %s...\n", newVersion.c_str());
        delay(1000);
        ESP.restart();
    } else {
        Serial.println("[OTA] Not finished");
        _otaInProgress = false;
        setLEDState(LED_ERROR);
    }
}

void otaLoop() {
    if (_otaInProgress) return;

    unsigned long now = millis();
    if (_otaForceCheck || (now - _lastOTACheck >= OTA_CHECK_INTERVAL_MS)) {
        _lastOTACheck = now;
        _otaForceCheck = false;
        checkForUpdate();
    }
}
