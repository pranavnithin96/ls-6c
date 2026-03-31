#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>
#include "led_status.h"

#define OTA_CHECK_INTERVAL_MS 3600000  // 1 hour
#define MAX_CRASH_COUNT 3
// FIRMWARE_VERSION defined in Line_Sight.ino

static String _otaDeviceId;
static String _otaBaseUrl;
static unsigned long _lastOTACheck = 0;
static bool _otaInProgress = false;
static bool _otaForceCheck = false;

void forceOTACheck() { _otaForceCheck = true; }

// Forward declarations
void logError(const String& message);
void disconnectHTTP();

// Force URL to HTTP — OTA can't use HTTPS (not enough RAM for SSL)
static String forceHTTP(const String& url) {
    String out = url;
    out.replace("https://", "http://");
    return out;
}

// Rollback check — called early in setup before anything else
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
            Serial.println("[OTA] Too many crashes — rolling back!");
            otaPrefs.putBool("updated", false);
            otaPrefs.putInt("crashes", 0);
            otaPrefs.end();

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
    } else if (!wasCrash && justUpdated) {
        Serial.println("[OTA] Firmware stable — marking as good");
        otaPrefs.putBool("updated", false);
        otaPrefs.putInt("crashes", 0);
        esp_ota_mark_app_valid_cancel_rollback();
    }

    otaPrefs.end();
}

void initOTAUpdater(const String& deviceId, const String& serverBaseUrl) {
    _otaDeviceId = deviceId;
    // OTA uses HTTP only — server configured to serve /api/firmware/* on HTTP
    String baseUrl = forceHTTP(serverBaseUrl);
    int apiIdx = baseUrl.indexOf("/api/");
    if (apiIdx >= 0) {
        _otaBaseUrl = baseUrl.substring(0, apiIdx) + "/api/firmware/";
    } else {
        _otaBaseUrl = "http://linesights.com/api/firmware/";
    }
    Serial.printf("[OTA] URL: %s\n", _otaBaseUrl.c_str());
    Serial.printf("[OTA] Version: %s\n", FIRMWARE_VERSION);
}

String getCurrentVersion() { return FIRMWARE_VERSION; }
bool isUpdateInProgress() { return _otaInProgress; }

void checkForUpdate() {
    if (_otaInProgress) return;

    // Free persistent HTTPS connection to reclaim ~40KB RAM
    disconnectHTTP();
    Serial.printf("[OTA] Free heap: %u bytes\n", ESP.getFreeHeap());

    // Check for update via HTTP
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

    String payload = http.getString();
    http.end();

    Serial.printf("[OTA] Response: %d bytes\n", payload.length());
    if (payload.length() > 0 && payload.length() < 500) {
        Serial.printf("[OTA] Body: %s\n", payload.c_str());
    }

    if (payload.length() == 0) {
        Serial.println("[OTA] Empty response");
        return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        Serial.printf("[OTA] JSON error: %s\n", err.c_str());
        return;
    }

    if (!(doc["update_available"] | false)) {
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

    // Force download URL to HTTP too
    downloadUrl = forceHTTP(downloadUrl);

    _otaInProgress = true;
    setLEDState(LED_OTA_UPDATING);
    Serial.printf("[OTA] Heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("[OTA] Downloading: %s\n", downloadUrl.c_str());

    HTTPClient dlHttp;
    dlHttp.begin(downloadUrl);
    dlHttp.setTimeout(60000);
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

    Serial.printf("[OTA] Size: %d bytes, heap: %u\n", contentLength, ESP.getFreeHeap());

    if (!Update.begin(contentLength)) {
        Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());
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
