#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>
#include "config.h"
#include "led_status.h"

// ============================================================================
// OTA Updater — Partition validation, version comparison, safe rollback
// ============================================================================

static String _otaDeviceId;
static String _otaBaseUrl;
static unsigned long _lastOTACheck = 0;
static bool _otaInProgress = false;
static bool _otaForceCheck = false;

void forceOTACheck() { _otaForceCheck = true; }

void logError(const String& message);
void disconnectHTTP();
void feedWatchdog();

static String forceHTTP(const String& url) {
    String out = url;
    out.replace("https://", "http://");
    return out;
}

// Semantic version comparison: returns true if newVer > oldVer
static bool isVersionGreater(const String& newVer, const String& oldVer) {
    int nMaj = 0, nMin = 0, nPat = 0;
    int oMaj = 0, oMin = 0, oPat = 0;
    sscanf(newVer.c_str(), "%d.%d.%d", &nMaj, &nMin, &nPat);
    sscanf(oldVer.c_str(), "%d.%d.%d", &oMaj, &oMin, &oPat);
    if (nMaj != oMaj) return nMaj > oMaj;
    if (nMin != oMin) return nMin > oMin;
    return nPat > oPat;
}

// Rollback check — called FIRST in setup()
void checkFirmwareRollback() {
    Preferences otaPrefs;
    otaPrefs.begin("otastate", false);

    int crashCount = otaPrefs.getInt("crashes", 0);
    bool justUpdated = otaPrefs.getBool("updated", false);

    esp_reset_reason_t reason = esp_reset_reason();
    // Broader crash detection: anything that isn't a clean boot
    bool wasCrash = (reason != ESP_RST_POWERON && reason != ESP_RST_SW &&
                     reason != ESP_RST_DEEPSLEEP);

    if (wasCrash && justUpdated) {
        crashCount++;
        otaPrefs.putInt("crashes", crashCount);
        Serial.printf("[OTA] Post-update crash #%d/%d (reason: %d)\n", crashCount, MAX_CRASH_COUNT, (int)reason);

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
                Serial.println("[OTA] No previous partition for rollback");
            }
            return;
        }
    } else if (!wasCrash && justUpdated) {
        Serial.println("[OTA] Firmware stable — marking valid");
        otaPrefs.putBool("updated", false);
        otaPrefs.putInt("crashes", 0);
        esp_ota_mark_app_valid_cancel_rollback();
    }

    otaPrefs.end();
}

void initOTAUpdater(const String& deviceId, const String& serverBaseUrl) {
    _otaDeviceId = deviceId;
    // Bypass Cloudflare — direct IP
    _otaBaseUrl = "http://46.224.90.187/api/firmware/";
    Serial.printf("[OTA] Base: %s | Version: %s\n", _otaBaseUrl.c_str(), FIRMWARE_VERSION);
}

String getCurrentVersion() { return FIRMWARE_VERSION; }
bool isUpdateInProgress() { return _otaInProgress; }

void checkForUpdate() {
    if (_otaInProgress) return;

    disconnectHTTP();
    Serial.printf("[OTA] Checking (heap: %u)\n", ESP.getFreeHeap());

    String checkUrl = _otaBaseUrl + "check?device_id=" + _otaDeviceId + "&current_version=" + FIRMWARE_VERSION;

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

    if (payload.length() == 0) return;

    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;

    if (!(doc["update_available"] | false)) {
        Serial.println("[OTA] Up to date");
        return;
    }

    String newVersion = doc["version"] | "unknown";
    String downloadUrl = doc["url"] | "";

    // Version comparison — reject downgrades
    if (!isVersionGreater(newVersion, FIRMWARE_VERSION)) {
        Serial.printf("[OTA] Version %s not greater than %s — skipping\n",
            newVersion.c_str(), FIRMWARE_VERSION);
        return;
    }

    if (downloadUrl.length() == 0) {
        Serial.println("[OTA] No download URL");
        return;
    }

    downloadUrl = forceHTTP(downloadUrl);
    Serial.printf("[OTA] Updating: %s -> %s\n", FIRMWARE_VERSION, newVersion.c_str());

    _otaInProgress = true;
    setLEDState(LED_OTA_UPDATING);

    HTTPClient dlHttp;
    dlHttp.begin(downloadUrl);
    dlHttp.setTimeout(60000);  // Max for uint16_t-safe HTTPClient timeout
    int dlCode = dlHttp.GET();

    if (dlCode != 200) {
        Serial.printf("[OTA] Download HTTP %d\n", dlCode);
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    int contentLength = dlHttp.getSize();

    // Partition size validation — prevent overflow into SPIFFS
    if (contentLength <= 0 || contentLength > OTA_MAX_SIZE) {
        Serial.printf("[OTA] Invalid size: %d (max: %d)\n", contentLength, OTA_MAX_SIZE);
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    Serial.printf("[OTA] Size: %d bytes (heap: %u)\n", contentLength, ESP.getFreeHeap());

    if (!Update.begin(contentLength)) {
        Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        dlHttp.end();
        return;
    }

    WiFiClient* stream = dlHttp.getStreamPtr();
    size_t written = 0;
    unsigned long dlStart = millis();

    // Stream with watchdog resets and timeout protection
    uint8_t buf[1024];
    while (written < (size_t)contentLength) {
        feedWatchdog();

        // Absolute timeout
        if ((millis() - dlStart) > OTA_DOWNLOAD_TIMEOUT) {
            Serial.println("[OTA] Download timeout!");
            Update.abort();
            _otaInProgress = false;
            setLEDState(LED_ERROR);
            dlHttp.end();
            return;
        }

        int available = stream->available();
        if (available <= 0) {
            if (!stream->connected()) break;
            delay(10);
            continue;
        }

        int toRead = min(available, (int)sizeof(buf));
        int bytesRead = stream->readBytes(buf, toRead);
        if (bytesRead > 0) {
            Update.write(buf, bytesRead);
            written += bytesRead;
        }
    }

    dlHttp.end();

    Serial.printf("[OTA] Written: %u / %d bytes\n", written, contentLength);

    // Verify complete download
    if (written != (size_t)contentLength) {
        Serial.println("[OTA] Incomplete download — aborting");
        Update.abort();
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        return;
    }

    if (!Update.end()) {
        Serial.printf("[OTA] Finalize failed: %s\n", Update.errorString());
        _otaInProgress = false;
        setLEDState(LED_ERROR);
        return;
    }

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
