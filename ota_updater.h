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
static volatile bool _otaInProgress = false;
static volatile bool _otaForceCheck = false;
static String _otaDownloadUrl;
static String _otaTargetVersion;
static String _otaExpectedMd5;
static size_t _otaTotalBytes = 0;
static size_t _otaWrittenBytes = 0;
static unsigned long _otaSessionStartedMs = 0;
static unsigned long _otaNextChunkAtMs = 0;
static uint8_t _otaChunkFailures = 0;
static const char* _otaStatus = "idle";
static WiFiClient _otaClient;

void forceOTACheck() { _otaForceCheck = true; }

void logError(const String& message);
void disconnectHTTP();
void feedWatchdog();
int getQueueSize();
long getLastSuccessAgeS();

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
const char* getOTAStatus() { return _otaStatus; }
uint32_t getOTAProgress() { return (uint32_t)_otaWrittenBytes; }
uint32_t getOTATotal() { return (uint32_t)_otaTotalBytes; }
const String& getOTATarget() { return _otaTargetVersion; }

static void otaFail(const char* reason) {
    Serial.printf("[OTA] Failed: %s (%u/%u bytes)\n", reason,
                  (unsigned)_otaWrittenBytes, (unsigned)_otaTotalBytes);
    if (_otaInProgress) Update.abort();
    _otaInProgress = false;
    _otaStatus = "failed";
    _otaDownloadUrl = "";
    _otaExpectedMd5 = "";
    _otaNextChunkAtMs = 0;
    _otaClient.stop();
    setLEDState(LED_ERROR);
}

void checkForUpdate() {
    if (_otaInProgress) return;

    disconnectHTTP();
    Serial.printf("[OTA] Checking (heap: %u)\n", ESP.getFreeHeap());

    char checkUrlBuf[256];
    snprintf(checkUrlBuf, sizeof(checkUrlBuf), "%scheck?device_id=%s&current_version=%s",
        _otaBaseUrl.c_str(), _otaDeviceId.c_str(), FIRMWARE_VERSION);
    String checkUrl = checkUrlBuf;

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(checkUrl);
    http.setTimeout(10000);
    int httpCode = http.GET();

    if (httpCode != 200) {
        Serial.printf("[OTA] Check failed: HTTP %d\n", httpCode);
        http.end();
        _otaStatus = "failed";
        return;
    }

    String payload = http.getString();
    http.end();

    if (payload.length() == 0) {
        _otaStatus = "failed";
        return;
    }

    JsonDocument doc;
    if (deserializeJson(doc, payload)) {
        _otaStatus = "failed";
        return;
    }

    if (!(doc["update_available"] | false)) {
        Serial.println("[OTA] Up to date");
        _otaStatus = "idle";
        return;
    }

    String newVersion = doc["version"] | "unknown";
    String downloadUrl = doc["url"] | "";
    String expectedMd5 = doc["md5"] | "";
    uint32_t totalBytes = doc["size"] | 0U;

    // Version comparison — reject downgrades
    if (!isVersionGreater(newVersion, FIRMWARE_VERSION)) {
        Serial.printf("[OTA] Version %s not greater than %s — skipping\n",
            newVersion.c_str(), FIRMWARE_VERSION);
        _otaStatus = "idle";
        return;
    }

    if (downloadUrl.length() == 0 || totalBytes == 0 || totalBytes > OTA_MAX_SIZE ||
        expectedMd5.length() != 32) {
        Serial.println("[OTA] Missing URL, MD5, or valid size metadata");
        _otaStatus = "failed";
        return;
    }

    downloadUrl = forceHTTP(downloadUrl);
    Serial.printf("[OTA] Staging %s -> %s in %u-byte ranges (%u bytes total)\n",
                  FIRMWARE_VERSION, newVersion.c_str(),
                  (unsigned)OTA_CHUNK_BYTES, (unsigned)totalBytes);
    if (!Update.begin(totalBytes)) {
        Serial.printf("[OTA] Begin failed: %s\n", Update.errorString());
        _otaStatus = "failed";
        return;
    }
    // From this point otaFail() must abort the open Update transaction.
    _otaInProgress = true;
    if (!Update.setMD5(expectedMd5.c_str())) {
        otaFail("invalid MD5 metadata");
        return;
    }

    _otaDownloadUrl = downloadUrl;
    _otaTargetVersion = newVersion;
    _otaExpectedMd5 = expectedMd5;
    _otaTotalBytes = totalBytes;
    _otaWrittenBytes = 0;
    _otaSessionStartedMs = millis();
    _otaNextChunkAtMs = millis();
    _otaChunkFailures = 0;
    _otaStatus = "downloading";
    setLEDState(LED_OTA_UPDATING);
}

static void otaDownloadChunk() {
    if (!_otaInProgress || _otaWrittenBytes >= _otaTotalBytes) return;
    unsigned long now = millis();
    if (now - _otaSessionStartedMs > OTA_SESSION_MAX_MS) {
        otaFail("session timeout");
        return;
    }
    if ((int32_t)(now - _otaNextChunkAtMs) < 0) return;
    // processSendQueue() runs before otaLoop(); if it could not empty ordinary
    // telemetry, do no firmware work this turn.
    if (getQueueSize() != 0) return;

    size_t start = _otaWrittenBytes;
    size_t end = min(start + (size_t)OTA_CHUNK_BYTES, _otaTotalBytes) - 1;
    size_t expected = end - start + 1;
    String range = String("bytes=") + (unsigned)start + "-" + (unsigned)end;

    HTTPClient http;
    http.setReuse(true);
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(_otaClient, _otaDownloadUrl);
    http.setTimeout(15000);
    const char* responseHeaders[] = {"Content-Range"};
    http.collectHeaders(responseHeaders, 1);
    http.addHeader("Accept-Encoding", "identity");
    http.addHeader("Range", range);
    int code = http.GET();
    String expectedContentRange = String("bytes ") + (unsigned)start + "-" +
                                  (unsigned)end + "/" + (unsigned)_otaTotalBytes;
    String actualContentRange = http.header("Content-Range");
    if (code != 206 || http.getSize() != (int)expected ||
        actualContentRange != expectedContentRange) {
        Serial.printf("[OTA] Range %u-%u failed: HTTP %d size %d content-range '%s'\n",
                      (unsigned)start, (unsigned)end, code, http.getSize(),
                      actualContentRange.c_str());
        http.end();
        _otaClient.stop();
        _otaChunkFailures++;
        if (_otaChunkFailures >= OTA_MAX_CHUNK_FAILURES) otaFail("range failures");
        else _otaNextChunkAtMs = millis() + OTA_CHUNK_RETRY_MS;
        return;
    }

    WiFiClient* stream = http.getStreamPtr();
    uint8_t buffer[1024];
    size_t requestRead = 0;
    unsigned long chunkStarted = millis();
    unsigned long lastByteAt = chunkStarted;
    bool writeFailed = false;
    while (requestRead < expected) {
        feedWatchdog();
        int available = stream->available();
        if (available > 0) {
            int toRead = min(available, (int)sizeof(buffer));
            toRead = min(toRead, (int)(expected - requestRead));
            int got = stream->readBytes(buffer, toRead);
            if (got > 0) {
                size_t written = Update.write(buffer, got);
                if (written != (size_t)got) {
                    writeFailed = true;
                    break;
                }
                requestRead += got;
                _otaWrittenBytes += got;
                lastByteAt = millis();
                continue;
            }
        }
        if (!stream->connected() || millis() - lastByteAt > OTA_CHUNK_STALL_MS ||
            millis() - chunkStarted > OTA_CHUNK_MAX_MS) break;
        delay(5);
    }
    http.end();

    if (writeFailed) {
        otaFail("flash write");
        return;
    }
    if (requestRead != expected) {
        _otaClient.stop();
        Serial.printf("[OTA] Partial range: +%u/%u, resume at %u\n",
                      (unsigned)requestRead, (unsigned)expected,
                      (unsigned)_otaWrittenBytes);
        if (requestRead == 0) _otaChunkFailures++;
        else _otaChunkFailures = 0;
        if (_otaChunkFailures >= OTA_MAX_CHUNK_FAILURES) otaFail("stalled ranges");
        else _otaNextChunkAtMs = millis() + OTA_CHUNK_RETRY_MS;
        return;
    }

    _otaChunkFailures = 0;
    _otaNextChunkAtMs = millis();
    Serial.printf("[OTA] Progress: %u/%u\n",
                  (unsigned)_otaWrittenBytes, (unsigned)_otaTotalBytes);

    if (_otaWrittenBytes != _otaTotalBytes) return;
    if (!Update.end() || !Update.isFinished()) {
        otaFail("finalize or MD5 verification");
        return;
    }

    Preferences otaPrefs;
    otaPrefs.begin("otastate", false);
    otaPrefs.putBool("updated", true);
    otaPrefs.putInt("crashes", 0);
    otaPrefs.end();
    _otaStatus = "flashing";
    Serial.printf("[OTA] Success! Rebooting to %s...\n", _otaTargetVersion.c_str());
    flushBeforeRestart();
    delay(1000);
    ESP.restart();
}

// Core 0 state machine. Each turn performs at most one bounded range request,
// returning to live uploads and heartbeat processing between firmware chunks.
void otaLoop() {
    if (_otaInProgress) {
        otaDownloadChunk();
        return;
    }

    unsigned long now = millis();
    if (_otaForceCheck || (now - _lastOTACheck >= OTA_CHECK_INTERVAL_MS)) {
        long successAge = getLastSuccessAgeS();
        if (getQueueSize() != 0 || successAge < 0 || successAge > 30) return;
        _lastOTACheck = now;
        _otaForceCheck = false;
        _otaStatus = "checking";
        checkForUpdate();
    }
}
