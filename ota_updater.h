#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>
#include <mbedtls/base64.h>
#include <mbedtls/pk.h>
#include <mbedtls/sha256.h>
#include "config.h"
#include "led_status.h"
#include "ota_trust.h"

// ============================================================================
// OTA Updater — Partition validation, version comparison, safe rollback
// ============================================================================

static String _otaDeviceId;
static String _otaBaseUrl;
static unsigned long _lastOTACheck = 0;
static volatile bool _otaInProgress = false;
static volatile bool _otaPendingValidation = false;
static volatile bool _otaForceCheck = false;
static String _otaDownloadUrl;
static String _otaTargetVersion;
static String _otaExpectedMd5;
static String _otaSignatureB64;
static uint8_t _otaExpectedSha256[32] = {};
static size_t _otaTotalBytes = 0;
static size_t _otaWrittenBytes = 0;
static unsigned long _otaSessionStartedMs = 0;
static unsigned long _otaNextChunkAtMs = 0;
static uint8_t _otaChunkFailures = 0;
static const char* _otaStatus = "idle";
static WiFiClient _otaClient;
static mbedtls_sha256_context _otaSha256;
static bool _otaSha256Active = false;

// Arduino-ESP32 otherwise marks a PENDING_VERIFY image valid inside
// initArduino(), before setup() and before any network/data health check. This
// strong override keeps the bootloader rollback window open until otaLoop()
// observes five healthy minutes and a recent production-data acknowledgement.
extern "C" bool verifyRollbackLater() { return true; }

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

// Rollback check — called FIRST in setup(). The ESP-IDF bootloader handles the
// actual rollback if a pending image resets before we explicitly validate it.
void checkFirmwareRollback() {
    Preferences otaPrefs;
    otaPrefs.begin("otastate", false);
    bool justUpdated = otaPrefs.getBool("updated", false);

    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (running != nullptr &&
        esp_ota_get_state_partition(running, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        _otaPendingValidation = true;
        _otaStatus = "validating";
        Serial.println("[OTA] New image pending: starting five-minute health validation");
    } else if (justUpdated) {
        // A pending image that rebooted was rejected by the bootloader before
        // reaching this code. Clear only our narration marker; never select an
        // INVALID/ABORTED partition as the old implementation did.
        otaPrefs.putBool("updated", false);
        otaPrefs.putInt("crashes", 0);
        Serial.println("[OTA] Update marker without pending image (already valid or rolled back)");
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
bool isUpdateInProgress() { return _otaInProgress || _otaPendingValidation; }
const char* getOTAStatus() { return _otaStatus; }
uint32_t getOTAProgress() { return (uint32_t)_otaWrittenBytes; }
uint32_t getOTATotal() { return (uint32_t)_otaTotalBytes; }
const String& getOTATarget() { return _otaTargetVersion; }

static bool otaParseSha256(const String& hex, uint8_t out[32]) {
    if (hex.length() != 64) return false;
    for (int i = 0; i < 32; ++i) {
        char pair[3] = {hex[i * 2], hex[i * 2 + 1], 0};
        char* end = nullptr;
        long value = strtol(pair, &end, 16);
        if (end != pair + 2 || value < 0 || value > 255) return false;
        out[i] = (uint8_t)value;
    }
    return true;
}

static bool otaVerifyReleaseSignature(const uint8_t digest[32],
                                      const String& version,
                                      size_t imageSize) {
    uint8_t signature[96];
    size_t signatureBytes = 0;
    int rc = mbedtls_base64_decode(
        signature, sizeof(signature), &signatureBytes,
        (const uint8_t*)_otaSignatureB64.c_str(), _otaSignatureB64.length());
    if (rc != 0 || signatureBytes == 0) return false;

    // Bind the signed image to its advertised version and exact length. If we
    // signed only the image digest, an on-path attacker could replay an older
    // legitimate image while relabelling its unsigned JSON version as newer.
    mbedtls_sha256_context manifest;
    mbedtls_sha256_init(&manifest);
    uint8_t manifestDigest[32];
    const char prefix[] = "linesights-ota-v1";
    const uint8_t zero = 0;
    String sizeText = String((unsigned)imageSize);
    rc = mbedtls_sha256_starts_ret(&manifest, 0);
    if (rc == 0) rc = mbedtls_sha256_update_ret(
        &manifest, (const uint8_t*)prefix, sizeof(prefix));
    if (rc == 0) rc = mbedtls_sha256_update_ret(
        &manifest, (const uint8_t*)version.c_str(), version.length());
    if (rc == 0) rc = mbedtls_sha256_update_ret(&manifest, &zero, 1);
    if (rc == 0) rc = mbedtls_sha256_update_ret(
        &manifest, (const uint8_t*)sizeText.c_str(), sizeText.length());
    if (rc == 0) rc = mbedtls_sha256_update_ret(&manifest, &zero, 1);
    if (rc == 0) rc = mbedtls_sha256_update_ret(&manifest, digest, 32);
    if (rc == 0) rc = mbedtls_sha256_finish_ret(&manifest, manifestDigest);
    mbedtls_sha256_free(&manifest);
    if (rc != 0) return false;

    mbedtls_pk_context key;
    mbedtls_pk_init(&key);
    rc = mbedtls_pk_parse_public_key(
        &key, (const uint8_t*)OTA_SIGNING_PUBLIC_KEY,
        sizeof(OTA_SIGNING_PUBLIC_KEY));
    if (rc == 0) {
        rc = mbedtls_pk_verify(
            &key, MBEDTLS_MD_SHA256, manifestDigest, 32,
            signature, signatureBytes);
    }
    mbedtls_pk_free(&key);
    return rc == 0;
}

static void otaValidationLoop() {
    if (!_otaPendingValidation) return;
    if (millis() < OTA_HEALTH_VALIDATION_MS) return;
    long successAge = getLastSuccessAgeS();
    if (successAge < 0 || successAge > 60 || getQueueSize() != 0 ||
        ESP.getMinFreeHeap() < OTA_VALIDATION_MIN_HEAP) return;

    esp_err_t rc = esp_ota_mark_app_valid_cancel_rollback();
    if (rc != ESP_OK) {
        Serial.printf("[OTA] Health validation could not mark image valid: %d\n", (int)rc);
        _otaStatus = "failed";
        return;
    }
    Preferences otaPrefs;
    otaPrefs.begin("otastate", false);
    otaPrefs.putBool("updated", false);
    otaPrefs.putInt("crashes", 0);
    otaPrefs.end();
    _otaPendingValidation = false;
    _otaStatus = "idle";
    Serial.println("[OTA] Five-minute health validation passed; image marked valid");
}

static void otaFail(const char* reason) {
    Serial.printf("[OTA] Failed: %s (%u/%u bytes)\n", reason,
                  (unsigned)_otaWrittenBytes, (unsigned)_otaTotalBytes);
    if (_otaInProgress) Update.abort();
    if (_otaSha256Active) {
        mbedtls_sha256_free(&_otaSha256);
        _otaSha256Active = false;
    }
    _otaInProgress = false;
    _otaStatus = "failed";
    _otaDownloadUrl = "";
    _otaExpectedMd5 = "";
    _otaSignatureB64 = "";
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

    int metadataBytes = http.getSize();
    if (metadataBytes <= 0 || metadataBytes > 4096) {
        Serial.printf("[OTA] Invalid metadata response size: %d\n", metadataBytes);
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
    String expectedSha256 = doc["sha256"] | "";
    String signature = doc["signature"] | "";
    String signatureAlg = doc["signature_alg"] | "";
    uint32_t totalBytes = doc["size"] | 0U;

    // Version comparison — reject downgrades
    if (!isVersionGreater(newVersion, FIRMWARE_VERSION)) {
        Serial.printf("[OTA] Version %s not greater than %s — skipping\n",
            newVersion.c_str(), FIRMWARE_VERSION);
        _otaStatus = "idle";
        return;
    }

    if (downloadUrl.length() == 0 || totalBytes == 0 || totalBytes > OTA_MAX_SIZE ||
        expectedMd5.length() != 32 || signature.length() == 0 ||
        signatureAlg != "ecdsa-p256-sha256-manifest-v1" ||
        !otaParseSha256(expectedSha256, _otaExpectedSha256)) {
        Serial.println("[OTA] Missing or invalid signed release metadata");
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
    mbedtls_sha256_init(&_otaSha256);
    if (mbedtls_sha256_starts_ret(&_otaSha256, 0) != 0) {
        mbedtls_sha256_free(&_otaSha256);
        otaFail("SHA-256 initialization");
        return;
    }
    _otaSha256Active = true;

    _otaDownloadUrl = downloadUrl;
    _otaTargetVersion = newVersion;
    _otaExpectedMd5 = expectedMd5;
    _otaSignatureB64 = signature;
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
                if (mbedtls_sha256_update_ret(&_otaSha256, buffer, got) != 0) {
                    writeFailed = true;
                    break;
                }
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
    uint8_t digest[32];
    if (!_otaSha256Active ||
        mbedtls_sha256_finish_ret(&_otaSha256, digest) != 0) {
        otaFail("SHA-256 finalize");
        return;
    }
    mbedtls_sha256_free(&_otaSha256);
    _otaSha256Active = false;
    uint8_t mismatch = 0;
    for (int i = 0; i < 32; ++i) mismatch |= digest[i] ^ _otaExpectedSha256[i];
    if (mismatch != 0 || !otaVerifyReleaseSignature(
            digest, _otaTargetVersion, _otaTotalBytes)) {
        otaFail("release signature verification");
        return;
    }
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
    otaValidationLoop();
    if (_otaPendingValidation) return;
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
