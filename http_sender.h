#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "config.h"
#include "ct_sensor.h"

#include <esp32/rom/miniz.h>

// Forward declaration
void logError(const String& message);

// ============================================================================
// HTTP Sender — Tiered storage for zero data loss
//   Online:  1/sec JSON POST
//   Offline: 1/sec binary → compress in 60-reading blocks → append to flash
//   Resume:  Live data first, upload offline.dat in background
// ============================================================================

// --- File header for offline data ---
#define OFFLINE_MAGIC "LS01"

struct __attribute__((packed)) OfflineHeader {
    char magic[4];              // "LS01"
    char device_id[32];         // null-padded
    uint32_t start_epoch;       // first reading timestamp
};

// --- RAM ring buffer for online sending ---
#define BUFFER_FILE "/buffer.json"
#define BUFFER_TMP  "/buffer.tmp"

struct BufferedReading {
    String json;
    bool used;
};

static BufferedReading _sendBuffer[MAX_BUFFER_SIZE];
static int _bufferHead = 0;
static int _bufferTail = 0;
static volatile int _bufferCount = 0;

static String _httpServerUrl;
static String _bulkUploadUrl;
static uint32_t _totalSent = 0;
static uint32_t _totalFailed = 0;
static uint32_t _totalDropped = 0;
static int _consecutiveFailures = 0;
static unsigned long _lastSendAttempt = 0;
static int _backoffMs = 1000;
static bool _httpDebug = false;
static unsigned long _lastBufferSave = 0;
static bool _fsReady = false;

// --- Offline state ---
static bool _offlineMode = false;
static unsigned long _wifiDownSince = 0;
static uint32_t _offlineReadingsStored = 0;
static uint32_t _offlineBlockCount = 0;
static uint32_t _offlineFileSize = 0;
static unsigned long _lastUploadAttempt = 0;
static bool _uploadPending = false;

// Block accumulator (60 readings of 12 bytes each = 720 bytes)
static uint8_t _blockBuf[OFFLINE_BLOCK_RAW_SIZE];
static int _blockIdx = 0;  // byte offset into _blockBuf
static int _blockReadings = 0;  // readings in current block

static SemaphoreHandle_t _bufMutex = NULL;

void setBufferMutex(SemaphoreHandle_t m) { _bufMutex = m; }
void setHTTPDebug(bool on) { _httpDebug = on; }
bool getHTTPDebug() { return _httpDebug; }
void disconnectHTTP() {}
bool isOfflineMode() { return _offlineMode; }
uint32_t getOfflineStored() { return _offlineReadingsStored; }
uint32_t getOfflineFileSize() { return _offlineFileSize; }

// --- LittleFS buffer persistence (for short outages) ---
void loadBufferFromFlash() {
    if (!_fsReady || !LittleFS.exists(BUFFER_FILE)) return;
    File f = LittleFS.open(BUFFER_FILE, "r");
    if (!f) return;
    String content = f.readString();
    f.close();

    JsonDocument doc;
    if (deserializeJson(doc, content)) { LittleFS.remove(BUFFER_FILE); return; }

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
    LittleFS.remove(BUFFER_FILE);
    if (loaded > 0) Serial.printf("[BUF] Loaded %d readings from flash\n", loaded);
}

void saveBufferToFlash() {
    if (!_fsReady || _bufferCount == 0) return;
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        int idx = _bufferTail;
        for (int i = 0; i < _bufferCount; i++) {
            if (_sendBuffer[idx].used) {
                JsonDocument item;
                deserializeJson(item, _sendBuffer[idx].json);
                arr.add(item);
            }
            idx = (idx + 1) % MAX_BUFFER_SIZE;
        }
        xSemaphoreGive(_bufMutex);
    } else return;

    File f = LittleFS.open(BUFFER_TMP, "w");
    if (!f) return;
    serializeJson(doc, f);
    f.close();
    if (LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
    LittleFS.rename(BUFFER_TMP, BUFFER_FILE);
}

// ====================================================================
// OFFLINE STORAGE: Compressed binary blocks on LittleFS
// ====================================================================

// Write file header (called once when entering offline mode)
void writeOfflineHeader(const String& deviceId) {
    File f = LittleFS.open(OFFLINE_FILE, "w");
    if (!f) { Serial.println("[OFFLINE] Failed to create file"); return; }

    OfflineHeader hdr;
    memcpy(hdr.magic, OFFLINE_MAGIC, 4);
    memset(hdr.device_id, 0, 32);
    strncpy(hdr.device_id, deviceId.c_str(), 31);

    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
        hdr.start_epoch = mktime(&timeinfo);
    } else {
        hdr.start_epoch = millis() / 1000;
    }

    f.write((uint8_t*)&hdr, sizeof(hdr));
    f.close();
    _offlineFileSize = sizeof(hdr);
    _offlineReadingsStored = 0;
    _offlineBlockCount = 0;
}

// Flush current block: compress and append to file
void flushOfflineBlock() {
    if (_blockReadings == 0) return;
    if (!_fsReady) return;

    int rawSize = _blockReadings * 12;

    // Compress the block using tdefl (available in ESP32 ROM)
    uint8_t compressed[1024];
    size_t compLen = tdefl_compress_mem_to_mem(compressed, sizeof(compressed),
                                               _blockBuf, rawSize,
                                               TDEFL_DEFAULT_MAX_PROBES);

    if (compLen == 0 || compLen == (size_t)-1) {
        Serial.printf("[OFFLINE] Compression failed — writing raw\n");
        // Fallback: write uncompressed with flag
        File f = LittleFS.open(OFFLINE_FILE, "a");
        if (f) {
            uint16_t sz = rawSize | 0x8000;  // Bit 15 = uncompressed flag
            f.write((uint8_t*)&sz, 2);
            f.write(_blockBuf, rawSize);
            _offlineFileSize += 2 + rawSize;
            f.close();
        }
    } else {
        File f = LittleFS.open(OFFLINE_FILE, "a");
        if (f) {
            uint16_t sz = (uint16_t)compLen;  // Bit 15 clear = compressed
            f.write((uint8_t*)&sz, 2);
            f.write(compressed, compLen);
            _offlineFileSize += 2 + compLen;
            f.close();
        }
    }

    _offlineReadingsStored += _blockReadings;
    _offlineBlockCount++;

    float ratio = (float)(rawSize) / (compLen > 0 ? compLen : 1);
    Serial.printf("[OFFLINE] Block %u: %d readings, %d→%lu bytes (%.1fx), total %uKB, %u readings\n",
        _offlineBlockCount, _blockReadings, rawSize, compLen, ratio,
        _offlineFileSize / 1024, _offlineReadingsStored);

    _blockReadings = 0;
    _blockIdx = 0;
}

// Store one reading into the block accumulator
void storeOfflineReading(CTReading readings[6]) {
    if (!_fsReady) return;

    // Check file size cap
    if (_offlineFileSize >= OFFLINE_MAX_BYTES) {
        static bool warned = false;
        if (!warned) { Serial.println("[OFFLINE] Flash full — cannot store more"); warned = true; }
        return;
    }

    // Pack 6 amps values as int16_t × 1000 (milliamps)
    for (int i = 0; i < 6; i++) {
        int16_t a = (int16_t)(readings[i].amps * 1000.0f);
        memcpy(&_blockBuf[_blockIdx], &a, 2);
        _blockIdx += 2;
    }
    _blockReadings++;

    // Flush block when full (every 60 readings = 1 minute)
    if (_blockReadings >= OFFLINE_BLOCK_READINGS) {
        flushOfflineBlock();
    }
}

void enterOfflineMode(const String& deviceId) {
    if (_offlineMode) return;
    _offlineMode = true;
    _blockReadings = 0;
    _blockIdx = 0;

    // Save RAM buffer to flash first
    saveBufferToFlash();

    // Create/overwrite offline file with header (or append if file exists from previous session)
    if (!LittleFS.exists(OFFLINE_FILE)) {
        writeOfflineHeader(deviceId);
    } else {
        // Resume appending to existing file
        File f = LittleFS.open(OFFLINE_FILE, "r");
        if (f) { _offlineFileSize = f.size(); f.close(); }
    }

    Serial.printf("[OFFLINE] Entered — 1Hz binary, 60-reading compressed blocks\n");
    Serial.printf("[OFFLINE] File: %uKB, max %uKB\n", _offlineFileSize / 1024, OFFLINE_MAX_BYTES / 1024);
}

void exitOfflineMode() {
    if (!_offlineMode) return;

    // Flush any remaining readings in the current block
    if (_blockReadings > 0) {
        flushOfflineBlock();
    }

    _offlineMode = false;
    _wifiDownSince = 0;
    _uploadPending = true;
    _lastUploadAttempt = 0;

    Serial.printf("[OFFLINE] Exited — %u readings stored in %uKB, upload pending\n",
        _offlineReadingsStored, _offlineFileSize / 1024);
}

// Upload offline file to server (single POST, background)
bool uploadOfflineFile(const String& deviceId) {
    if (!_fsReady || !LittleFS.exists(OFFLINE_FILE)) {
        _uploadPending = false;
        return true;
    }

    File f = LittleFS.open(OFFLINE_FILE, "r");
    if (!f) { _uploadPending = false; return true; }

    size_t fileSize = f.size();
    if (fileSize <= sizeof(OfflineHeader)) {
        f.close();
        LittleFS.remove(OFFLINE_FILE);
        _uploadPending = false;
        return true;
    }

    Serial.printf("[UPLOAD] Sending %u bytes to server...\n", fileSize);

    // Read entire file into memory for POST
    // Max ~550KB — we have ~280KB free heap... need to stream instead
    // Use chunked reading: POST with WiFiClient directly

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(_bulkUploadUrl);
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("X-Device-Id", deviceId);
    http.addHeader("X-Format", "ls01-blocked");
    http.addHeader("Content-Length", String(fileSize));
    http.setTimeout(30000);  // 30s for large upload

    // Stream file directly
    int httpCode = http.sendRequest("POST", &f, fileSize);
    f.close();
    http.end();

    if (httpCode == 200) {
        Serial.printf("[UPLOAD] Success! %u readings delivered. Deleting file.\n", _offlineReadingsStored);
        LittleFS.remove(OFFLINE_FILE);
        _uploadPending = false;
        _offlineReadingsStored = 0;
        _offlineFileSize = 0;
        _offlineBlockCount = 0;
        return true;
    } else {
        Serial.printf("[UPLOAD] Failed: HTTP %d — will retry in 60s\n", httpCode);
        _lastUploadAttempt = millis();
        return false;
    }
}

// ====================================================================
// INIT
// ====================================================================
void initHTTPSender(const String& serverUrl) {
    _httpServerUrl = "http://46.224.90.187/api/data";
    _bulkUploadUrl = "http://46.224.90.187/api/data/bulk";

    for (int i = 0; i < MAX_BUFFER_SIZE; i++) _sendBuffer[i].used = false;
    _bufferCount = 0; _bufferHead = 0; _bufferTail = 0;

    if (LittleFS.begin(true)) {
        _fsReady = true;
        loadBufferFromFlash();

        // Check for pending offline data from previous session
        if (LittleFS.exists(OFFLINE_FILE)) {
            File f = LittleFS.open(OFFLINE_FILE, "r");
            if (f) {
                _offlineFileSize = f.size();
                _offlineReadingsStored = (_offlineFileSize - sizeof(OfflineHeader)) / 4;  // Approximate
                f.close();
                _uploadPending = true;
                Serial.printf("[FS] Found offline data: %uKB — will upload when connected\n",
                    _offlineFileSize / 1024);
            }
        }

        Serial.printf("[FS] LittleFS ready, %u/%u bytes\n", LittleFS.usedBytes(), LittleFS.totalBytes());
    } else {
        Serial.println("[FS] LittleFS failed");
    }
    Serial.printf("[HTTP] -> %s\n", _httpServerUrl.c_str());
}

// ====================================================================
// QUEUE READING (online mode, mutex must be held)
// ====================================================================
void queueReading(const String& deviceId, const String& location, const String& timezone,
                  float gridVoltage, CTReading readings[NUM_CT_CHANNELS], const String& timestamp) {

    JsonDocument doc;
    doc["device_id"] = deviceId;
    doc["timestamp"] = timestamp;
    doc["location"] = location;
    doc["timezone"] = timezone;

    JsonObject cts = doc["readings"]["cts"].to<JsonObject>();
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        char key[8]; snprintf(key, sizeof(key), "ct_%d", i + 1);
        JsonObject ct = cts[key].to<JsonObject>();
        float w = readings[i].watts, a = readings[i].amps;
        if (isnan(w) || isinf(w)) w = 0.0f;
        if (isnan(a) || isinf(a)) a = 0.0f;
        ct["real_power_w"] = serialized(String(w, 1));
        ct["amps"] = serialized(String(a, 3));
        ct["pf"] = serialized(String(readings[i].pf, 3));
    }
    doc["readings"]["voltage_rms"] = serialized(String(gridVoltage, 1));

    String json;
    serializeJson(doc, json);

    if (_bufferCount >= MAX_BUFFER_SIZE) {
        _sendBuffer[_bufferTail].json = String();
        _sendBuffer[_bufferTail].used = false;
        _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
        _bufferCount--;
        _totalDropped++;
        recordSendDrop();
    }

    _sendBuffer[_bufferHead].json = json;
    _sendBuffer[_bufferHead].used = true;
    _bufferHead = (_bufferHead + 1) % MAX_BUFFER_SIZE;
    _bufferCount++;
}

// ====================================================================
// PROCESS SEND QUEUE (Core 0) — Live first, then offline upload
// ====================================================================
void processSendQueue() {
    if (WiFi.status() != WL_CONNECTED) {
        if (_wifiDownSince == 0) _wifiDownSince = millis();
        if (!_offlineMode && (millis() - _wifiDownSince > OFFLINE_GRACE_MS)) {
            // Will be called from main loop with device ID
        }
        return;
    }

    // WiFi is up
    if (_offlineMode) exitOfflineMode();
    _wifiDownSince = 0;

    // Priority 1: Send live buffered readings
    if (_bufferCount > 0) {
        unsigned long now = millis();
        if (_consecutiveFailures > 0 && (now - _lastSendAttempt) < (unsigned long)_backoffMs) goto periodic;

        int sent = 0;
        while (_bufferCount > 0 && sent < MAX_SENDS_PER_LOOP) {
            String json;
            if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (!_sendBuffer[_bufferTail].used) { xSemaphoreGive(_bufMutex); break; }
                json = _sendBuffer[_bufferTail].json;
                xSemaphoreGive(_bufMutex);
            } else break;

            // Skip corrupt entries
            if (json.length() < 10 || json.indexOf("device_id") < 0) {
                if (_fsReady) {
                    File rf = LittleFS.open("/rejected.log", "a");
                    if (rf) { rf.println(json); rf.close(); }
                }
                if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    _sendBuffer[_bufferTail].used = false;
                    _sendBuffer[_bufferTail].json = String();
                    _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                    _bufferCount--;
                    xSemaphoreGive(_bufMutex);
                }
                continue;
            }

            HTTPClient http;
            http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
            http.begin(_httpServerUrl);
            http.addHeader("Content-Type", "application/json");
            http.setTimeout(HTTP_TIMEOUT_MS);
            int httpCode = http.POST(json);
            http.end();
            _lastSendAttempt = millis();

            if (httpCode == 200) {
                _totalSent++; _consecutiveFailures = 0; _backoffMs = 1000;
                recordSendSuccess();
                if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    _sendBuffer[_bufferTail].used = false;
                    _sendBuffer[_bufferTail].json = String();
                    _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                    _bufferCount--;
                    xSemaphoreGive(_bufMutex);
                }
                sent++;
            } else if (httpCode == 400) {
                if (_fsReady) {
                    File rf = LittleFS.open("/rejected.log", "a");
                    if (rf) { rf.println(json); rf.close(); }
                }
                if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    _sendBuffer[_bufferTail].used = false;
                    _sendBuffer[_bufferTail].json = String();
                    _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                    _bufferCount--;
                    xSemaphoreGive(_bufMutex);
                }
                logError("400 — saved to /rejected.log");
                sent++;
            } else {
                _totalFailed++; _consecutiveFailures++;
                _backoffMs = min(_backoffMs * 2, (int)MAX_BACKOFF_MS);
                recordSendFailure();
                if (httpCode > 0) {
                    if (_consecutiveFailures == 1) logError("HTTP " + String(httpCode));
                } else {
                    if (_consecutiveFailures == 1) logError("HTTP err: " + http.errorToString(httpCode));
                }
                if (_consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                    logError("HTTP stall: backing off");
                    _backoffMs = MAX_BACKOFF_MS;
                    _consecutiveFailures = 0;
                }
                break;
            }
        }
    }

    // Priority 2: Upload offline file in background (only when live queue is empty)
    if (_uploadPending && _bufferCount == 0) {
        unsigned long now = millis();
        if (now - _lastUploadAttempt >= OFFLINE_UPLOAD_RETRY_MS) {
            uploadOfflineFile("");  // Device ID is in the file header
        }
    }

periodic:
    // Periodic RAM buffer save
    unsigned long now = millis();
    if (_fsReady && (now - _lastBufferSave >= BUFFER_SAVE_INTERVAL_MS)) {
        _lastBufferSave = now;
        if (_bufferCount > 0) saveBufferToFlash();
        else if (LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
    }
}

int getQueueSize()          { return _bufferCount; }
uint32_t getTotalSent()     { return _totalSent; }
uint32_t getTotalFailed()   { return _totalFailed; }
uint32_t getTotalDropped()  { return _totalDropped; }
bool isUploadPending()      { return _uploadPending; }
