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
// HTTP Sender v4 — SPINLOCK RULES:
//   portENTER_CRITICAL only for simple variable read/write (no I/O, no alloc)
//   FreeRTOS mutex (_bufMutex) for longer operations on buffer entries
//   File operations: never inside any lock (LittleFS needs interrupts)
// ============================================================================

#define OFFLINE_MAGIC "LS01"
#define BUFFER_FILE "/buffer.json"
#define BUFFER_TMP  "/buffer.tmp"

struct __attribute__((packed)) OfflineHeader {
    char magic[4];
    char device_id[32];
    uint32_t start_epoch;
};

struct BufferedReading {
    String json;
    bool used;
};

static BufferedReading _sendBuffer[MAX_BUFFER_SIZE];
static int _bufferHead = 0;
static int _bufferTail = 0;
static int _bufferCount = 0;

static String _httpServerUrl;
static String _bulkUploadUrl;
static String _httpDeviceId;
static uint32_t _totalSent = 0;
static uint32_t _totalFailed = 0;
static uint32_t _totalDropped = 0;
static int _consecutiveFailures = 0;
static unsigned long _lastSendAttempt = 0;
static int _backoffMs = 1000;
static bool _httpDebug = false;
static unsigned long _lastBufferSave = 0;
static bool _fsReady = false;

// Spinlocks — ONLY for simple variable read/write, NEVER for I/O
static portMUX_TYPE _bufCntMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE _offlineMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE _blockMux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool _offlineMode = false;
static unsigned long _wifiDownSince = 0;
static uint32_t _offlineReadingsStored = 0;
static uint32_t _offlineBlockCount = 0;
static uint32_t _offlineFileSize = 0;
static unsigned long _lastUploadAttempt = 0;
static volatile bool _uploadPending = false;

// Block accumulator
static uint8_t _blockBuf[OFFLINE_BLOCK_RAW_SIZE];
static int _blockIdx = 0;
static int _blockReadings = 0;

static SemaphoreHandle_t _bufMutex = NULL;

// Atomic buffer count helpers
static inline int bufCount() {
    portENTER_CRITICAL(&_bufCntMux); int c = _bufferCount; portEXIT_CRITICAL(&_bufCntMux); return c;
}
static inline void bufCountInc() {
    portENTER_CRITICAL(&_bufCntMux); _bufferCount++; portEXIT_CRITICAL(&_bufCntMux);
}
static inline void bufCountDec() {
    portENTER_CRITICAL(&_bufCntMux); _bufferCount--; portEXIT_CRITICAL(&_bufCntMux);
}

void setBufferMutex(SemaphoreHandle_t m) { _bufMutex = m; }
void setHTTPDebug(bool on) { _httpDebug = on; }
bool getHTTPDebug() { return _httpDebug; }
void disconnectHTTP() {}

bool isOfflineMode() {
    portENTER_CRITICAL(&_offlineMux); bool m = _offlineMode; portEXIT_CRITICAL(&_offlineMux); return m;
}
uint32_t getOfflineStored() { return _offlineReadingsStored; }
uint32_t getOfflineFileSize() { return _offlineFileSize; }
bool isUploadPending() { return _uploadPending; }

// --- LittleFS buffer persistence ---
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
        if (bufCount() >= MAX_BUFFER_SIZE) break;
        String json;
        serializeJson(v, json);
        _sendBuffer[_bufferHead].json = json;
        _sendBuffer[_bufferHead].used = true;
        _bufferHead = (_bufferHead + 1) % MAX_BUFFER_SIZE;
        bufCountInc();
        loaded++;
    }
    LittleFS.remove(BUFFER_FILE);
    if (loaded > 0) Serial.printf("[BUF] Loaded %d readings from flash\n", loaded);
}

// FIX: Copy buffer entries under mutex, write file WITHOUT any lock
void saveBufferToFlash() {
    if (!_fsReady || bufCount() == 0) return;

    // Step 1: Copy entries under mutex
    String copies[50];
    int copyCount = 0;

    if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        int idx = _bufferTail;
        int total = bufCount();
        for (int i = 0; i < total && copyCount < 50; i++) {
            if (_sendBuffer[idx].used && _sendBuffer[idx].json.length() > 10) {
                copies[copyCount++] = _sendBuffer[idx].json;
            }
            idx = (idx + 1) % MAX_BUFFER_SIZE;
        }
        xSemaphoreGive(_bufMutex);
    } else return;

    // Step 2: Write file WITHOUT any lock (LittleFS needs interrupts)
    File f = LittleFS.open(BUFFER_TMP, "w");
    if (!f) return;
    f.print("[");
    for (int i = 0; i < copyCount; i++) {
        if (i > 0) f.print(",");
        f.print(copies[i]);
        if (i % 10 == 0) feedWatchdog();
    }
    f.print("]");
    f.close();

    if (LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
    LittleFS.rename(BUFFER_TMP, BUFFER_FILE);
}

// ====================================================================
// OFFLINE STORAGE — NO locks around I/O, spinlocks only for variables
// ====================================================================

void writeOfflineHeader(const String& deviceId) {
    // Prepare header OUTSIDE any lock
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

    // File I/O — NO spinlock (LittleFS needs interrupts)
    File f = LittleFS.open(OFFLINE_FILE, "w");
    if (!f) { Serial.println("[OFFLINE] Failed to create file"); return; }
    f.write((uint8_t*)&hdr, sizeof(hdr));
    f.close();

    _offlineFileSize = sizeof(hdr);
    _offlineReadingsStored = 0;
    _offlineBlockCount = 0;
}

// Flush block: copy data under spinlock, compress + write OUTSIDE lock
void flushOfflineBlock() {
    // Step 1: Snapshot block data under spinlock (fast, no I/O)
    uint8_t localBuf[OFFLINE_BLOCK_RAW_SIZE];
    int localReadings;
    int localRawSize;

    portENTER_CRITICAL(&_blockMux);
    if (_blockReadings == 0) { portEXIT_CRITICAL(&_blockMux); return; }
    localReadings = _blockReadings;
    localRawSize = _blockReadings * 12;
    memcpy(localBuf, _blockBuf, localRawSize);
    _blockReadings = 0;
    _blockIdx = 0;
    portEXIT_CRITICAL(&_blockMux);

    // Step 2: Compress OUTSIDE lock (CPU-intensive but no I/O)
    if (!_fsReady) return;
    uint8_t compressed[1024];
    size_t compLen = tdefl_compress_mem_to_mem(compressed, sizeof(compressed),
                                               localBuf, localRawSize,
                                               TDEFL_DEFAULT_MAX_PROBES);

    // Step 3: Write to file with CRC16 for power-loss detection
    // Block format: [uint16_t size_flags] [uint16_t crc16] [data...]
    //   Bit 15 of size_flags: 1=uncompressed, 0=compressed
    //   CRC16 covers the data bytes only
    File f = LittleFS.open(OFFLINE_FILE, "a");
    if (!f) return;

    if (compLen == 0 || compLen == (size_t)-1) {
        // Uncompressed fallback
        uint16_t sz = localRawSize | 0x8000;
        uint16_t crc = 0xFFFF;
        for (int i = 0; i < localRawSize; i++) crc = (crc >> 8) ^ ((crc ^ localBuf[i]) & 0xFF) * 0x1021;
        f.write((uint8_t*)&sz, 2);
        f.write((uint8_t*)&crc, 2);
        f.write(localBuf, localRawSize);
        _offlineFileSize += 4 + localRawSize;
        compLen = localRawSize;
    } else {
        uint16_t sz = (uint16_t)compLen;
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < compLen; i++) crc = (crc >> 8) ^ ((crc ^ compressed[i]) & 0xFF) * 0x1021;
        f.write((uint8_t*)&sz, 2);
        f.write((uint8_t*)&crc, 2);
        f.write(compressed, compLen);
        _offlineFileSize += 4 + compLen;
    }
    f.close();

    _offlineReadingsStored += localReadings;
    _offlineBlockCount++;

    float ratio = (float)(localRawSize) / (compLen > 0 ? compLen : 1);
    Serial.printf("[OFFLINE] Blk%u: %d rdgs, %d->%u bytes (%.1fx), %uKB total\n",
        _offlineBlockCount, localReadings, localRawSize, (unsigned)compLen, ratio,
        _offlineFileSize / 1024);
}

// Store one reading: spinlock only for array access, flush outside lock
void storeOfflineReading(CTReading readings[6]) {
    if (!_fsReady) return;
    if (_offlineFileSize >= OFFLINE_MAX_BYTES) {
        static bool warned = false;
        if (!warned) { Serial.println("[OFFLINE] Flash full"); warned = true; }
        return;
    }

    bool shouldFlush = false;

    portENTER_CRITICAL(&_blockMux);
    for (int i = 0; i < 6; i++) {
        int16_t a = (int16_t)(readings[i].amps * 1000.0f);
        memcpy(&_blockBuf[_blockIdx], &a, 2);
        _blockIdx += 2;
    }
    _blockReadings++;
    shouldFlush = (_blockReadings >= OFFLINE_BLOCK_READINGS);
    portEXIT_CRITICAL(&_blockMux);

    // Flush OUTSIDE spinlock (does I/O)
    if (shouldFlush) {
        flushOfflineBlock();
    }
}

void enterOfflineMode(const String& deviceId) {
    portENTER_CRITICAL(&_offlineMux);
    if (_offlineMode) { portEXIT_CRITICAL(&_offlineMux); return; }
    _offlineMode = true;
    portEXIT_CRITICAL(&_offlineMux);

    portENTER_CRITICAL(&_blockMux);
    _blockReadings = 0;
    _blockIdx = 0;
    portEXIT_CRITICAL(&_blockMux);

    saveBufferToFlash();  // No lock issues — uses mutex internally

    if (!LittleFS.exists(OFFLINE_FILE)) {
        writeOfflineHeader(deviceId);  // No lock — file I/O is safe
    } else {
        File f = LittleFS.open(OFFLINE_FILE, "r");
        if (f) { _offlineFileSize = f.size(); f.close(); }
    }

    Serial.printf("[OFFLINE] Entered — 1Hz compressed blocks, %uKB stored\n", _offlineFileSize / 1024);
}

void exitOfflineMode() {
    portENTER_CRITICAL(&_offlineMux);
    if (!_offlineMode) { portEXIT_CRITICAL(&_offlineMux); return; }
    portEXIT_CRITICAL(&_offlineMux);

    flushOfflineBlock();  // Flush remaining — handles its own lock

    portENTER_CRITICAL(&_offlineMux);
    _offlineMode = false;
    portEXIT_CRITICAL(&_offlineMux);

    _wifiDownSince = 0;
    _uploadPending = true;
    _lastUploadAttempt = 0;

    Serial.printf("[OFFLINE] Exited — %u readings in %uKB, upload pending\n",
        _offlineReadingsStored, _offlineFileSize / 1024);
}

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
    f.close();

    Serial.printf("[UPLOAD] Sending %u bytes...\n", fileSize);
    feedWatchdog();

    File uf = LittleFS.open(OFFLINE_FILE, "r");
    if (!uf) { _uploadPending = false; return true; }

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(_bulkUploadUrl);
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("X-Device-Id", deviceId);
    http.addHeader("X-Format", "ls01-blocked");
    http.addHeader("Content-Length", String(fileSize));
    http.setTimeout(30000);

    int httpCode = http.sendRequest("POST", &uf, fileSize);
    uf.close();
    http.end();

    if (httpCode == 200) {
        Serial.printf("[UPLOAD] OK! %u readings delivered\n", _offlineReadingsStored);
        LittleFS.remove(OFFLINE_FILE);
        _uploadPending = false;
        _offlineReadingsStored = 0;
        _offlineFileSize = 0;
        _offlineBlockCount = 0;
        return true;
    } else {
        Serial.printf("[UPLOAD] Failed: HTTP %d — retry in 60s\n", httpCode);
        _lastUploadAttempt = millis();
        return false;
    }
}

// ====================================================================
// INIT
// ====================================================================
void initHTTPSender(const String& serverUrl, const String& deviceId) {
    _httpServerUrl = "http://46.224.90.187/api/data";
    _bulkUploadUrl = "http://46.224.90.187/api/data/bulk";
    _httpDeviceId = deviceId;

    for (int i = 0; i < MAX_BUFFER_SIZE; i++) _sendBuffer[i].used = false;
    _bufferCount = 0; _bufferHead = 0; _bufferTail = 0;

    if (LittleFS.begin(true)) {
        _fsReady = true;
        loadBufferFromFlash();
        if (LittleFS.exists(OFFLINE_FILE)) {
            File f = LittleFS.open(OFFLINE_FILE, "r");
            if (f) {
                _offlineFileSize = f.size();
                f.close();
                _uploadPending = true;
                Serial.printf("[FS] Offline data pending: %uKB\n", _offlineFileSize / 1024);
            }
        }
        Serial.printf("[FS] LittleFS %u/%u bytes\n", LittleFS.usedBytes(), LittleFS.totalBytes());
    } else {
        Serial.println("[FS] LittleFS failed");
    }
    Serial.printf("[HTTP] -> %s\n", _httpServerUrl.c_str());
}

// ====================================================================
// QUEUE READING
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

    if (json.length() < 200) {
        Serial.printf("[HTTP] JSON short (%d bytes) — possible truncation\n", json.length());
    }

    if (bufCount() >= MAX_BUFFER_SIZE) {
        _sendBuffer[_bufferTail].json = String();
        _sendBuffer[_bufferTail].used = false;
        _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
        bufCountDec();
        _totalDropped++;
        recordSendDrop();
    }

    _sendBuffer[_bufferHead].json = json;
    _sendBuffer[_bufferHead].used = true;
    _bufferHead = (_bufferHead + 1) % MAX_BUFFER_SIZE;
    bufCountInc();
}

// ====================================================================
// PROCESS SEND QUEUE — Clean control flow, no goto
// ====================================================================
void processSendQueue() {
    if (WiFi.status() != WL_CONNECTED) {
        if (_wifiDownSince == 0) _wifiDownSince = millis();
        return;
    }

    if (isOfflineMode()) exitOfflineMode();
    _wifiDownSince = 0;

    unsigned long now = millis();

    bool shouldSend = (bufCount() > 0) &&
        (_consecutiveFailures == 0 || (now - _lastSendAttempt) >= (unsigned long)_backoffMs);

    if (shouldSend) {
        int sent = 0;
        while (bufCount() > 0 && sent < MAX_SENDS_PER_LOOP) {
            String json;
            if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (!_sendBuffer[_bufferTail].used) { xSemaphoreGive(_bufMutex); break; }
                json = _sendBuffer[_bufferTail].json;
                xSemaphoreGive(_bufMutex);
            } else break;

            if (json.length() < 10 || json.indexOf("device_id") < 0) {
                if (_fsReady) {
                    File rf = LittleFS.open("/rejected.log", "a");
                    if (rf) { rf.println(json); rf.close(); }
                }
                if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    _sendBuffer[_bufferTail].used = false;
                    _sendBuffer[_bufferTail].json = String();
                    _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                    bufCountDec();
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
                    bufCountDec();
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
                    bufCountDec();
                    xSemaphoreGive(_bufMutex);
                }
                logError("400 — saved to /rejected.log");
                sent++;
            } else {
                _totalFailed++; _consecutiveFailures++;
                _backoffMs = min(_backoffMs * 2, (int)MAX_BACKOFF_MS);
                recordSendFailure();
                if (_consecutiveFailures == 1) {
                    char errBuf[64];
                    if (httpCode > 0) {
                        snprintf(errBuf, sizeof(errBuf), "HTTP %d", httpCode);
                    } else {
                        snprintf(errBuf, sizeof(errBuf), "HTTP err: %s", http.errorToString(httpCode).c_str());
                    }
                    logError(errBuf);
                }
                if (_consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
                    logError("HTTP stall: retry in 5s");
                    _backoffMs = 5000;
                    _consecutiveFailures = 0;
                }
                break;
            }
        }
    }

    if (_uploadPending && bufCount() == 0 && (now - _lastUploadAttempt >= OFFLINE_UPLOAD_RETRY_MS)) {
        uploadOfflineFile(_httpDeviceId);
    }

    if (_fsReady && (now - _lastBufferSave >= BUFFER_SAVE_INTERVAL_MS)) {
        _lastBufferSave = now;
        if (bufCount() > 0) saveBufferToFlash();
        else if (LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
    }
}

int getQueueSize()          { return bufCount(); }
uint32_t getTotalSent()     { return _totalSent; }
uint32_t getTotalFailed()   { return _totalFailed; }
uint32_t getTotalDropped()  { return _totalDropped; }
