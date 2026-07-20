#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "config.h"
#include "ct_sensor.h"

#include <esp32/rom/miniz.h>

// Forward declarations
void logError(const String& message);
void flushOfflineBlock();               // defined below; called from writeOfflineHeader
static String moveOfflineFileToLegacy();  // defined below; called from flushOfflineBlock (chunk rotation)

// ============================================================================
// HTTP Sender v4 — SPINLOCK RULES:
//   portENTER_CRITICAL only for simple variable read/write (no I/O, no alloc)
//   FreeRTOS mutex (_bufMutex) for longer operations on buffer entries
//   File operations: never inside any lock (LittleFS needs interrupts)
// ============================================================================

// LS02: centi-amp encoding (C3) + per-block epoch/flags/crc (D4). Magic bump is
// unconditional because the scale (centi vs milli) and block layout changed; the
// server dispatches on the 4-byte BODY magic, never the X-Format header (D2),
// which a rolled-back v2.6 unit would still send as "ls01-blocked".
#define OFFLINE_MAGIC         "LS02"
#define OFFLINE_MAGIC_LEGACY  "LS01"          // v2.6 files — drained first, never appended to
#define OFFLINE_LEGACY_FILE   "/offline.legacy.dat"
#define OFFLINE_AMP_SCALE     100.0f          // centi-amps: max 327.67A >> 50.5A ADC ceiling
#define OFFLINE_BLOCK_FLAG_UNSYNCED 0x01
#define BUFFER_FILE "/buffer.json"
#define BUFFER_TMP  "/buffer.tmp"

// File header (40 bytes, packed). magic distinguishes LS01/LS02 for the server.
// file_epoch is a best-effort creation reference only; authoritative per-reading
// time comes from each block's start_epoch, plus the upload NTP reference pair
// (X-Ntp-*) for blocks flagged clock_unsynced.
struct __attribute__((packed)) OfflineHeader {
    char magic[4];
    char device_id[32];
    uint32_t file_epoch;
};

// LS02 per-block header (13 bytes, packed) precedes each block payload.
//   start_epoch  : best-effort wall-clock epoch of the block's first reading (0 if unsynced)
//   start_millis : millis() at the block's first reading — re-stamp anchor for unsynced blocks
//   flags        : bit0 = clock_unsynced (epoch unreliable); bits1-7 reserved (0)
//   size_flags   : bit15 = 1 uncompressed / 0 compressed; bits0-14 = payload byte length
//   crc16        : CRC16-CCITT over the payload bytes as stored (compressed or raw)
// The CRC also bounds the rollback-append hazard: if a rolled-back v2.6 unit appends
// LS01 blocks after LS02 ones, the decoder reads garbage as a block header and the
// CRC fails — the server stops at that offset and quarantines the tail, never drops it.
struct __attribute__((packed)) OfflineBlockHeader {
    uint32_t start_epoch;
    uint32_t start_millis;
    uint8_t  flags;
    uint16_t size_flags;
    uint16_t crc16;
};

// CRC16-CCITT (poly 0x1021, init 0xFFFF).
static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// Fixed-size buffer entries — NO heap allocation, NO fragmentation.
// Sized for the FEATURE_WAVEFORM_STATS payload (~1.1KB worst case) since the flag
// is runtime-togglable: the static ring must always fit the on-case. Off-case
// payloads are ~0.4KB (unused tail). +27KB BSS vs v2.6 (30 x 896) — bench must
// confirm heap margin stays clear of the 15KB reboot floor with the flag on.
#define JSON_BUF_SIZE 1536
struct BufferedReading {
    char json[JSON_BUF_SIZE];
    uint16_t len;
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
// Telemetry: last data-plane HTTP result + when a POST last succeeded (0 = never)
static int _lastHttpCode = 0;
static unsigned long _lastSuccessMs = 0;
// Rejected-log drain state (machinery defined below processSendQueue)
static volatile bool _rejectedDrainPending = false;
static void drainOneRejectedFile();

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
static volatile bool _offlineTestLock = false;  // Blocks probe during test

// Block accumulator
static uint8_t _blockBuf[OFFLINE_BLOCK_RAW_SIZE];
static int _blockIdx = 0;
static int _blockReadings = 0;
// Per-block timestamp metadata, captured at the block's first reading (D4)
static uint32_t _blockStartEpoch = 0;
static uint32_t _blockStartMillis = 0;
static uint8_t _blockFlags = 0;

static SemaphoreHandle_t _bufMutex = NULL;

// Flash-full warning latch — reset whenever a fresh offline file is created so a
// later capacity event in the same uptime is surfaced again.
static bool _offlineFullWarned = false;
// Cached count of /offline.rejected.N.dat files (-1 = not scanned yet). Avoids
// 1000 LittleFS.exists() calls per heartbeat; maintained by quarantine/recover.
static int _rejectedCount = -1;
// True while every block in the current offline.dat was written THIS boot session
// (the X-Anchor-Valid precondition). Cleared when boot rotation fails and an
// old-session file is inherited; set again when a fresh file is created.
static bool _anchorSameSession = true;
// Approx bytes of offline backlog on flash beyond the current file (legacy +
// quarantined .dat files). Feeds the TOTAL capacity check — with 64KB chunking
// the per-file size alone no longer reflects real usage. Scanned once at boot,
// then maintained incrementally.
static uint32_t _backlogBytes = 0;
static inline void backlogSub(uint32_t n) { _backlogBytes = (n > _backlogBytes) ? 0 : _backlogBytes - n; }

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

// Rejected-log lifecycle: 50KB active file, rotated to numbered archives when
// full, drained by the heartbeat "upload_rejected_log" command. The old behavior
// (cap reached -> silently stop writing) was active data loss on the fleet.
#define REJECTED_LOG "/rejected.log"
#define REJECTED_MAX_BYTES 50000
#define REJECTED_ARCHIVE_MAX 4        // /rejected.1.log .. /rejected.4.log (~250KB total budget)

// Rotate the ACTIVE rejected.log into a FREE archive slot. Archives are
// IMMUTABLE once created — never renamed, shifted, or overwritten; only the
// drain deletes one after a confirmed 200. That immutability is what makes the
// whole rejected-log store race-free across cores WITHOUT a lock: a concurrent
// double-rotation just means one core's rename fails (source already moved) and
// it appends to the fresh active file instead. (An earlier shift-chain design
// renamed/deleted existing archives and raced both a concurrent rotation and
// the drain's streaming — deliberately removed.)
// Returns false when no free slot exists (active file is left in place).
static bool rotateRejectedLog() {
    if (!LittleFS.exists(REJECTED_LOG)) return false;
    char dst[24];
    for (int n = 1; n <= REJECTED_ARCHIVE_MAX; n++) {
        snprintf(dst, sizeof(dst), "/rejected.%d.log", n);
        if (!LittleFS.exists(dst)) return LittleFS.rename(REJECTED_LOG, dst);
    }
    return false;
}

// Dedicated mutex for rejected-log appends (created in initHTTPSender). Using
// the ring mutex here would make Core 1's file writes starve Core 0's
// saveBufferToFlash (50ms take) whenever LittleFS pauses for garbage collection.
static SemaphoreHandle_t _rejMux = NULL;

void writeRejected(const String& json) {
    if (!_fsReady) return;
    // Serialize appends/rotation across cores (Core 0: 400/422 path; Core 1:
    // overflow flush) so lines can't interleave mid-write. Best-effort: on a
    // timeout, proceed unguarded — a possibly-garbled line beats a lost one.
    bool locked = (_rejMux && xSemaphoreTake(_rejMux, pdMS_TO_TICKS(100)) == pdTRUE);
    if (LittleFS.exists(REJECTED_LOG)) {
        File check = LittleFS.open(REJECTED_LOG, "r");
        if (check) {
            size_t sz = check.size();
            check.close();
            if (sz >= REJECTED_MAX_BYTES && !rotateRejectedLog() && sz >= 2 * REJECTED_MAX_BYTES) {
                // All archives full AND the active file hit the hard ceiling
                // (~100KB): drop, surfaced. Total budget ~300KB. Recovery:
                // operator runs upload_rejected_log to drain the archives.
                static unsigned long lastFullLog = 0;
                if (millis() - lastFullLog > 60000) {
                    lastFullLog = millis();
                    logError("rejected store full — dropping until drained");
                }
                if (locked) xSemaphoreGive(_rejMux);
                return;
            }
        }
    }
    File rf = LittleFS.open(REJECTED_LOG, "a");
    if (rf) { rf.println(json); rf.close(); }
    if (locked) xSemaphoreGive(_rejMux);
}

// Ring-full overflow handoff: queueReading (called UNDER bufferMutex) must not
// do file I/O, so it parks the overflow JSON here and Core 1 writes it to
// /rejected.log after releasing the mutex. Single producer + single consumer
// (both the Core-1 loop), so no locking needed on the slot itself.
static char _overflowJson[JSON_BUF_SIZE];
static volatile bool _overflowPending = false;

void flushOverflowReading() {
    if (!_overflowPending) return;
    writeRejected(String(_overflowJson));
    _overflowPending = false;
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
    if (deserializeJson(doc, content)) {
        // Corrupt — quarantine, never delete (zero-loss). Numbered slots so a
        // repeat corruption doesn't overwrite an earlier quarantined file.
        char dst[32];
        for (int n = 0; n < 10; n++) {
            snprintf(dst, sizeof(dst), "/buffer.corrupt.%d.json", n);
            if (!LittleFS.exists(dst)) { LittleFS.rename(BUFFER_FILE, dst); break; }
        }
        // All 10 slots taken: cap flash use the same way saveBufferToFlash would
        if (LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
        logError("buffer.json corrupt — quarantined");
        return;
    }

    JsonArray arr = doc.as<JsonArray>();
    int loaded = 0;
    for (JsonVariant v : arr) {
        if (bufCount() >= MAX_BUFFER_SIZE) break;
        size_t len = serializeJson(v, _sendBuffer[_bufferHead].json, JSON_BUF_SIZE);
        _sendBuffer[_bufferHead].len = len;
        _sendBuffer[_bufferHead].used = true;
        _bufferHead = (_bufferHead + 1) % MAX_BUFFER_SIZE;
        bufCountInc();
        loaded++;
    }
    LittleFS.remove(BUFFER_FILE);
    if (loaded > 0) Serial.printf("[BUF] Loaded %d readings from flash\n", loaded);
}

// Save entire ring buffer to flash — copies under mutex, writes unlocked
void saveBufferToFlash() {
    if (!_fsReady || bufCount() == 0) return;

    // Step 1: Snapshot indices under mutex, stream entries to file
    int snapTail, snapCount;
    if (_bufMutex && xSemaphoreTake(_bufMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        snapTail = _bufferTail;
        snapCount = bufCount();
        xSemaphoreGive(_bufMutex);
    } else return;

    // Step 2: Write ALL entries to temp file (LittleFS needs interrupts, no lock)
    File f = LittleFS.open(BUFFER_TMP, "w");
    if (!f) return;
    f.print("[");
    bool first = true;
    int idx = snapTail;
    for (int i = 0; i < snapCount; i++) {
        if (_sendBuffer[idx].used && _sendBuffer[idx].len > 10) {
            if (!first) f.print(",");
            f.write((uint8_t*)_sendBuffer[idx].json, _sendBuffer[idx].len);
            first = false;
        }
        idx = (idx + 1) % MAX_BUFFER_SIZE;
        if (i % 20 == 0) feedWatchdog();
    }
    f.print("]");
    f.close();

    if (LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
    LittleFS.rename(BUFFER_TMP, BUFFER_FILE);
}

// ====================================================================
// OFFLINE STORAGE — NO locks around I/O, spinlocks only for variables
// ====================================================================

static uint32_t _lastKnownEpoch = 0;      // Fallback timestamp
static uint32_t _lastKnownEpochMs = 0;    // millis() when it was captured

// Best-effort current epoch. Returns true if the clock is NTP-synced (epoch
// trustworthy); false otherwise (epoch is a millis-extrapolated estimate or 0).
static bool currentEpoch(uint32_t& epochOut) {
    struct tm t;
    if (getLocalTime(&t, 0)) {
        epochOut = (uint32_t)mktime(&t);
        _lastKnownEpoch = epochOut;
        _lastKnownEpochMs = millis();
        return true;
    }
    // Extrapolate by the time SINCE capture — adding whole uptime double-counted.
    epochOut = (_lastKnownEpoch > 0)
        ? (uint32_t)(_lastKnownEpoch + (millis() - _lastKnownEpochMs) / 1000) : 0;
    return false;
}

// Single-writer latch for offline.dat creation (test-and-set under _offlineMux).
static volatile bool _headerBusy = false;

// CREATE-IF-MISSING and single-writer: two cores can race to create the file
// (Core-0 enterOfflineMode vs a Core-1 flush). The loser waits for the winner,
// then finds the file present and returns. NEVER truncates an existing file —
// that closes the interleave where one core's "w" wiped a block the other core
// had just created+appended.
void writeOfflineHeader(const String& deviceId) {
    portENTER_CRITICAL(&_offlineMux);
    bool lost = _headerBusy;
    if (!lost) _headerBusy = true;
    portEXIT_CRITICAL(&_offlineMux);
    if (lost) {
        while (_headerBusy) delay(1);   // bounded — a header write is a few ms
        return;
    }
    if (LittleFS.exists(OFFLINE_FILE)) { _headerBusy = false; return; }

    OfflineHeader hdr;
    memcpy(hdr.magic, OFFLINE_MAGIC, 4);
    memset(hdr.device_id, 0, 32);
    strncpy(hdr.device_id, deviceId.c_str(), 31);

    // file_epoch is a best-effort creation reference only — never 422'd on.
    // Authoritative per-reading time is per-block (start_epoch + millis anchor).
    uint32_t ep; currentEpoch(ep);
    hdr.file_epoch = ep;

    // File I/O — NO spinlock (LittleFS needs interrupts)
    File f = LittleFS.open(OFFLINE_FILE, "w");
    if (!f) { Serial.println("[OFFLINE] Failed to create file"); _headerBusy = false; return; }
    f.write((uint8_t*)&hdr, sizeof(hdr));
    f.close();

    _offlineFileSize = sizeof(hdr);
    _offlineReadingsStored = 0;
    _offlineBlockCount = 0;
    _offlineFullWarned = false;
    _anchorSameSession = true;   // fresh file — all blocks will be this session's
    _headerBusy = false;         // release BEFORE the flush below (it may re-enter)

    // Zero-loss: a residual RAM block (e.g. readings stored via the Core-1
    // mutex-timeout fallback before this file existed) is flushed into the fresh
    // file rather than discarded — its block header carries its own epoch/millis,
    // so timestamps stay correct regardless of which file it lands in.
    flushOfflineBlock();
}

// Flush block: copy data under spinlock, compress + write OUTSIDE lock
void flushOfflineBlock() {
    // Step 1: Snapshot block data under spinlock (fast, no I/O)
    uint8_t localBuf[OFFLINE_BLOCK_RAW_SIZE];
    int localReadings;
    int localRawSize;

    uint32_t bEpoch; uint32_t bMillis; uint8_t bFlags;
    portENTER_CRITICAL(&_blockMux);
    if (_blockReadings == 0) { portEXIT_CRITICAL(&_blockMux); return; }
    localReadings = _blockReadings;
    localRawSize = _blockReadings * 12;
    memcpy(localBuf, _blockBuf, localRawSize);
    bEpoch = _blockStartEpoch; bMillis = _blockStartMillis; bFlags = _blockFlags;
    _blockReadings = 0;
    _blockIdx = 0;
    portEXIT_CRITICAL(&_blockMux);

    // Step 2: Compress OUTSIDE lock (CPU-intensive but no I/O)
    if (!_fsReady) return;
    uint8_t compressed[1024];
    size_t compLen = tdefl_compress_mem_to_mem(compressed, sizeof(compressed),
                                               localBuf, localRawSize,
                                               TDEFL_DEFAULT_MAX_PROBES);

    // Step 3: Write LS02 block — [OfflineBlockHeader][payload]. CRC16 over payload.
    OfflineBlockHeader bh;
    bh.start_epoch = bEpoch;
    bh.start_millis = bMillis;
    bh.flags = bFlags;

    const uint8_t* payload; size_t payloadLen;
    if (compLen == 0 || compLen == (size_t)-1) {
        bh.size_flags = (uint16_t)(localRawSize) | 0x8000;   // bit15 = uncompressed
        payload = localBuf; payloadLen = localRawSize;
    } else {
        bh.size_flags = (uint16_t)compLen;                   // compressed
        payload = compressed; payloadLen = compLen;
    }
    bh.crc16 = crc16_ccitt(payload, payloadLen);

    // Never create a headerless file: the Core-1 mutex-timeout fallback can flush
    // before enterOfflineMode has written any header. Safe mutual call: our block
    // is already snapshotted+reset above, so the flushOfflineBlock call at the end
    // of writeOfflineHeader sees an empty accumulator and no-ops (depth 2, ends).
    if (!LittleFS.exists(OFFLINE_FILE)) writeOfflineHeader(_httpDeviceId);
    File f = LittleFS.open(OFFLINE_FILE, "a");
    if (!f) { logError("offline block write failed — file open"); return; }
    size_t w1 = f.write((uint8_t*)&bh, sizeof(bh));
    size_t w2 = f.write(payload, payloadLen);
    f.close();
    if (w1 != sizeof(bh) || w2 != payloadLen) {
        // Short write (flash full/error). Earlier blocks in the file are safe
        // (the server's CRC walk stops at the truncated block and quarantines
        // the tail), but THIS block's readings are lost — they were already
        // snapshotted out of RAM and a partial deflate stream is unrecoverable.
        // The 32KB free-space guard exists to pre-empt ever getting here.
        logError("offline block write short — flash full? (block lost)");
    }
    _offlineFileSize += w1 + w2;   // honest accounting, not assumed

    _offlineReadingsStored += localReadings;
    _offlineBlockCount++;

    float ratio = (float)(localRawSize) / (payloadLen > 0 ? payloadLen : 1);
    Serial.printf("[OFFLINE] Blk%u: %d rdgs, %d->%u bytes (%.1fx)%s, %uKB total\n",
        _offlineBlockCount, localReadings, localRawSize, (unsigned)payloadLen, ratio,
        (bFlags & OFFLINE_BLOCK_FLAG_UNSYNCED) ? " UNSYNCED" : "",
        _offlineFileSize / 1024);

    // Chunked rotation (poor-WiFi resumability): cap each file at 64KB so an
    // upload is a short burst a spotty link can finish — a dying 550KB stream
    // restarts from zero; delivered 64KB chunks are permanent progress. The next
    // flush recreates a fresh header automatically. Chunks upload as legacy slots
    // (X-Anchor-Valid 0 — conservative: their unsynced blocks stage, not re-stamp).
    if (_offlineFileSize >= OFFLINE_CHUNK_BYTES) {
        String moved = moveOfflineFileToLegacy();
        if (moved.length() > 0) {
            _backlogBytes += _offlineFileSize;
            _offlineFileSize = 0;
            _uploadPending = true;
            Serial.printf("[OFFLINE] Chunk rotated -> %s\n", moved.c_str());
        }
        // rotation failure (slots full): file keeps growing to OFFLINE_MAX_BYTES — still bounded
    }
}

// Store one reading: spinlock only for array access, flush outside lock
void storeOfflineReading(CTReading readings[6]) {
    if (!_fsReady) return;
    // The per-file cap can't see legacy/rejected backlog eating the partition.
    // Check real free space every 256th reading (~4min; usedBytes() walks the FS,
    // too costly at 1Hz) so a full FS takes the documented capacity-drop path
    // instead of silently losing blocks on failed appends.
    static bool _fsNearlyFull = false;
    static uint8_t _fsCheckCtr = 0;
    if (_fsCheckCtr++ == 0) {
        _fsNearlyFull = (LittleFS.totalBytes() - LittleFS.usedBytes()) < 32768;
    }
    // Capacity is the TOTAL offline footprint (current file + rotated chunks +
    // quarantined files), not just the current file — chunking made per-file
    // size meaningless as a budget.
    if (_fsNearlyFull || (_offlineFileSize + _backlogBytes) >= OFFLINE_MAX_BYTES) {
        // Over-capacity: this drops the NEWEST readings (can't rewrite an append-only
        // compressed file cheaply). True zero-loss past flash capacity is physically
        // impossible; see the buffer-budget note. Surface it to the heartbeat.
        if (!_offlineFullWarned) {
            Serial.println("[OFFLINE] Flash full — readings dropping");
            logError("OFFLINE flash full — capacity exceeded, dropping");
            _offlineFullWarned = true;
        }
        return;
    }

    // Block-start metadata (D4): epoch/millis are computed OUTSIDE the spinlock
    // (getLocalTime/mktime must not run inside portENTER_CRITICAL) but ASSIGNED
    // inside the same critical section as the append — a check-then-set across
    // two lock windows raced with flushOfflineBlock() resetting the block from
    // Core 0 (exitOfflineMode/flushBeforeRestart), leaving a new block with the
    // previous block's timestamps. Cost: one currentEpoch() per 1Hz reading.
    uint32_t ep; bool synced = currentEpoch(ep);
    uint32_t nowMs = millis();

    bool shouldFlush = false;
    portENTER_CRITICAL(&_blockMux);
    if (_blockReadings == 0) {
        _blockStartEpoch = ep;
        _blockStartMillis = nowMs;
        _blockFlags = synced ? 0 : OFFLINE_BLOCK_FLAG_UNSYNCED;
    }
    for (int i = 0; i < 6; i++) {
        // C3: centi-amps (amps*100) in int16 — max 327.67A, no overflow at the
        // 50.5A ADC ceiling; 10mA step is below the ADC's 12.3mA/count resolution.
        int16_t a = (int16_t)lroundf(readings[i].amps * OFFLINE_AMP_SCALE);
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

// Move OFFLINE_FILE into the oldest-first upload queue (legacy slots). Falls back
// to numbered slots when the primary is taken — NEVER leaves the caller free to
// truncate an un-migrated file (zero-loss). Returns the destination slot path, or
// "" when all 99 slots are occupied / rename failed (file left in place untouched).
static String moveOfflineFileToLegacy() {
    if (!LittleFS.exists(OFFLINE_FILE)) return String("");
    if (!LittleFS.exists(OFFLINE_LEGACY_FILE)) {
        return LittleFS.rename(OFFLINE_FILE, OFFLINE_LEGACY_FILE) ? String(OFFLINE_LEGACY_FILE) : String("");
    }
    char alt[48];
    for (int n = 1; n < 100; n++) {
        snprintf(alt, sizeof(alt), "/offline.legacy.%d.dat", n);
        if (!LittleFS.exists(alt)) return LittleFS.rename(OFFLINE_FILE, alt) ? String(alt) : String("");
    }
    logError("legacy slots full — offline file left in place");
    return String("");
}

// Called from Core 1 — must be FAST. No saveBufferToFlash here.
void enterOfflineMode(const String& deviceId) {
    portENTER_CRITICAL(&_offlineMux);
    if (_offlineMode) { portEXIT_CRITICAL(&_offlineMux); return; }
    _offlineMode = true;
    portEXIT_CRITICAL(&_offlineMux);

    // Only init the offline file header — fast single write
    if (!LittleFS.exists(OFFLINE_FILE)) {
        writeOfflineHeader(deviceId);
    } else {
        // Defensive: never append LS02 blocks to a non-LS02 file (boot migration
        // should have moved any legacy file already, but guard the race anyway).
        char magic[4] = {0};
        File f = LittleFS.open(OFFLINE_FILE, "r");
        if (f) { f.read((uint8_t*)magic, 4); _offlineFileSize = f.size(); f.close(); }
        if (memcmp(magic, OFFLINE_MAGIC, 4) != 0) {
            // Only create a fresh file once the old one is safely out of the way —
            // writeOfflineHeader opens with "w" and would truncate it otherwise.
            // If all legacy slots are full (pathological), leave the foreign file:
            // appended LS02 blocks fail its CRC walk server-side and the tail is
            // quarantined there — degraded, but nothing stored is destroyed here.
            if (moveOfflineFileToLegacy().length() > 0) {
                writeOfflineHeader(deviceId);   // resets _offlineFileSize
            }
            _uploadPending = true;
        }
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

// Pick the next offline file to upload: legacy (oldest) first, then numbered
// legacy, then the current LS02 file. Returns "" when nothing is pending.
static String nextOfflineUploadTarget() {
    if (LittleFS.exists(OFFLINE_LEGACY_FILE)) return OFFLINE_LEGACY_FILE;
    char alt[48];
    for (int n = 1; n < 100; n++) {
        snprintf(alt, sizeof(alt), "/offline.legacy.%d.dat", n);
        if (LittleFS.exists(alt)) return String(alt);
    }
    if (LittleFS.exists(OFFLINE_FILE)) return OFFLINE_FILE;
    return String("");
}

// Move a file to the next free numbered rejected slot — NEVER overwrite (zero-loss).
// Quarantined files are kept on flash for later recovery (a "recover_rejected"
// command renames them back to a legacy slot); they are never silently dropped.
static bool quarantineOfflineFile(const String& path) {
    char dst[48];
    for (int n = 0; n < 1000; n++) {
        snprintf(dst, sizeof(dst), "/offline.rejected.%d.dat", n);
        if (!LittleFS.exists(dst)) {
            bool ok = LittleFS.rename(path.c_str(), dst);
            if (ok && _rejectedCount >= 0) _rejectedCount++;
            Serial.printf("[UPLOAD] Quarantined %s -> %s (kept, not deleted)\n", path.c_str(), dst);
            logError("offline file quarantined — kept for recovery, NOT deleted");
            return ok;
        }
    }
    logError("rejected slots full — leaving offline file in place");
    return false;   // do NOT delete; leave the file where it is
}

// Re-stamp tracking: a file that 422s gets one NTP-resync + retry before quarantine.
static bool _offline422Retried = false;
// Legacy slot holding a file rotated at UPLOAD time this session — its blocks'
// millis anchors are still valid (unlike boot-rotated slots). RAM-only: a reboot
// correctly demotes it to anchor-stale. Cleared when the file is removed/renamed.
static String _sessionLegacyPath = "";

bool uploadOfflineFile(const String& deviceId) {
    if (!_fsReady) { _uploadPending = false; return true; }

    String target = nextOfflineUploadTarget();
    if (target.length() == 0) { _uploadPending = false; return true; }

    File chk = LittleFS.open(target, "r");
    // Transient open failure — keep pending, but stamp the attempt so a
    // persistently unopenable file retries at 60s pace, not every 100ms tick.
    if (!chk) { _lastUploadAttempt = millis(); return false; }
    size_t fileSize = chk.size();
    chk.close();

    if (fileSize <= sizeof(OfflineHeader)) {
        // Header-only/empty — no readings in ANY slot, safe to remove. (Quarantining
        // legacy ones just ping-pongs empty files through recover_rejected forever.)
        LittleFS.remove(target);
        if (target == OFFLINE_FILE) _offlineFileSize = 0;
        else backlogSub((uint32_t)fileSize);
        if (target == _sessionLegacyPath) _sessionLegacyPath = "";
        return false;   // re-evaluate remaining files next tick
    }

    if (target == OFFLINE_FILE) {
        // Rotate before streaming so the uploaded file is IMMUTABLE. Streaming
        // offline.dat directly raced a concurrent Core-1 flush: blocks appended
        // during the (up to 30s) POST were beyond the streamed length, and the
        // 200-path remove() destroyed them unsent. After rotation, a concurrent
        // flush creates a fresh headered offline.dat this upload never touches.
        String moved = moveOfflineFileToLegacy();
        if (moved.length() == 0) { _lastUploadAttempt = millis(); return false; }
        _sessionLegacyPath = moved;   // written this session — anchors still valid
        _offlineReadingsStored = 0; _offlineFileSize = 0; _offlineBlockCount = 0;
        target = moved;
        // Re-stat: appends may have landed between the size read and the rename;
        // the rotated file can no longer grow, so this size is final.
        File rs = LittleFS.open(target, "r");
        if (!rs) { _lastUploadAttempt = millis(); return false; }
        fileSize = rs.size();
        rs.close();
        _backlogBytes += (uint32_t)fileSize;   // now part of the backlog until delivered
    }

    Serial.printf("[UPLOAD] Sending %s (%u bytes)...\n", target.c_str(), (unsigned)fileSize);
    feedWatchdog();

    File uf = LittleFS.open(target, "r");
    if (!uf) { _lastUploadAttempt = millis(); return false; }

    // Upload-time NTP reference pair — lets the server re-stamp clock_unsynced
    // blocks: boot_epoch = X-Ntp-Epoch - X-Ntp-Millis/1000;
    // reading_epoch = boot_epoch + block.start_millis/1000 + reading_index.
    uint32_t ntpEpoch; bool ntpValid = currentEpoch(ntpEpoch);

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(_bulkUploadUrl);
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("X-Device-Id", deviceId);
    http.addHeader("X-Format", "ls02-blocked");           // advisory only; server dispatches on body magic
    http.addHeader("X-Ntp-Epoch", String(ntpEpoch));
    http.addHeader("X-Ntp-Millis", String(millis()));
    http.addHeader("X-Ntp-Valid", ntpValid ? "1" : "0");
    // Anchor validity: the millis-based re-stamp formula only holds when the
    // blocks were written THIS boot session — i.e. only for the slot we rotated
    // at upload time above. Boot-rotated/recovered files carry start_millis from
    // an earlier session and must be staged, not re-stamped.
    bool sameSession = (target == _sessionLegacyPath) && _anchorSameSession;
    http.addHeader("X-Anchor-Valid", (sameSession && ntpValid) ? "1" : "0");
    // Positional stamping is interval-relative, not 1Hz: readings within a block
    // are getSendInterval() seconds apart (runtime-configurable 1-60s via portal
    // or heartbeat set_interval). Decoder multiplies the index by this. Best-effort:
    // exact unless the interval changed while this file was being written.
    http.addHeader("X-Interval", String(getSendInterval()));
    // NO manual Content-Length: sendRequest(type, Stream*, size) emits its own,
    // and addHeader doesn't dedupe it — two Content-Length headers get the whole
    // upload 400'd by strict proxies (request-smuggling defense) → quarantine loop.
    http.setTimeout(30000);

    int httpCode = http.sendRequest("POST", &uf, fileSize);
    uf.close();
    http.end();
    _lastUploadAttempt = millis();
    _lastHttpCode = httpCode;
    if (httpCode == 200) _lastSuccessMs = millis();

    if (httpCode == 200) {
        Serial.printf("[UPLOAD] OK! %s delivered\n", target.c_str());
        LittleFS.remove(target);
        backlogSub((uint32_t)fileSize);
        if (target == _sessionLegacyPath) _sessionLegacyPath = "";
        _offline422Retried = false;
        bool more = (nextOfflineUploadTarget().length() > 0);
        _uploadPending = more;
        // Server just took a file fine — drain the rest on the next tick instead
        // of one file per OFFLINE_UPLOAD_RETRY_MS (a rotated backlog would take
        // minutes otherwise). Failures below keep the 60s pacing.
        if (more) _lastUploadAttempt = 0;
        return !more;
    }

    if (httpCode == 422) {
        // C2: NEVER delete on 4xx. Re-stamp == resync NTP + retry once with a fresh
        // X-Ntp-* reference. Still 422 -> quarantine the whole file (kept, not dropped).
        if (!_offline422Retried) {
            logError("422 on offline upload — syncing NTP, retry once");
            syncNTP();
            _offline422Retried = true;
            return false;
        }
        logError("422 again — quarantining offline file (not deleting)");
        quarantineOfflineFile(target);
        if (target == _sessionLegacyPath) _sessionLegacyPath = "";
        _offline422Retried = false;
        return false;
    }

    if (httpCode == 400) {
        // Malformed per server. Do NOT delete (C2) — quarantine for recovery.
        logError("400 on offline upload — quarantining (not deleting)");
        quarantineOfflineFile(target);
        if (target == _sessionLegacyPath) _sessionLegacyPath = "";
        return false;
    }

    // Transport error / 5xx — keep the file, retry later.
    Serial.printf("[UPLOAD] Failed: HTTP %d — retry in 60s\n", httpCode);
    return false;
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
    if (!_rejMux) _rejMux = xSemaphoreCreateMutex();   // before any writeRejected is possible

    if (LittleFS.begin(true)) {
        _fsReady = true;
        loadBufferFromFlash();

        // Rotate ANY pre-boot offline.dat to a legacy slot at boot — LS01 and LS02
        // alike. LS01: never append LS02 blocks to it. LS02: its blocks'
        // start_millis anchors are only valid within the boot session that wrote
        // them, so the current file must never span sessions (X-Anchor-Valid
        // contract — the server only re-stamps unsynced blocks when the upload's
        // NTP reference pair is from the same session as the blocks).
        // uploadOfflineFile drains legacy slots first; server dispatches on body magic.
        if (LittleFS.exists(OFFLINE_FILE)) {
            if (moveOfflineFileToLegacy().length() > 0) {
                Serial.println("[FS] Rotated pre-boot offline.dat -> legacy slot");
            } else {
                // Slots full — old-session file stays current. Its blocks' millis
                // anchors are stale, so its upload must never claim anchor validity.
                _anchorSameSession = false;
            }
        }
        // One-time backlog scan: sum legacy + quarantined offline files for the
        // total-capacity check, and pre-warm the rejected-count cache so the
        // first heartbeat never pays the 1000-exists scan.
        {
            char p[48]; int rc = 0; uint32_t bl = 0;
            for (int n = 0; n < 100; n++) {
                snprintf(p, sizeof(p), n == 0 ? OFFLINE_LEGACY_FILE : "/offline.legacy.%d.dat", n);
                if (!LittleFS.exists(p)) continue;
                File bf = LittleFS.open(p, "r");
                if (bf) { bl += bf.size(); bf.close(); }
            }
            for (int n = 0; n < 1000; n++) {
                snprintf(p, sizeof(p), "/offline.rejected.%d.dat", n);
                if (!LittleFS.exists(p)) continue;
                rc++;
                File bf = LittleFS.open(p, "r");
                if (bf) { bl += bf.size(); bf.close(); }
            }
            _backlogBytes = bl;
            _rejectedCount = rc;
            if (bl > 0) Serial.printf("[FS] Offline backlog: %uKB across legacy+rejected slots\n", bl / 1024);
        }
        // Pending if any legacy/rotated file remains
        if (nextOfflineUploadTarget().length() > 0) {
            _uploadPending = true;
            Serial.println("[FS] Offline data pending (legacy queue)");
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

    // clock_unsynced (D5): tell the server this reading's timestamp is pre-NTP so
    // it stages/re-stamps instead of 422'ing. Server must never 4xx a flagged row.
    struct tm _tnow;
    if (!getLocalTime(&_tnow, 0)) doc["clock_unsynced"] = true;

    const bool wf = waveformStatsEnabled();   // cached RAM read, no NVS in this path

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
        // Half-B waveform features — LIVE payload only (D3), flag-gated. Off => byte-identical to v2.6.
        if (wf) {
            ct["peak_amps"]      = serialized(String(readings[i].peak_amps, 1));
            ct["env_peak_ratio"] = serialized(String(readings[i].env_peak_ratio, 2));
            ct["ripple_amps"]    = serialized(String(readings[i].ripple_amps, 2));
            JsonArray e5 = ct["env5"].to<JsonArray>();
            for (int k = 0; k < 5; k++) e5.add(serialized(String(readings[i].env5[k], 1)));
        }
    }
    doc["readings"]["voltage_rms"] = serialized(String(gridVoltage, 1));

    // Serialize to fixed stack buffer — avoids heap fragmentation from String growth
    char jsonBuf[1536];
    size_t jsonLen = serializeJson(doc, jsonBuf, sizeof(jsonBuf));
    if (jsonLen < 200 || jsonLen >= sizeof(jsonBuf)) {
        Serial.printf("[HTTP] JSON bad: %u bytes (heap:%u)\n", jsonLen, ESP.getFreeHeap());
        return;  // Don't queue garbage
    }


    if (bufCount() >= MAX_BUFFER_SIZE) {
        // Ring full (send stall — only happens in the window before offline mode
        // kicks in). NEVER advance _bufferTail from here: the ring is SPSC and the
        // tail belongs to Core 0, which may be mid-POST on that exact entry.
        // Zero-loss: park the reading for flushOverflowReading() — the caller
        // holds bufferMutex, so no file I/O may happen here (it starved Core 0's
        // saveBufferToFlash and broke the <10ms hold invariant). The slot can't
        // be clobbered: the same Core-1 loop iteration always drains it first.
        memcpy(_overflowJson, jsonBuf, jsonLen + 1);
        _overflowPending = true;
        _totalDropped++;
        recordSendDrop();
        static unsigned long _lastDropLog = 0;
        if (millis() - _lastDropLog > 60000) {
            _lastDropLog = millis();
            logError("send ring full — reading saved to /rejected.log");
        }
        return;
    }

    memcpy(_sendBuffer[_bufferHead].json, jsonBuf, jsonLen + 1);
    _sendBuffer[_bufferHead].len = jsonLen;
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
    _wifiDownSince = 0;

    // If in offline mode (server unreachable), probe every 10s
    if (isOfflineMode()) {
        if (_offlineTestLock) return;  // Test in progress — don't probe
        unsigned long now = millis();
        if (now - _lastSendAttempt < 10000) return;
        // Probe with a minimal POST
        HTTPClient http;
        http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
        http.begin(_httpServerUrl);
        http.addHeader("Content-Type", "application/json");
        http.setTimeout(HTTP_TIMEOUT_MS);
        int code = http.POST("{}");
        http.end();
        _lastSendAttempt = millis();
        if (code > 0) {  // Server responded (even 400 means it's reachable)
            Serial.printf("[HTTP] Server back (HTTP %d) — exiting offline mode\n", code);
            _consecutiveFailures = 0;
            _backoffMs = 1000;
            // C1: re-queue the stall-time RAM ring persisted to buffer.json. Those
            // readings predate the offline.dat blocks, so sending them first (the
            // upload is gated on bufCount()==0) preserves oldest-first ordering.
            // MUST run BEFORE exitOfflineMode(): while the offline flag is up,
            // Core 1 routes readings to offline storage and never touches the
            // ring, so this unmutexed write to _bufferHead runs uncontended. The
            // moment the flag drops, Core 1 resumes queueReading() at the head.
            loadBufferFromFlash();
            exitOfflineMode();
        }
        return;
    }

    unsigned long now = millis();

    bool shouldSend = (bufCount() > 0) &&
        (_consecutiveFailures == 0 || (now - _lastSendAttempt) >= (unsigned long)_backoffMs);

    if (shouldSend) {
        int sent = 0;
        while (bufCount() > 0 && sent < MAX_SENDS_PER_LOOP) {
            // Read from tail — no String allocation, just pointer to fixed buffer
            if (!_sendBuffer[_bufferTail].used || _sendBuffer[_bufferTail].len < 10) {
                _sendBuffer[_bufferTail].used = false;
                _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                bufCountDec();
                continue;
            }

            HTTPClient http;
            http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
            http.begin(_httpServerUrl);
            http.addHeader("Content-Type", "application/json");
            http.setTimeout(HTTP_TIMEOUT_MS);
            int httpCode = http.POST((uint8_t*)_sendBuffer[_bufferTail].json, _sendBuffer[_bufferTail].len);
            http.end();
            _lastSendAttempt = millis();
            _lastHttpCode = httpCode;

            if (httpCode == 200) {
                _lastSuccessMs = millis();
                _totalSent++; _consecutiveFailures = 0; _backoffMs = 1000;
                recordSendSuccess();
                // Keep last known epoch fresh for offline fallback
                struct tm t; if (getLocalTime(&t, 0)) _lastKnownEpoch = mktime(&t);
                _sendBuffer[_bufferTail].used = false;
                _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                bufCountDec();
                sent++;
            } else if (httpCode == 400) {
                writeRejected(String(_sendBuffer[_bufferTail].json));
                _sendBuffer[_bufferTail].used = false;
                _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                bufCountDec();
                logError("400 — saved to /rejected.log");
                sent++;
            } else if (httpCode == 422) {
                // 422 = bad timestamp (pre-NTP). With the clock_unsynced flag + the
                // D5 server contract this should not happen; if it does, PRESERVE the
                // reading (zero-loss) to /rejected.log instead of dropping, then resync.
                writeRejected(String(_sendBuffer[_bufferTail].json));
                logError("422 — saved to /rejected.log, triggering NTP");
                _sendBuffer[_bufferTail].used = false;
                _bufferTail = (_bufferTail + 1) % MAX_BUFFER_SIZE;
                bufCountDec();
                syncNTP();  // Try to fix the clock
                break;      // Stop sending until timestamps are valid
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
                    logError("HTTP stall: server unreachable");
                    _backoffMs = 5000;
                    _consecutiveFailures = 1;

                    if (!isOfflineMode()) {
                        saveBufferToFlash();
                        // Clear buffer (fixed arrays, no heap to free)
                        for (int i = 0; i < MAX_BUFFER_SIZE; i++) _sendBuffer[i].used = false;
                        _bufferHead = 0; _bufferTail = 0; _bufferCount = 0;
                        enterOfflineMode(_httpDeviceId);
                        Serial.printf("[HTTP] Offline mode — heap: %u\n", ESP.getFreeHeap());
                    }
                }
                break;
            }
        }
    }

    if (_uploadPending && bufCount() == 0 && (now - _lastUploadAttempt >= OFFLINE_UPLOAD_RETRY_MS)) {
        uploadOfflineFile(_httpDeviceId);
    } else if (_rejectedDrainPending && bufCount() == 0 &&
               (_lastUploadAttempt == 0 || now - _lastUploadAttempt >= OFFLINE_UPLOAD_RETRY_MS)) {
        // Rejected-log drain: one immutable file per tick, ring-empty gated —
        // offline backlog (raw readings) always takes priority via the else-if.
        drainOneRejectedFile();
    }
}

// Periodic buffer save — must be called from Core 0 regardless of WiFi state
void periodicBufferSave() {
    unsigned long now = millis();
    if (_fsReady && (now - _lastBufferSave >= BUFFER_SAVE_INTERVAL_MS)) {
        _lastBufferSave = now;
        if (bufCount() > 0) saveBufferToFlash();
        // C1: never delete buffer.json while offline or an upload is pending — it
        // holds the stall-time ring that still needs re-queuing. Only remove it when
        // truly drained (online, ring empty, nothing pending).
        else if (!isOfflineMode() && !_uploadPending && LittleFS.exists(BUFFER_FILE)) LittleFS.remove(BUFFER_FILE);
    }
}

// Call before any ESP.restart() to preserve buffered data
void flushBeforeRestart() {
    if (_fsReady && bufCount() > 0) {
        Serial.printf("[BUF] Flushing %d readings to flash before restart...\n", bufCount());
        saveBufferToFlash();
    }
    // Also flush any partial offline block
    flushOfflineBlock();
}

// Operator-triggered (heartbeat "recover_rejected" command): move quarantined
// files back into the upload queue so they're retried. Zero-loss recovery path
// for files that 422'd/400'd twice — they were kept on flash, never dropped.
void recoverRejectedFiles() {
    if (!_fsReady) return;
    char src[48]; char dst[48];
    int moved = 0;
    for (int n = 0; n < 1000; n++) {
        snprintf(src, sizeof(src), "/offline.rejected.%d.dat", n);
        if (!LittleFS.exists(src)) continue;
        bool placed = false;
        for (int m = 1; m < 100; m++) {
            snprintf(dst, sizeof(dst), "/offline.legacy.%d.dat", m);
            if (!LittleFS.exists(dst)) {
                placed = LittleFS.rename(src, dst);
                if (placed) { moved++; if (_rejectedCount > 0) _rejectedCount--; }
                break;
            }
        }
        if (!placed) break;   // no free legacy slot — stop, leave the rest quarantined
    }
    if (moved > 0) { _uploadPending = true; Serial.printf("[RECOVER] Re-queued %d rejected file(s)\n", moved); }
}

int getRejectedFileCount() {
    if (!_fsReady) return 0;
    // Scan once (boot-lazy), then serve the cached count — 1000 LittleFS.exists()
    // per 60s heartbeat stalled the Core 0 network task. quarantine/recover keep
    // the cache current; all three run on the same task, so no locking needed.
    if (_rejectedCount < 0) {
        char p[48]; int c = 0;
        for (int n = 0; n < 1000; n++) { snprintf(p, sizeof(p), "/offline.rejected.%d.dat", n); if (LittleFS.exists(p)) c++; }
        _rejectedCount = c;
    }
    return _rejectedCount;
}

int getQueueSize()          { return bufCount(); }
uint32_t getTotalSent()     { return _totalSent; }
uint32_t getTotalFailed()   { return _totalFailed; }
uint32_t getTotalDropped()  { return _totalDropped; }
int getLastHttpCode()       { return _lastHttpCode; }
uint32_t getBacklogBytes()  { return _backlogBytes; }   // rotated chunks + quarantined files
// Seconds since the last successful data POST; -1 = never this uptime
long getLastSuccessAgeS()   { return _lastSuccessMs ? (long)((millis() - _lastSuccessMs) / 1000) : -1; }
uint32_t getRejectedLogBytes() {
    if (!_fsReady) return 0;
    uint32_t total = 0;
    char p[24];
    for (int n = 0; n <= REJECTED_ARCHIVE_MAX; n++) {   // 0 = the active log
        if (n == 0) snprintf(p, sizeof(p), REJECTED_LOG);
        else snprintf(p, sizeof(p), "/rejected.%d.log", n);
        if (!LittleFS.exists(p)) continue;
        File f = LittleFS.open(p, "r");
        if (f) { total += f.size(); f.close(); }
    }
    return total;
}

// POST one NDJSON file to the bulk endpoint. 200 => delete (delivered);
// anything else => keep for the next attempt. Server contract: X-Format
// "rejected-ndjson", one live-payload JSON per line, ingest idempotently.
static bool _postRejectedFile(const char* path) {
    if (!LittleFS.exists(path)) return true;
    File f = LittleFS.open(path, "r");
    if (!f) return false;
    size_t sz = f.size();
    if (sz == 0) { f.close(); LittleFS.remove(path); return true; }

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(_bulkUploadUrl);
    http.addHeader("Content-Type", "application/x-ndjson");
    http.addHeader("X-Device-Id", _httpDeviceId);
    http.addHeader("X-Format", "rejected-ndjson");
    http.setTimeout(30000);
    int code = http.sendRequest("POST", &f, sz);
    f.close();
    http.end();
    if (code == 200) {
        Serial.printf("[REJECTED] Uploaded %s (%u bytes)\n", path, (unsigned)sz);
        LittleFS.remove(path);
        return true;
    }
    logError("rejected-log upload failed — kept");
    return false;
}

// Operator-triggered (heartbeat "upload_rejected_log"): flag the drain; the
// actual uploads happen ONE FILE PER TICK from processSendQueue, gated on an
// empty send ring. Draining everything inline blocked Core 0 for up to 150s,
// which filled the ring and MANUFACTURED new rejected entries — and streaming
// the active file raced Core-1 appends (200-remove destroyed unsent lines).
// Only immutable archives are ever streamed; the active file is rotated first.
void requestRejectedDrain() { _rejectedDrainPending = true; }
bool isRejectedDrainPending() { return _rejectedDrainPending; }

static void drainOneRejectedFile() {
    // Give up after 10 consecutive failures (server missing the endpoint, etc.)
    // so a stuck drain can't hold the scheduled-reboot gate forever. Files stay
    // on flash; the operator re-sends the command once the server is fixed.
    static int drainFails = 0;
    char p[24];
    for (int n = 1; n <= REJECTED_ARCHIVE_MAX; n++) {   // oldest archive first
        snprintf(p, sizeof(p), "/rejected.%d.log", n);
        if (!LittleFS.exists(p)) continue;
        feedWatchdog();
        // Success => next file next tick (gate reopened); failure => 60s pace
        if (_postRejectedFile(p)) {
            drainFails = 0;
            _lastUploadAttempt = 0;
        } else {
            _lastUploadAttempt = millis();
            if (++drainFails >= 10) {
                drainFails = 0;
                _rejectedDrainPending = false;
                logError("rejected drain aborted after 10 failures — files kept");
            }
        }
        return;
    }
    // No archives left: rotate the active log so next tick streams an immutable
    // copy while Core 1 appends to a fresh active file.
    if (LittleFS.exists(REJECTED_LOG) && rotateRejectedLog()) { _lastUploadAttempt = 0; return; }
    _rejectedDrainPending = false;   // nothing left to drain
}
