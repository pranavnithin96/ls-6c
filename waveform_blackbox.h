#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <Preferences.h>
#include "config.h"

// Cross-module hooks (declared here; defined in http_sender.h / ota_updater.h /
// diagnostics.h, all in the same translation unit) — used by wbStreamLoop's
// health gates so streaming can never compete with live data or an OTA.
int  getQueueSize();
long getLastSuccessAgeS();
bool isUpdateInProgress();
void feedWatchdog();

// ============================================================================
// Waveform black box (2.14.0)
//
// A rolling ring of RAW 1kHz ADC samples for ONE channel — the strongest
// channel, as chosen by readAllCT each window. Streaming raw samples
// continuously is impossible (1kHz x 6ch would dwarf the live payload), but a
// frozen 12-second snapshot around an interesting moment costs one 24KB upload
// and gives millisecond-grade ground truth: what the rectified waveform
// actually looks like, what a crash spike looks like raw, what the inter-part
// gaps on the EQ machines really are.
//
// Capture is COMMAND-TRIGGERED only (heartbeat action "capture_waveform"),
// rate-limited to one per 10 minutes. No automatic triggers in 2.14.0 — the
// first job of this tool is to collect calibration/validation snapshots on
// request, not to invent an alerting channel.
//
// Memory: the ring is heap-allocated ONCE at boot (wbInit). It cannot be
// static — 24KB of .bss overflows the ESP32's dram0 static segment by ~6KB
// (measured; the segment is much smaller than total DRAM). Boot-time heap is
// ~250KB free, so the one-time malloc is safe; if it ever fails the black box
// simply disables itself (wbFeed no-ops) rather than crashing anything.
// ============================================================================

static void wbLoadStreamPref();   // stream section, below — called from wbInit

// Task ownership. The ring is written ONLY by readAllCT's sampling loop
// (loopTask, core 1). The heartbeat command handler runs on networkTask
// (core 0), so it must NOT touch the ring — it only sets _wbCaptureRequested,
// and the actual freeze happens on loopTask between sampling windows
// (wbServiceCapture, called right after readAllCT in loop()). That also puts
// the flash write between windows instead of during one — a flash write
// stalls the other core's cache, which would distort the sampling cadence.
// The upload (file read + HTTP, no ring access) stays on networkTask.
static uint16_t* _wbRing = nullptr;   // heap, allocated once by wbInit()
static uint32_t _wbHead = 0;          // next write position
static uint32_t _wbCount = 0;         // total samples ever written (saturating view)
static int8_t   _wbChannel = -1;      // channel currently in the ring
static unsigned long _wbLastCaptureMs = 0;
static volatile bool _wbCaptureRequested = false;   // set by networkTask
static volatile bool _wbDumpPending = false;
// Stream-reader coordination (2.15.0). _wbTotal counts samples fed since the
// last ring reset; _wbGen bumps on every reset (channel switch). Both are
// written ONLY by Core 1 (wbFeed) and read by Core 0 (wbStreamLoop). Aligned
// 32-bit reads/writes are atomic on the ESP32, and the reader keeps a
// multi-thousand-sample cushion behind the writer, so no barrier is needed —
// an off-by-a-few-samples view of _wbTotal is harmless by construction.
static volatile uint32_t _wbTotal = 0;
static volatile uint8_t  _wbGen = 0;

// One-time boot allocation. Call from setup() before sampling starts.
void wbInit() {
    if (_wbRing != nullptr) return;
    _wbRing = (uint16_t*)malloc(WB_RING_SAMPLES * sizeof(uint16_t));
    if (_wbRing == nullptr)
        Serial.println("[WB] ring alloc FAILED - black box disabled");
    else
        Serial.printf("[WB] ring ready: %ds @ 1kHz (%uB)\n",
                      WB_RING_SECONDS, WB_RING_SAMPLES * 2);
    // A dump captured just before a reboot/OTA would otherwise be orphaned:
    // the pending flag lives in RAM, but the file survives on flash. Re-arm
    // the upload if one is waiting. (Safe if LittleFS isn't mounted yet —
    // exists() just returns false and the next capture overwrites the file.)
    if (LittleFS.exists(WB_DUMP_FILE)) {
        _wbDumpPending = true;
        Serial.println("[WB] found dump from before reboot - upload re-armed");
    }
    wbLoadStreamPref();        // restore the per-device streaming enable
}

// Called once per sample for the chosen channel from readAllCT's loop.
// Channel switches (strongest channel changed) restart the ring — a ring with
// mixed channels is worse than a short one.
void wbFeed(int ch, uint16_t v) {
    if (_wbRing == nullptr) return;
    if (ch != _wbChannel) {
        _wbChannel = (int8_t)ch;
        _wbHead = 0;
        _wbCount = 0;
        _wbTotal = 0;    // stream cursor resyncs via the generation bump
        _wbGen++;
    }
    _wbRing[_wbHead] = v;
    _wbHead = (_wbHead + 1) % WB_RING_SAMPLES;
    if (_wbCount < WB_RING_SAMPLES) _wbCount++;
    _wbTotal++;          // after the store — the counted sample is always in place
}

struct __attribute__((packed)) WbDumpHeader {
    char     magic[4];        // "WBD1"
    uint32_t epoch;           // device wall clock at capture (0 if unsynced)
    uint8_t  channel;         // 0-based CT index
    uint8_t  reserved;
    uint16_t window_ms;       // sampling window at capture time
    uint16_t sample_rate_hz;  // 1000
    uint16_t n_samples;       // how many samples follow
};

bool wbDumpPending() { return _wbDumpPending; }
int  wbChannel()     { return _wbChannel; }

// networkTask side: just raise the flag. loopTask does the work.
void wbRequestCapture() { _wbCaptureRequested = true; }

// Freeze the ring to flash, oldest sample first. loopTask ONLY (see above).
// Returns false when rate-limited, ring empty, or flash write failed.
static bool captureWaveform(int window_ms) {
    unsigned long now = millis();
    if (_wbRing == nullptr) return false;
    if (_wbLastCaptureMs != 0 && (now - _wbLastCaptureMs) < WB_MIN_CAPTURE_GAP_MS) {
        Serial.println("[WB] capture rate-limited");
        return false;
    }
    if (_wbCount == 0 || _wbChannel < 0) {
        Serial.println("[WB] ring empty");
        return false;
    }

    File f = LittleFS.open(WB_DUMP_FILE, "w");
    if (!f) { Serial.println("[WB] open failed"); return false; }

    WbDumpHeader h = {};
    memcpy(h.magic, "WBD1", 4);
    h.epoch = (uint32_t)time(nullptr);
    h.channel = (uint8_t)_wbChannel;
    h.window_ms = (uint16_t)window_ms;
    h.sample_rate_hz = 1000;
    h.n_samples = (uint16_t)_wbCount;
    f.write((const uint8_t*)&h, sizeof(h));

    // Oldest-first: when the ring is full the oldest sample is at _wbHead.
    // Written as at most TWO large writes (the ring's two contiguous
    // segments), NOT per-sample — 12,000 individual 2-byte f.write() calls
    // each pay LittleFS overhead and stalled the sampling task for seconds.
    // ESP32 is little-endian, so dumping the uint16 array raw preserves the
    // wire format exactly.
    if (_wbCount == WB_RING_SAMPLES && _wbHead != 0) {
        f.write((const uint8_t*)&_wbRing[_wbHead], (WB_RING_SAMPLES - _wbHead) * 2);
        f.write((const uint8_t*)&_wbRing[0], _wbHead * 2);
    } else {
        f.write((const uint8_t*)&_wbRing[0], _wbCount * 2);
    }
    f.close();

    _wbLastCaptureMs = now;
    _wbDumpPending = true;
    Serial.printf("[WB] captured ch%d n=%u -> %s\n", _wbChannel + 1,
                  (unsigned)_wbCount, WB_DUMP_FILE);
    return true;
}

// loopTask side: called once per second, right after readAllCT returns, so
// the freeze (and its flash write) lands BETWEEN sampling windows.
void wbServiceCapture(int window_ms) {
    if (!_wbCaptureRequested) return;
    _wbCaptureRequested = false;
    captureWaveform(window_ms);
}

// Upload the frozen dump as a raw binary POST (no base64 — a 24KB body
// streamed straight from flash, zero large allocations). Metadata rides in
// query params so the server never has to parse the body to route it.
// Called from the network side when wbDumpPending(); clears the file on 2xx.
bool wbUploadDump(const String& serverBase, const String& deviceId) {
    if (!_wbDumpPending) return false;
    File f = LittleFS.open(WB_DUMP_FILE, "r");
    if (!f) { _wbDumpPending = false; return false; }

    WbDumpHeader h;
    if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h) || memcmp(h.magic, "WBD1", 4) != 0) {
        f.close();
        LittleFS.remove(WB_DUMP_FILE);
        _wbDumpPending = false;
        return false;
    }
    f.seek(0);

    HTTPClient http;
    String url = serverBase + "/api/waveform/dump?device_id=" + deviceId +
                 "&ct=" + String((int)h.channel + 1) +
                 "&epoch=" + String((unsigned long)h.epoch) +
                 "&rate=" + String((int)h.sample_rate_hz) +
                 "&window_ms=" + String((int)h.window_ms) +
                 "&n=" + String((int)h.n_samples);
    http.begin(url);
    http.setTimeout(20000);           // 24KB over a thin 4G link needs patience
    http.addHeader("Content-Type", "application/octet-stream");
    int code = http.sendRequest("POST", &f, f.size());
    http.end();
    f.close();

    if (code >= 200 && code < 300) {
        LittleFS.remove(WB_DUMP_FILE);
        _wbDumpPending = false;
        Serial.printf("[WB] dump uploaded (%d)\n", code);
        return true;
    }
    // keep the file; retried on the next heartbeat cycle
    Serial.printf("[WB] dump upload failed (%d), will retry\n", code);
    return false;
}

// ============================================================================
// Continuous waveform streaming (2.15.0) — pilot-scale raw 1kHz export.
//
// Reads the SAME ring wbFeed fills and ships it as 3000-sample (6KB) binary
// chunks to /api/waveform/stream, ~one POST per 6s of wall time. Design rules
// inherited from the rest of the codebase:
//   - snapshot-then-act: samples are memcpy'd into a heap scratch buffer
//     first; the network never touches the ring.
//   - SPSC ownership: Core 1 writes the ring (wbFeed), Core 0 only reads
//     behind a wide cushion. No locks. The cushion argument: a chunk is only
//     copied when the writer is ≥ WB_STREAM_CHUNK_SAMPLES ahead of the copy
//     region's END-of-wrap distance (enforced by the lag cap below), and the
//     copy itself takes microseconds vs the writer's ~500 samples/second.
//   - live data first: the same gate family as the background uploads
//     (ring at/below low-water, recent live 200, no OTA in flight).
//   - LOSSY BY DESIGN: if the uplink can't keep up, the cursor jumps forward
//     and a gap appears in `off` — this is a research data hose, not the
//     zero-loss delivery pipeline. Gaps are visible (off/seq discontinuity),
//     never silent.
// ============================================================================
static bool          _wsEnabled = false;
static uint8_t*      _wsBuf = nullptr;        // heap scratch, lazily allocated
static uint32_t      _wsCursor = 0;           // samples consumed (units of _wbTotal)
static uint8_t       _wsGen = 0xFF;           // generation the cursor is valid for
static uint32_t      _wsSeq = 0;              // delivered-chunk counter (diagnostic)
static unsigned long _wsLastAttemptMs = 0;
static bool          _wsBackoff = false;
static WiFiClient    _wsClient;               // own pooled socket — never the
static HTTPClient    _wsHttp;                 // live data plane's connection
static bool          _wsHttpInit = false;

bool     wbStreamOn()  { return _wsEnabled; }
uint32_t wbStreamSeq() { return _wsSeq; }

// 2.16: streaming is STRICTLY on-request — RAM flag only, never persisted.
// Every boot starts quiet; a wfstream_on command must arrive each time the
// operator wants the hose open. A power-cycled/rebooted device can never
// resume streaming on its own (owner decision 2026-08-03).
void wbSetStream(bool on) {
    _wsEnabled = on;
    Serial.printf("[WS] streaming %s (session-only, not persisted)\n",
                  on ? "ENABLED" : "disabled");
}

static void wbLoadStreamPref() {
    // one-time cleanup of the 2.15.0 persisted flag, then always-off at boot
    Preferences p; p.begin("lscfg", false);
    if (p.isKey("wfstream")) p.remove("wfstream");
    p.end();
    _wsEnabled = false;
}

// Core 0, every networkTask cycle (WiFi already confirmed up by the caller).
// At most one chunk POST per call; all early-outs are O(1).
void wbStreamLoop() {
    if (!_wsEnabled || _wbRing == nullptr) return;
    if (isUpdateInProgress()) return;              // never compete with an OTA
    unsigned long now = millis();
    if (_wsBackoff && (now - _wsLastAttemptMs) < WB_STREAM_RETRY_MS) return;
    // Live data keeps absolute priority — same shape as the background-upload
    // gate. A demonstrably healthy uplink = a live 200 in the last 30s.
    if (getQueueSize() > BG_UPLOAD_LOWATER) return;
    long okAge = getLastSuccessAgeS();
    if (okAge < 0 || okAge > 30) return;
    if (ESP.getFreeHeap() < WB_STREAM_MIN_HEAP) return;

    // Snapshot the writer's position (single volatile reads; cushion below
    // makes exact ordering irrelevant).
    uint8_t  gen   = _wbGen;
    uint32_t total = _wbTotal;
    int8_t   ch    = _wbChannel;
    if (ch < 0) return;
    if (gen != _wsGen) {
        // Ring was reset (channel switch) — resync to the oldest valid sample.
        _wsGen = gen;
        _wsCursor = (total > WB_RING_SAMPLES) ? total - WB_RING_SAMPLES : 0;
    }
    // Lag cap: keep the copy region a full ring-minus-chunk-minus-margin away
    // from the writer's clobber point. 12000-ring, 3000-chunk, 1000-margin =>
    // max lag 8000 samples; beyond that, jump to the freshest full chunk.
    const uint32_t maxLag = WB_RING_SAMPLES - WB_STREAM_CHUNK_SAMPLES - 1000;
    if (total - _wsCursor > maxLag) _wsCursor = total - WB_STREAM_CHUNK_SAMPLES;
    if (total - _wsCursor < WB_STREAM_CHUNK_SAMPLES) return;   // chunk not ready

    if (_wsBuf == nullptr) {
        _wsBuf = (uint8_t*)malloc(WB_STREAM_CHUNK_SAMPLES * 2);
        if (_wsBuf == nullptr) {          // heap denied: disable rather than thrash
            _wsEnabled = false;
            Serial.println("[WS] scratch alloc FAILED - streaming disabled");
            return;
        }
    }

    // Copy the chunk out of the ring (≤2 contiguous segments), then re-check
    // the generation: a channel switch mid-copy means mixed data — discard.
    uint32_t start = _wsCursor % WB_RING_SAMPLES;
    uint32_t firstN = WB_RING_SAMPLES - start;
    if (firstN > WB_STREAM_CHUNK_SAMPLES) firstN = WB_STREAM_CHUNK_SAMPLES;
    memcpy(_wsBuf, &_wbRing[start], firstN * 2);
    if (firstN < WB_STREAM_CHUNK_SAMPLES)
        memcpy(_wsBuf + firstN * 2, &_wbRing[0],
               (WB_STREAM_CHUNK_SAMPLES - firstN) * 2);
    if (_wbGen != gen) return;            // ring reset mid-copy — resync next pass

    String url = String("http://46.224.90.187/api/waveform/stream?device_id=") +
                 getDeviceId() +
                 "&ct=" + String((int)ch + 1) +
                 "&seq=" + String((unsigned long)_wsSeq) +
                 "&off=" + String((unsigned long)_wsCursor) +
                 "&n=" + String(WB_STREAM_CHUNK_SAMPLES) +
                 "&window_ms=" + String(getSampleWindowMs()) +
                 "&gen=" + String((int)gen) +
                 "&epoch=" + String((unsigned long)time(nullptr));
    if (!_wsHttpInit) {
        _wsHttp.setReuse(true);           // one socket across chunks, not one per
        _wsHttp.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
        _wsHttp.setTimeout(8000);
        _wsHttpInit = true;
    }
    feedWatchdog();
    _wsHttp.begin(_wsClient, url);
    _wsHttp.addHeader("Content-Type", "application/octet-stream");
    int code = _wsHttp.POST(_wsBuf, WB_STREAM_CHUNK_SAMPLES * 2);
    _wsHttp.end();
    feedWatchdog();
    _wsLastAttemptMs = millis();

    if (code >= 200 && code < 300) {
        _wsCursor += WB_STREAM_CHUNK_SAMPLES;
        _wsSeq++;
        _wsBackoff = false;
    } else {
        // Keep the cursor (retry the same chunk after backoff); if the outage
        // outlasts the ring, the lag cap above skips forward with a visible gap.
        _wsBackoff = true;
        _wsClient.stop();                 // reaped keep-alive: next POST reconnects
        static unsigned long _wsLastErrLog = 0;
        if (millis() - _wsLastErrLog > 300000) {   // log at most every 5 min
            _wsLastErrLog = millis();
            Serial.printf("[WS] chunk POST failed (%d)\n", code);
        }
    }
}
