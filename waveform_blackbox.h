#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include "config.h"

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

// One-time boot allocation. Call from setup() before sampling starts.
void wbInit() {
    if (_wbRing != nullptr) return;
    _wbRing = (uint16_t*)malloc(WB_RING_SAMPLES * sizeof(uint16_t));
    if (_wbRing == nullptr)
        Serial.println("[WB] ring alloc FAILED - black box disabled");
    else
        Serial.printf("[WB] ring ready: %ds @ 1kHz (%uB)\n",
                      WB_RING_SECONDS, WB_RING_SAMPLES * 2);
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
    }
    _wbRing[_wbHead] = v;
    _wbHead = (_wbHead + 1) % WB_RING_SAMPLES;
    if (_wbCount < WB_RING_SAMPLES) _wbCount++;
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
    uint32_t start = (_wbCount == WB_RING_SAMPLES) ? _wbHead : 0;
    for (uint32_t i = 0; i < _wbCount; i++) {
        uint16_t v = _wbRing[(start + i) % WB_RING_SAMPLES];
        f.write((const uint8_t*)&v, 2);
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
