#pragma once

#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <sys/time.h>
#include "config.h"
#include "wfs2_protocol.h"

// Cross-module health hooks (defined in http_sender.h / ota_updater.h /
// diagnostics.h in the same Arduino translation unit).
int  getQueueSize();
long getLastSuccessAgeS();
bool isUpdateInProgress();
void feedWatchdog();

// ============================================================================
// WFS2 complete-frame waveform queue (2.18.2)
//
// Core 1 owns WRITING slots and appends configured channels in scan order.
// Core 0 owns SENDING slots and POSTs one complete frame. READY/EMPTY handoff
// is protected by a spinlock containing fixed-size state changes only. The
// network never reads a slot that the sampler can overwrite.
// ============================================================================

enum Wfs2SlotState : uint8_t {
    WFS2_EMPTY = 0,
    WFS2_WRITING,
    WFS2_READY,
    WFS2_SENDING,
};

struct Wfs2Slot {
    Wfs2Header header;
    uint8_t payload[WFS2_MAX_PAYLOAD_BYTES];
    volatile uint8_t state;
};

static_assert(offsetof(Wfs2Slot, payload) == sizeof(Wfs2Header),
              "WFS2 header and sample payload must be contiguous");

static portMUX_TYPE _wbMux = portMUX_INITIALIZER_UNLOCKED;
static Wfs2Slot* _wbSlots = nullptr;
static int8_t _wbWriteSlot = -1;               // Core 1 only
static uint32_t _wbWriteSamples = 0;            // Core 1 only
static Wfs2DeltaEncoder _wbEncoder;              // Core 1 only
static uint32_t _wbWriteCrc = 0xffffffffu;
static bool _wbWriteOnDemand = false;           // Core 1 only
static volatile bool _wbCaptureRequested = false;
static volatile bool _wbStreamEnabled = false;
static uint32_t _wbBootId = 0;
static volatile uint32_t _wbFramesGenerated = 0;
static volatile uint32_t _wbFramesDelivered = 0;
static volatile uint32_t _wbFramesDropped = 0;
static uint32_t _wbFrameSeq = 0;
static unsigned long _wbLastAttemptMs = 0;
static bool _wbBackoff = false;
static unsigned long _wbSingleReadySinceMs = 0;
static WiFiClient _wbClient;
static HTTPClient _wbHttp;
static bool _wbHttpInit = false;
static uint8_t* _wbBatchBuf = nullptr;
static size_t _wbBatchCapacity = 0;

static uint8_t _wbQueuedFramesUnlocked() {
    if (_wbSlots == nullptr) return 0;
    uint8_t n = 0;
    for (int i = 0; i < WFS2_QUEUE_DEPTH; ++i)
        if (_wbSlots[i].state == WFS2_READY || _wbSlots[i].state == WFS2_SENDING) ++n;
    return n;
}

uint8_t wbQueuedFrames() {
    portENTER_CRITICAL(&_wbMux);
    uint8_t n = _wbQueuedFramesUnlocked();
    portEXIT_CRITICAL(&_wbMux);
    return n;
}

uint32_t wbFramesGenerated() { return _wbFramesGenerated; }
uint32_t wbFramesDelivered() { return _wbFramesDelivered; }
uint32_t wbFramesDropped()   { return _wbFramesDropped; }
bool wbStreamOn()            { return _wbStreamEnabled; }
uint32_t wbStreamSeq()       { return _wbFramesDelivered; } // legacy heartbeat alias

void wbSetStream(bool on) {
    _wbStreamEnabled = on;
    Preferences p;
    p.begin("lscfg", false);
    p.putBool("wfstream", on);
    p.end();
    Serial.printf("[WFS2] streaming %s\n", on ? "ENABLED" : "disabled");
}

void wbInit() {
    if (_wbSlots != nullptr) return;
    _wbSlots = (Wfs2Slot*)calloc(WFS2_QUEUE_DEPTH, sizeof(Wfs2Slot));
    if (_wbSlots == nullptr) {
        Serial.printf("[WFS2] queue alloc FAILED (%u bytes) - waveform disabled\n",
                      (unsigned)(WFS2_QUEUE_DEPTH * sizeof(Wfs2Slot)));
        _wbStreamEnabled = false;
        return;
    }
    _wbBootId = esp_random();
    Preferences p;
    p.begin("lscfg", true);
    _wbStreamEnabled = p.getBool("wfstream", false);
    p.end();
    Serial.printf("[WFS2] queue ready: %d x %u bytes, boot=%08X, stream=%s\n",
                  WFS2_QUEUE_DEPTH, (unsigned)sizeof(Wfs2Slot),
                  (unsigned)_wbBootId, _wbStreamEnabled ? "on" : "off");
}

// Called by Core 1 immediately before the first ADC scan of a one-second frame.
// Returns false when streaming is off, no CT is active, or all queue slots are
// occupied. A full queue drops the NEW frame and increments a visible counter.
bool wbBeginFrame(uint8_t activeMask, uint32_t configRevision) {
    activeMask &= 0x3f;
    _wbWriteSlot = -1;
    _wbWriteSamples = 0;
    _wbWriteCrc = 0xffffffffu;
    if (_wbSlots == nullptr || activeMask == 0) return false;

    bool onDemand = _wbCaptureRequested;
    if (!_wbStreamEnabled && !onDemand) return false;
    uint32_t frameSeq = _wbFrameSeq++;

    int slot = -1;
    portENTER_CRITICAL(&_wbMux);
    for (int i = 0; i < WFS2_QUEUE_DEPTH; ++i) {
        if (_wbSlots[i].state == WFS2_EMPTY) {
            _wbSlots[i].state = WFS2_WRITING;
            slot = i;
            break;
        }
    }
    if (slot < 0) _wbFramesDropped++;
    portEXIT_CRITICAL(&_wbMux);
    if (slot < 0) return false;

    Wfs2Slot& out = _wbSlots[slot];
    memset(&out.header, 0, sizeof(out.header));
    memcpy(out.header.magic, "WFS2", 4);
    out.header.version = 2;
    out.header.header_bytes = sizeof(Wfs2Header);
    out.header.active_mask = activeMask;
    out.header.channel_count = wfs2ChannelCount(activeMask);
    out.header.boot_id = _wbBootId;
    out.header.frame_seq = frameSeq;
    out.header.config_revision = configRevision;
    out.header.capture_monotonic_us = (uint64_t)esp_timer_get_time();
    timeval tv = {};
    gettimeofday(&tv, nullptr);
    if (tv.tv_sec >= 1700000000L) {
        out.header.capture_epoch_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint32_t)tv.tv_usec;
        out.header.flags |= WFS2_FLAG_CLOCK_SYNCED;
    }
    if (onDemand) out.header.flags |= WFS2_FLAG_ON_DEMAND;
    out.header.flags |= WFS2_FLAG_DELTA_RLE;
    out.header.dropped_frames = _wbFramesDropped;
    out.header.sample_rate_hz = WFS2_SAMPLE_RATE_HZ;
    out.header.samples_per_channel = WFS2_SAMPLES_PER_CHANNEL;
    out.header.raw_payload_bytes = (uint32_t)wfs2PayloadBytes(
        activeMask, WFS2_SAMPLES_PER_CHANNEL);
    out.header.payload_bytes = out.header.raw_payload_bytes;

    _wbEncoder.begin(out.payload, sizeof(out.payload));
    _wbWriteSlot = (int8_t)slot;
    _wbWriteOnDemand = onDemand;
    return true;
}

// Called once for every enabled channel in every millisecond scan. readAllCT
// iterates channels in ascending order, which is the WFS2 wire ordering.
void wbFeed(int ch, uint16_t value) {
    if (_wbWriteSlot < 0) return;
    if (_wbWriteSamples >= WFS2_MAX_FRAME_SAMPLES) return;
    bool ok = _wbEncoder.feed((uint8_t)ch, value);
    _wbWriteCrc = wfs2Crc32Update(_wbWriteCrc, (uint8_t)(value & 0xff));
    _wbWriteCrc = wfs2Crc32Update(_wbWriteCrc, (uint8_t)(value >> 8));
    if (ok) _wbWriteSamples++;
}

// Publish only a complete frame. Partial or overfull frames are discarded as a
// unit, so the server never mistakes truncated data for continuous acquisition.
void wbEndFrame(uint32_t sampleDurationUs, uint16_t timingOverruns) {
    if (_wbWriteSlot < 0) return;
    Wfs2Slot& out = _wbSlots[_wbWriteSlot];
    uint32_t expectedSamples = (uint32_t)out.header.channel_count *
                               out.header.samples_per_channel;
    bool complete = _wbEncoder.finish() &&
                    _wbWriteSamples == expectedSamples &&
                    _wbEncoder.size() <= out.header.raw_payload_bytes;
    if (complete) {
        out.header.sample_duration_us = sampleDurationUs;
        out.header.timing_overruns = timingOverruns;
        if (timingOverruns > 0) out.header.flags |= WFS2_FLAG_TIMING_OVERRUN;
        out.header.payload_bytes = _wbEncoder.size();
        out.header.payload_crc32 = ~_wbWriteCrc;
        _wbFramesGenerated++;
    }

    portENTER_CRITICAL(&_wbMux);
    out.state = complete ? WFS2_READY : WFS2_EMPTY;
    if (!complete) _wbFramesDropped++;
    if (complete && _wbWriteOnDemand) _wbCaptureRequested = false;
    portEXIT_CRITICAL(&_wbMux);
    if (!complete) {
        Serial.printf("[WFS2] incomplete frame: samples=%u/%u bytes=%u/%u\n",
                      (unsigned)_wbWriteSamples, (unsigned)expectedSamples,
                      (unsigned)_wbEncoder.size(), (unsigned)out.header.raw_payload_bytes);
    }
    _wbWriteSlot = -1;
    _wbWriteSamples = 0;
    _wbWriteOnDemand = false;
}

void wbRequestCapture() { _wbCaptureRequested = true; }
bool wbDumpPending() { return _wbCaptureRequested || wbQueuedFrames() > 0; }
int wbChannel() {
    uint8_t mask = getActiveCTMask();
    for (int ch = 0; ch < NUM_CT_CHANNELS; ++ch)
        if (mask & (1u << ch)) return ch;
    return -1;
}

// Compatibility hooks retained for the command/heartbeat call sites. WFS2
// publishes the on-demand frame through wbStreamLoop; no flash dump is used.
void wbServiceCapture(int windowMs) { (void)windowMs; }
bool wbUploadDump(const String& serverBase, const String& deviceId) {
    (void)serverBase;
    (void)deviceId;
    return false;
}

// Core 0, called every 100ms. Two already-encoded frames share one request,
// amortizing the request/response latency that dominated the production pilot.
void wbStreamLoop() {
    if (_wbSlots == nullptr) return;
    if (isUpdateInProgress()) return;
    unsigned long now = millis();
    if (_wbBackoff && (now - _wbLastAttemptMs) < WB_STREAM_RETRY_MS) return;
    // Waveforms are strictly lower priority than ordinary telemetry. The 2.18.0
    // pilot allowed seven live readings to queue and WFS made that backlog
    // worse; 2.18.2 yields as soon as even one ordinary summary is pending.
    if (getQueueSize() != 0) return;
    long okAge = getLastSuccessAgeS();
    if (okAge < 0 || okAge > 30) return;
    if (ESP.getFreeHeap() < WB_STREAM_MIN_HEAP) return;

    int selected[WFS2_BATCH_MAX_FRAMES];
    int selectedCount = 0;
    portENTER_CRITICAL(&_wbMux);
    for (int pick = 0; pick < WFS2_BATCH_MAX_FRAMES; ++pick) {
        int oldest = -1;
        uint32_t oldestSeq = 0;
        for (int i = 0; i < WFS2_QUEUE_DEPTH; ++i) {
            if (_wbSlots[i].state != WFS2_READY) continue;
            bool alreadySelected = false;
            for (int j = 0; j < selectedCount; ++j)
                if (selected[j] == i) alreadySelected = true;
            if (alreadySelected) continue;
            uint32_t seq = _wbSlots[i].header.frame_seq;
            if (oldest < 0 || (int32_t)(seq - oldestSeq) < 0) {
                oldest = i;
                oldestSeq = seq;
            }
        }
        if (oldest < 0) break;
        selected[selectedCount++] = oldest;
    }
    portEXIT_CRITICAL(&_wbMux);
    if (selectedCount == 0) {
        _wbSingleReadySinceMs = 0;
        return;
    }

    bool urgentSingle = !_wbStreamEnabled ||
        (_wbSlots[selected[0]].header.flags & WFS2_FLAG_ON_DEMAND);
    if (selectedCount == 1 && !urgentSingle) {
        if (_wbSingleReadySinceMs == 0) _wbSingleReadySinceMs = now;
        if (now - _wbSingleReadySinceMs < WFS2_BATCH_WAIT_MS) return;
    }
    _wbSingleReadySinceMs = 0;

    size_t bodyBytes = 0;
    for (int i = 0; i < selectedCount; ++i) {
        Wfs2Slot& frame = _wbSlots[selected[i]];
        size_t frameBytes = sizeof(Wfs2Header) + frame.header.payload_bytes;
        if (bodyBytes + frameBytes > WFS2_BATCH_BUFFER_BYTES) {
            _wbBackoff = true;
            _wbLastAttemptMs = millis();
            return;
        }
        bodyBytes += frameBytes;
    }
    if (bodyBytes > _wbBatchCapacity) {
        size_t growth = bodyBytes - _wbBatchCapacity;
        if (ESP.getFreeHeap() < WB_STREAM_MIN_HEAP + growth +
                                WB_STREAM_ALLOC_GUARD) {
            // Never let a worst-case noisy frame consume the heap margin that
            // ordinary telemetry, JSON, and WiFi recovery need to stay alive.
            return;
        }
        uint8_t* grown = (uint8_t*)realloc(_wbBatchBuf, bodyBytes);
        if (grown == nullptr) {
            _wbBackoff = true;
            _wbLastAttemptMs = millis();
            return;
        }
        _wbBatchBuf = grown;
        _wbBatchCapacity = bodyBytes;
    }

    portENTER_CRITICAL(&_wbMux);
    for (int i = 0; i < selectedCount; ++i)
        _wbSlots[selected[i]].state = WFS2_SENDING;
    portEXIT_CRITICAL(&_wbMux);
    bodyBytes = 0;
    for (int i = 0; i < selectedCount; ++i) {
        Wfs2Slot& frame = _wbSlots[selected[i]];
        size_t frameBytes = sizeof(Wfs2Header) + frame.header.payload_bytes;
        memcpy(_wbBatchBuf + bodyBytes, &frame.header, frameBytes);
        bodyBytes += frameBytes;
    }

    String url = String("http://46.224.90.187/api/waveform/v2") +
                 (selectedCount > 1 ? "/batch" : "") +
                 "?device_id=" + getDeviceId();
    if (!_wbHttpInit) {
        _wbHttp.setReuse(true);
        _wbHttp.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
        _wbHttp.setTimeout(8000);
        _wbHttpInit = true;
    }
    feedWatchdog();
    _wbHttp.begin(_wbClient, url);
    _wbHttp.addHeader("Content-Type", selectedCount > 1
        ? "application/vnd.linesights.wfs2-batch"
        : "application/vnd.linesights.wfs2");
    int code = _wbHttp.POST(_wbBatchBuf, bodyBytes);
    _wbHttp.end();
    feedWatchdog();
    _wbLastAttemptMs = millis();

    portENTER_CRITICAL(&_wbMux);
    if (code >= 200 && code < 300) {
        for (int i = 0; i < selectedCount; ++i)
            _wbSlots[selected[i]].state = WFS2_EMPTY;
        _wbFramesDelivered += selectedCount;
        _wbBackoff = false;
    } else {
        for (int i = 0; i < selectedCount; ++i)
            _wbSlots[selected[i]].state = WFS2_READY;
        _wbBackoff = true;
    }
    portEXIT_CRITICAL(&_wbMux);

    if (code < 200 || code >= 300) {
        _wbClient.stop();
        static unsigned long lastLogMs = 0;
        if (millis() - lastLogMs > 300000UL) {
            lastLogMs = millis();
            Serial.printf("[WFS2] %d-frame POST failed (%d)\n", selectedCount, code);
        }
    }
}
