#pragma once

#include <Arduino.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp32/rom/miniz.h>
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
// WFS2 complete-frame waveform queue (2.18.0)
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
    uint16_t samples[WFS2_MAX_FRAME_SAMPLES];
    volatile uint8_t state;
};

static_assert(offsetof(Wfs2Slot, samples) == sizeof(Wfs2Header),
              "WFS2 header and sample payload must be contiguous");

static portMUX_TYPE _wbMux = portMUX_INITIALIZER_UNLOCKED;
static Wfs2Slot* _wbSlots = nullptr;
static int8_t _wbWriteSlot = -1;               // Core 1 only
static uint32_t _wbWriteSamples = 0;            // Core 1 only
static bool _wbWriteOnDemand = false;           // Core 1 only
static volatile bool _wbCaptureRequested = false;
static volatile bool _wbStreamEnabled = false;
static uint32_t _wbBootId = 0;
static volatile uint32_t _wbFramesGenerated = 0;
static volatile uint32_t _wbFramesDelivered = 0;
static volatile uint32_t _wbFramesDropped = 0;
static unsigned long _wbLastAttemptMs = 0;
static bool _wbBackoff = false;
static WiFiClient _wbClient;
static HTTPClient _wbHttp;
static bool _wbHttpInit = false;
static uint8_t* _wbSendBuf = nullptr;              // Core 0 deflate/header staging

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
    if (_wbSlots == nullptr || activeMask == 0) return false;

    bool onDemand = _wbCaptureRequested;
    if (!_wbStreamEnabled && !onDemand) return false;

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
    out.header.frame_seq = _wbFramesGenerated++;
    out.header.config_revision = configRevision;
    out.header.capture_monotonic_us = (uint64_t)esp_timer_get_time();
    timeval tv = {};
    gettimeofday(&tv, nullptr);
    if (tv.tv_sec >= 1700000000L) {
        out.header.capture_epoch_us = (uint64_t)tv.tv_sec * 1000000ULL + (uint32_t)tv.tv_usec;
        out.header.flags |= WFS2_FLAG_CLOCK_SYNCED;
    }
    if (onDemand) out.header.flags |= WFS2_FLAG_ON_DEMAND;
    out.header.dropped_frames = _wbFramesDropped;
    out.header.sample_rate_hz = WFS2_SAMPLE_RATE_HZ;
    out.header.samples_per_channel = WFS2_SAMPLES_PER_CHANNEL;
    out.header.raw_payload_bytes = (uint32_t)wfs2PayloadBytes(
        activeMask, WFS2_SAMPLES_PER_CHANNEL);
    out.header.payload_bytes = out.header.raw_payload_bytes;

    _wbWriteSlot = (int8_t)slot;
    _wbWriteOnDemand = onDemand;
    return true;
}

// Called once for every enabled channel in every millisecond scan. readAllCT
// iterates channels in ascending order, which is the WFS2 wire ordering.
void wbFeed(int ch, uint16_t value) {
    (void)ch;
    if (_wbWriteSlot < 0) return;
    if (_wbWriteSamples >= WFS2_MAX_FRAME_SAMPLES) return;
    _wbSlots[_wbWriteSlot].samples[_wbWriteSamples++] = value;
}

// Publish only a complete frame. Partial or overfull frames are discarded as a
// unit, so the server never mistakes truncated data for continuous acquisition.
void wbEndFrame(uint32_t sampleDurationUs, uint16_t timingOverruns) {
    if (_wbWriteSlot < 0) return;
    Wfs2Slot& out = _wbSlots[_wbWriteSlot];
    uint32_t expectedSamples = (uint32_t)out.header.channel_count *
                               out.header.samples_per_channel;
    bool complete = _wbWriteSamples == expectedSamples;
    if (complete) {
        out.header.sample_duration_us = sampleDurationUs;
        out.header.timing_overruns = timingOverruns;
        if (timingOverruns > 0) out.header.flags |= WFS2_FLAG_TIMING_OVERRUN;
        out.header.payload_crc32 = wfs2Crc32(
            (const uint8_t*)out.samples, out.header.raw_payload_bytes);
    }

    portENTER_CRITICAL(&_wbMux);
    out.state = complete ? WFS2_READY : WFS2_EMPTY;
    if (!complete) _wbFramesDropped++;
    if (complete && _wbWriteOnDemand) _wbCaptureRequested = false;
    portEXIT_CRITICAL(&_wbMux);
    if (!complete) {
        Serial.printf("[WFS2] incomplete frame: got=%u want=%u\n",
                      (unsigned)_wbWriteSamples, (unsigned)expectedSamples);
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

// Core 0, called every 100ms. At most one complete frame is sent per call.
void wbStreamLoop() {
    if (_wbSlots == nullptr) return;
    if (isUpdateInProgress()) return;
    unsigned long now = millis();
    if (_wbBackoff && (now - _wbLastAttemptMs) < WB_STREAM_RETRY_MS) return;
    if (getQueueSize() > BG_UPLOAD_LOWATER) return;
    long okAge = getLastSuccessAgeS();
    if (okAge < 0 || okAge > 30) return;
    if (ESP.getFreeHeap() < WB_STREAM_MIN_HEAP) return;

    int slot = -1;
    uint32_t oldestSeq = 0;
    portENTER_CRITICAL(&_wbMux);
    for (int i = 0; i < WFS2_QUEUE_DEPTH; ++i) {
        if (_wbSlots[i].state != WFS2_READY) continue;
        uint32_t seq = _wbSlots[i].header.frame_seq;
        if (slot < 0 || (int32_t)(seq - oldestSeq) < 0) {
            slot = i;
            oldestSeq = seq;
        }
    }
    if (slot >= 0) _wbSlots[slot].state = WFS2_SENDING;
    portEXIT_CRITICAL(&_wbMux);
    if (slot < 0) return;

    Wfs2Slot& frame = _wbSlots[slot];
    if (_wbSendBuf == nullptr) {
        _wbSendBuf = (uint8_t*)malloc(WFS2_ENCODE_SCRATCH_BYTES);
        if (_wbSendBuf == nullptr) {
            portENTER_CRITICAL(&_wbMux);
            frame.state = WFS2_READY;
            _wbBackoff = true;
            portEXIT_CRITICAL(&_wbMux);
            _wbLastAttemptMs = millis();
            return;
        }
    }

    Wfs2Header encodedHeader = frame.header;
    const size_t rawBytes = frame.header.raw_payload_bytes;
    size_t encodedBytes = tdefl_compress_mem_to_mem(
        _wbSendBuf + sizeof(Wfs2Header),
        WFS2_ENCODE_SCRATCH_BYTES - sizeof(Wfs2Header),
        frame.samples, rawBytes, TDEFL_DEFAULT_MAX_PROBES);
    if (encodedBytes > 0 && encodedBytes != (size_t)-1 && encodedBytes < rawBytes) {
        encodedHeader.flags |= WFS2_FLAG_RAW_DEFLATE;
        encodedHeader.payload_bytes = (uint32_t)encodedBytes;
    } else {
        encodedBytes = rawBytes;
        encodedHeader.payload_bytes = (uint32_t)rawBytes;
        memcpy(_wbSendBuf + sizeof(Wfs2Header), frame.samples, rawBytes);
    }
    memcpy(_wbSendBuf, &encodedHeader, sizeof(encodedHeader));
    size_t bodyBytes = sizeof(Wfs2Header) + encodedBytes;
    String url = String("http://46.224.90.187/api/waveform/v2?device_id=") +
                 getDeviceId();
    if (!_wbHttpInit) {
        _wbHttp.setReuse(true);
        _wbHttp.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
        _wbHttp.setTimeout(8000);
        _wbHttpInit = true;
    }
    feedWatchdog();
    _wbHttp.begin(_wbClient, url);
    _wbHttp.addHeader("Content-Type", "application/vnd.linesights.wfs2");
    int code = _wbHttp.POST(_wbSendBuf, bodyBytes);
    _wbHttp.end();
    feedWatchdog();
    _wbLastAttemptMs = millis();

    portENTER_CRITICAL(&_wbMux);
    if (code >= 200 && code < 300) {
        frame.state = WFS2_EMPTY;
        _wbFramesDelivered++;
        _wbBackoff = false;
    } else {
        frame.state = WFS2_READY;
        _wbBackoff = true;
    }
    portEXIT_CRITICAL(&_wbMux);

    if (code < 200 || code >= 300) {
        _wbClient.stop();
        static unsigned long lastLogMs = 0;
        if (millis() - lastLogMs > 300000UL) {
            lastLogMs = millis();
            Serial.printf("[WFS2] frame POST failed (%d)\n", code);
        }
    }
}
