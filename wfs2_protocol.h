#pragma once

#include <stddef.h>
#include <stdint.h>

// WFS2 wire format. All integer fields and uint16 samples are little-endian.
// Payload order is scan-major: for every millisecond, enabled channels appear
// once in ascending CT order. Example mask 0b000101 => CT1, CT3, CT1, CT3...
struct __attribute__((packed)) Wfs2Header {
    char     magic[4];                 // "WFS2"
    uint8_t  version;                  // 2
    uint8_t  header_bytes;             // sizeof(Wfs2Header)
    uint8_t  active_mask;              // bit 0 = CT1 ... bit 5 = CT6
    uint8_t  channel_count;
    uint32_t boot_id;                  // random per boot; scopes frame_seq
    uint32_t frame_seq;                // increments for every generated frame
    uint32_t config_revision;          // applied CT configuration
    uint64_t capture_epoch_us;         // 0 when wall clock is not trustworthy
    uint64_t capture_monotonic_us;     // microseconds since boot
    uint32_t sample_duration_us;       // measured first-boundary to last-boundary
    uint32_t dropped_frames;           // cumulative before this frame
    uint16_t sample_rate_hz;
    uint16_t samples_per_channel;
    uint32_t raw_payload_bytes;
    uint32_t payload_bytes;             // encoded bytes following the header
    uint32_t payload_crc32;             // IEEE CRC-32 over DECODED samples
    uint16_t flags;
    uint16_t timing_overruns;         // scans that finished after their 1ms boundary
};

static_assert(sizeof(Wfs2Header) == 64, "WFS2 header layout changed");

enum : uint16_t {
    WFS2_FLAG_CLOCK_SYNCED = 0x0001,
    WFS2_FLAG_ON_DEMAND    = 0x0002,
    WFS2_FLAG_TIMING_OVERRUN = 0x0004,
    WFS2_FLAG_RAW_DEFLATE     = 0x0008,
    WFS2_FLAG_DELTA_RLE       = 0x0010,
};

static inline uint8_t wfs2ChannelCount(uint8_t mask) {
    mask &= 0x3f;
    uint8_t n = 0;
    while (mask) {
        n += mask & 1u;
        mask >>= 1;
    }
    return n;
}

static inline size_t wfs2PayloadBytes(uint8_t mask, uint16_t samplesPerChannel) {
    return (size_t)wfs2ChannelCount(mask) * samplesPerChannel * sizeof(uint16_t);
}

static inline uint32_t wfs2Crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)-(int32_t)(crc & 1u));
    }
    return ~crc;
}

static inline uint32_t wfs2Crc32Update(uint32_t crc, uint8_t value) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit)
        crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)-(int32_t)(crc & 1u));
    return crc;
}

// Bounded lossless scan-major encoder used directly by the sampler. The first
// value for every channel is literal; later values use per-channel deltas and
// a shared zero-run across scan order. For 12-bit ADC values every nonzero
// token is at most two bytes, so capacity == raw uint16 bytes is sufficient.
class Wfs2DeltaEncoder {
public:
    void begin(uint8_t* output, size_t capacity) {
        _output = output;
        _capacity = capacity;
        _size = 0;
        _seenMask = 0;
        _zeroRun = 0;
        _valid = output != nullptr;
        for (int i = 0; i < 6; ++i) _previous[i] = 0;
    }

    bool feed(uint8_t channel, uint16_t value) {
        if (!_valid || channel >= 6 || value > 4095) {
            _valid = false;
            return false;
        }
        bool ok = true;
        if (!(_seenMask & (1u << channel))) {
            ok = flushZeroRun() && putByte((uint8_t)(value & 0xff)) &&
                 putByte((uint8_t)(value >> 8));
            _seenMask |= 1u << channel;
        } else {
            int32_t delta = (int32_t)value - (int32_t)_previous[channel];
            if (delta == 0) {
                if (_zeroRun == UINT16_MAX) ok = flushZeroRun();
                if (ok) ++_zeroRun;
            } else {
                ok = flushZeroRun();
                uint16_t zigzag = (uint16_t)(((uint32_t)delta << 1) ^
                                              (uint32_t)(delta >> 31));
                if (ok) ok = putVarint((uint16_t)(zigzag + 1));
            }
        }
        _previous[channel] = value;
        if (!ok) _valid = false;
        return _valid;
    }

    bool finish() {
        if (!_valid) return false;
        _valid = flushZeroRun();
        return _valid;
    }

    size_t size() const { return _size; }
    bool valid() const { return _valid; }

private:
    bool putByte(uint8_t value) {
        if (_size >= _capacity) return false;
        _output[_size++] = value;
        return true;
    }

    bool putVarint(uint16_t value) {
        while (value >= 0x80) {
            if (!putByte((uint8_t)((value & 0x7f) | 0x80))) return false;
            value >>= 7;
        }
        return putByte((uint8_t)value);
    }

    bool flushZeroRun() {
        if (_zeroRun == 0) return true;
        bool ok = _zeroRun == 1
            ? putByte(0)
            : putByte(1) && putVarint(_zeroRun);
        _zeroRun = 0;
        return ok;
    }

    uint8_t* _output = nullptr;
    size_t _capacity = 0;
    size_t _size = 0;
    uint16_t _previous[6] = {};
    uint8_t _seenMask = 0;
    uint16_t _zeroRun = 0;
    bool _valid = false;
};
