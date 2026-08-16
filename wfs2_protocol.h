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
