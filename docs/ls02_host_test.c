/*
 * Host-side LS02 encoder — byte-for-byte the firmware's structs/CRC/encoding
 * (copied verbatim from http_sender.h). Emits an LS02 upload body to stdout so
 * the Python reference decoder can round-trip it: proves struct packing, sizes,
 * endianness, CRC, centi-amp scale, and the compressed-block deflate stream all
 * match between firmware and server. Compression: zlib raw deflate (wbits=-15),
 * the same stream format the ESP32's tdefl emits.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <zlib.h>

/* === verbatim from http_sender.h === */
struct __attribute__((packed)) OfflineHeader {
    char magic[4];
    char device_id[32];
    uint32_t file_epoch;
};
struct __attribute__((packed)) OfflineBlockHeader {
    uint32_t start_epoch;
    uint32_t start_millis;
    uint8_t  flags;
    uint16_t size_flags;
    uint16_t crc16;
};
static uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}
#define OFFLINE_AMP_SCALE 100.0f
/* === end verbatim === */

static size_t raw_deflate(const uint8_t* in, size_t inLen, uint8_t* out, size_t outCap) {
    z_stream zs; memset(&zs, 0, sizeof(zs));
    /* raw deflate, wbits=-15 — same stream family as ESP32 ROM tdefl */
    if (deflateInit2(&zs, 9, Z_DEFLATED, -15, 8, Z_DEFAULT_STRATEGY) != Z_OK) return 0;
    zs.next_in = (Bytef*)in; zs.avail_in = inLen;
    zs.next_out = out; zs.avail_out = outCap;
    int rc = deflate(&zs, Z_FINISH);
    size_t n = (rc == Z_STREAM_END) ? outCap - zs.avail_out : 0;
    deflateEnd(&zs);
    return n;
}

int main(void) {
    /* Layout must match the decoder's struct.Struct sizes exactly */
    assert(sizeof(struct OfflineHeader) == 40);
    assert(sizeof(struct OfflineBlockHeader) == 13);
    /* CRC-16/CCITT-FALSE check value */
    assert(crc16_ccitt((const uint8_t*)"123456789", 9) == 0x29B1);

    struct OfflineHeader hdr;
    memcpy(hdr.magic, "LS02", 4);
    memset(hdr.device_id, 0, 32);
    strncpy(hdr.device_id, "hosttest01", 31);
    hdr.file_epoch = 1751800000u;
    fwrite(&hdr, sizeof(hdr), 1, stdout);

    /* Block 1: synced, 10 readings, firmware encode path (lroundf centi-amps),
       COMPRESSED. Values include >32.767A to prove the old int16-milliamp
       overflow (C3) is gone: 45.67A was impossible pre-LS02. */
    uint8_t raw[10 * 12];
    int idx = 0;
    for (int r = 0; r < 10; r++) {
        float amps[6] = {0.13f, 1.50f, 12.34f, 45.67f, 50.50f, 0.13f + 0.01f * r};
        for (int i = 0; i < 6; i++) {
            int16_t a = (int16_t)lroundf(amps[i] * OFFLINE_AMP_SCALE);  /* verbatim encode */
            memcpy(&raw[idx], &a, 2); idx += 2;
        }
    }
    uint8_t comp[1024];
    size_t compLen = raw_deflate(raw, sizeof(raw), comp, sizeof(comp));
    assert(compLen > 0 && compLen < sizeof(raw));

    struct OfflineBlockHeader bh1;
    bh1.start_epoch = 1751800100u;
    bh1.start_millis = 250000u;
    bh1.flags = 0;
    bh1.size_flags = (uint16_t)compLen;          /* compressed */
    bh1.crc16 = crc16_ccitt(comp, compLen);
    fwrite(&bh1, sizeof(bh1), 1, stdout);
    fwrite(comp, compLen, 1, stdout);

    /* Block 2: clock_unsynced, 3 readings, UNCOMPRESSED fallback (bit15). */
    uint8_t raw2[3 * 12];
    idx = 0;
    for (int r = 0; r < 3; r++) {
        for (int i = 0; i < 6; i++) {
            int16_t a = (int16_t)lroundf(2.22f * OFFLINE_AMP_SCALE);
            memcpy(&raw2[idx], &a, 2); idx += 2;
        }
    }
    struct OfflineBlockHeader bh2;
    bh2.start_epoch = 0;
    bh2.start_millis = 5000u;
    bh2.flags = 0x01;                            /* OFFLINE_BLOCK_FLAG_UNSYNCED */
    bh2.size_flags = (uint16_t)sizeof(raw2) | 0x8000;
    bh2.crc16 = crc16_ccitt(raw2, sizeof(raw2));
    fwrite(&bh2, sizeof(bh2), 1, stdout);
    fwrite(raw2, sizeof(raw2), 1, stdout);

    fflush(stdout);
    fprintf(stderr, "encoder ok: hdr=40 blk=13 crc=0x29B1 comp=%zu/%zu\n", compLen, sizeof(raw));
    return 0;
}
