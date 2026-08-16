#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../wfs2_protocol.h"

int main() {
    static const uint8_t standardVector[] = "123456789";
    assert(wfs2Crc32(standardVector, 9) == 0xcbf43926u);

    assert(wfs2ChannelCount(0x00) == 0);
    assert(wfs2ChannelCount(0x01) == 1);
    assert(wfs2ChannelCount(0x07) == 3);
    assert(wfs2ChannelCount(0x3f) == 6);
    assert(wfs2PayloadBytes(0x07, 1000) == 6000);
    assert(wfs2PayloadBytes(0x3f, 1000) == 12000);

    Wfs2Header header = {};
    memcpy(header.magic, "WFS2", 4);
    header.version = 2;
    header.header_bytes = sizeof(header);
    header.active_mask = 0x05;
    header.channel_count = wfs2ChannelCount(header.active_mask);
    header.sample_rate_hz = 1000;
    header.samples_per_channel = 1000;
    header.raw_payload_bytes = (uint32_t)wfs2PayloadBytes(
        header.active_mask, header.samples_per_channel);
    header.payload_bytes = header.raw_payload_bytes;

    assert(sizeof(header) == 64);
    assert(header.channel_count == 2);
    assert(header.payload_bytes == 4000);
    assert(memcmp(header.magic, "WFS2", 4) == 0);

    // Scan-major order for CT1+CT3: [CT1@t0, CT3@t0, CT1@t1, CT3@t1].
    uint16_t samples[] = {101, 301, 102, 302};
    header.payload_crc32 = wfs2Crc32(
        reinterpret_cast<const uint8_t*>(samples), sizeof(samples));
    assert(header.payload_crc32 != 0);
    assert(samples[0] == 101 && samples[1] == 301);
    assert(samples[2] == 102 && samples[3] == 302);

    puts("WFS2 protocol test passed");
    return 0;
}
