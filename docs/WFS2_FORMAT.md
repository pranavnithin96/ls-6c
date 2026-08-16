# WFS2 complete-frame protocol

Firmware 2.18 replaces the strongest-channel WFS1 stream with complete,
configuration-scoped acquisition frames.

## CT configuration

Provisioning presents an explicit connected checkbox and rating for CT1-CT6.
A fresh device uses mask `0` and reports `ct_config_required=true` until that
setup is saved.

Remote configuration uses a heartbeat command:

```json
{
  "id": 123,
  "action": "set_ct_channels",
  "mask": 7,
  "revision": 18
}
```

Bits 0-5 represent CT1-CT6; mask `7` enables CT1, CT2 and CT3. `value` may be
used instead of `mask` for compatibility with scalar command storage. Revision
must increase. The device stages the command and applies it before the next
one-second frame, then reports `ct_active_mask` and `ct_config_revision` in its
heartbeat. Re-delivering the already-applied pair is idempotent.

## HTTP transport

`POST /api/waveform/v2?device_id=<id>`

Content type: `application/vnd.linesights.wfs2`

The request body is one 64-byte `Wfs2Header`, followed immediately by its
payload. Header fields are defined in
`wfs2_protocol.h`.

The payload is normally a raw-DEFLATE stream (`WFS2_FLAG_RAW_DEFLATE`) created
losslessly on Core 0. If compression cannot reduce a frame, the payload remains
plain little-endian `uint16`. `raw_payload_bytes` is the decoded length,
`payload_bytes` is the transmitted length, and `payload_crc32` covers decoded
samples.

Samples are scan-major in enabled-channel order. For mask `0b000101` the body
is:

```text
millisecond 0: CT1, CT3
millisecond 1: CT1, CT3
...
millisecond 999: CT1, CT3
```

The receiver must reject a frame unless all of these are true:

- magic is `WFS2`, version is `2`, and `header_bytes` is `64`;
- mask contains only bits 0-5 and `channel_count == popcount(mask)`;
- sample rate and samples per channel are both `1000`;
- request length is exactly `header_bytes + payload_bytes`;
- decoded length equals both `raw_payload_bytes` and
  `channel_count * samples_per_channel * 2`;
- decoded-payload CRC-32 matches `payload_crc32`;
- `(device_id, boot_id, frame_seq)` has not already been accepted.

`sample_duration_us` records the measured frame duration. `timing_overruns`
counts millisecond scans that completed after their deadline and sets the
`WFS2_FLAG_TIMING_OVERRUN` flag when nonzero.

Frames are never concatenated across seconds or configurations. Queue pressure
drops a whole new frame and increments `wfs2_dropped`; it never overwrites or
silently truncates a pending frame.

## Heartbeat observability

- `ct_active_mask`
- `ct_config_revision`
- `ct_config_required`
- `ct_config_pending`
- `wfs2_generated`
- `wfs2_delivered`
- `wfs2_dropped`
- `wfs2_queued`
- `sampling_overruns_last`
- `sampling_overruns_total`

WFS2 is still pilot-gated by `wfstream_on`. Ordinary one-Hz telemetry remains
independent of the waveform queue.
