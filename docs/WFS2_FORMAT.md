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

Header: `X-LS-WFS-Token: <per-device bearer credential>`. The server creates a
new random credential whenever a super-admin enables the stream, stores only
its SHA-256 verifier, and provisions the raw value through heartbeat. The
device persists it in NVS. A device ID is routing metadata, not authentication;
missing/mismatched credentials and devices without explicit desired streaming
state receive 403 before their bodies are parsed or stored.

For a single urgent or final frame, the request body is one 64-byte
`Wfs2Header` followed immediately by its payload. Normally the device sends up
to three complete, increasing-sequence frames in one request:

`POST /api/waveform/v2/batch?device_id=<id>`

Content type: `application/vnd.linesights.wfs2-batch`

The batch body is up to three independently valid header+payload frames
concatenated. The receiver validates the complete batch before storing any
frame and rejects mixed boot IDs, non-increasing sequences, and a fourth frame.

Firmware 2.18.4 encodes the payload losslessly while Core 1 samples, using
`WFS2_FLAG_DELTA_RLE`. The first enabled-channel scan is literal little-endian
`uint16`. Remaining scan-major values use a predictor per channel:

- token `0`: one unchanged sample;
- token `1`, then unsigned LEB128 `N`: `N` unchanged samples (`N >= 2`);
- token `>= 2`: `zigzag(delta) + 1`, encoded as unsigned LEB128.

All values must decode to the ESP32's 12-bit ADC range (0-4095). Tokens and
runs are bounded so malformed input cannot expand indefinitely. The encoding's
worst case is no larger than the original two bytes per sample. The server
continues accepting older raw-DEFLATE and plain WFS2 frames, but exactly one
codec flag may be set. `raw_payload_bytes` is the decoded length,
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

Waveform traffic is best-effort. Any queued ordinary one-Hz reading, an
unhealthy live-data acknowledgement age, OTA, or low heap pauses WFS2 uploads.
Five complete-frame slots absorb short stalls; after that, sequence gaps and the
cumulative drop count make missing wall time explicit.

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
