# LS02 offline upload format — single source of truth

Firmware **v2.7.0** and the server bulk-ingest decoder both code against this
document. Do not change it on one side only.

LS02 replaces LS01 because two things changed at once (bundled deliberately to
bump the binary format exactly once):

1. **C3** — offline amps are now **int16 centi-amps** (`amps × 100`), not int16
   milliamps. Milliamps overflowed above 32.767 A (garbage for the 40–50 A loads
   the ADC can actually reach); centi-amps top out at 327.67 A, far above the
   **~50.5 A hardware ceiling** (`0.0123 × 4095 + 0.13`), and 10 mA resolution is
   below the ADC's own 12.3 mA/count step, so it is lossless against the sensor.
2. **D4** — each block now carries its **own epoch + a monotonic millis anchor +
   a flags byte + a CRC16**, so a block can be timestamped, re-stamped, or
   quarantined without desyncing the rest of the file.

Half-B waveform features (`peak_amps`, `env_peak_ratio`, `ripple_amps`, `env5`)
are **NOT** in this binary format. They ride the live JSON payload only (D3).
Offline records are counts/amps only.

---

## 1. Dispatch rule (critical — D2)

The server **MUST** dispatch on the **first 4 bytes of the request body**, never
on the `X-Format` header.

| Body magic | Meaning | Decoder |
|------------|---------|---------|
| `LS01` | legacy v2.6 file | LS01 decoder (existing) |
| `LS02` | v2.7.0 file | LS02 decoder (this doc) |

Why not the header: a unit that OTA-upgrades to v2.7.0, writes an `LS02` file,
then **rolls back** to v2.6 (3 post-update crashes) will stream that `LS02` body
under v2.6's hard-coded `X-Format: ls01-blocked` header. Trusting the header
would mis-decode it. v2.7.0 sends `X-Format: ls02-blocked`, but it is advisory
only — **the body magic is authoritative.**

---

## 2. File layout

```
[ FileHeader (40 bytes) ]
[ Block 0 ]
[ Block 1 ]
...
[ Block N ]
```

### FileHeader — 40 bytes, packed, little-endian

| Offset | Size | Field        | Notes |
|-------:|-----:|--------------|-------|
| 0      | 4    | `magic`      | ASCII `"LS02"` (not NUL-terminated) |
| 4      | 32   | `device_id`  | ASCII, NUL-padded |
| 36     | 4    | `file_epoch` | uint32 — best-effort creation epoch, **reference only**. May be 0 or stale. **Never reject on this.** |

`file_epoch` exists for debugging only. Authoritative per-reading time comes from
each block (Section 4).

### Block — 13-byte header + payload

| Offset | Size | Field          | Notes |
|-------:|-----:|----------------|-------|
| 0      | 4    | `start_epoch`  | uint32 — best-effort wall-clock epoch of the block's **first** reading. 0 if the device had no clock. |
| 4      | 4    | `start_millis` | uint32 — `millis()` (ms since device boot) at the block's first reading. The re-stamp anchor. |
| 8      | 1    | `flags`        | bit0 = `clock_unsynced` (epoch unreliable); bits 1–7 reserved, 0 |
| 9      | 2    | `size_flags`   | uint16 — bit15 = `1` uncompressed / `0` compressed; bits 0–14 = **payload byte length** |
| 11     | 2    | `crc16`        | uint16 — CRC16-CCITT of the payload bytes **as stored** (compressed or raw) |
| 13     | L    | `payload`      | `L = size_flags & 0x7FFF` bytes |

---

## 3. Payload → readings

1. `compressed = (size_flags & 0x8000) == 0`
2. Verify `crc16_ccitt(payload) == crc16`. **On mismatch, STOP decoding the file
   at this offset** and quarantine the remaining bytes (Section 5) — do not
   discard, do not error the whole upload.
3. If compressed, raw = zlib **raw DEFLATE** inflate of payload (the device uses
   miniz `tdefl_compress_mem_to_mem`, raw deflate, no zlib/gzip wrapper →
   `zlib.decompressobj(wbits=-15)`). Else raw = payload.
4. `N = len(raw) / 12` readings. Each reading = **6 × int16 little-endian
   centi-amps**.
5. `amps[ch] = int16_value / 100.0`  (channel order = CT 1..6).

### CRC16-CCITT (matches firmware `crc16_ccitt`)
poly `0x1021`, init `0xFFFF`, no reflection, no final XOR.

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc
```

---

## 4. Timestamping each reading

For block with readings `i = 0 .. N-1`:

- **If `flags.clock_unsynced == 0` and `start_epoch > 0`:**
  `reading_epoch[i] = start_epoch + i × interval` — `interval` from the
  `X-Interval` request header (§8), 1 when absent. NOT a fixed 1 Hz cadence:
  the device send interval is runtime-configurable 1–60 s.

- **If `flags.clock_unsynced == 1`** (block written before NTP locked): use the
  **upload-time NTP reference** the device attaches as request headers:

  | Header | Meaning |
  |--------|---------|
  | `X-Ntp-Epoch`  | device epoch at the moment of upload |
  | `X-Ntp-Millis` | device `millis()` at the moment of upload |
  | `X-Ntp-Valid`  | `1` if the device clock was NTP-synced at upload, else `0` |

  If `X-Ntp-Valid == 1` **and** `X-Anchor-Valid == 1` (§8 — same-boot-session guard):
  ```
  boot_epoch       = X-Ntp-Epoch - (X-Ntp-Millis / 1000)
  reading_epoch[i] = boot_epoch + (block.start_millis / 1000) + i × interval
  ```
  This is the "re-stamp after NTP" mechanism — done from the reference pair, so
  the device never rewrites the flash file.

  If `X-Ntp-Valid == 0` or `X-Anchor-Valid != 1` (no clock at upload, or blocks
  from an earlier boot session — millis anchor invalid): **accept the rows and
  route them to the staging/quarantine table (D5). Never return 4xx.**
  They can be reconciled later or by relative time.

---

## 5. Error handling — the contract that keeps it zero-loss

The device **deletes its only copy on HTTP 200 and nothing else.** Any 4xx makes
it quarantine-and-retry, not drop. So the server's status code is a data-loss
lever. Rules:

- **200** — fully ingested (including rows sent to staging). Only a 200 lets the
  device delete the file.
- **Never 400/422 a whole upload for a partial problem.** Decode every block you
  can. For blocks that fail CRC or are truncated (rollback-append tail, flash
  corruption), ingest the good prefix, stage the bad tail, and still return
  **200** with a JSON body summarizing what was staged.
- **Never 4xx `clock_unsynced` rows or 1970 timestamps** — stage them.
- Reserve non-200 for "I genuinely could not store anything" (DB down) so the
  device keeps the file and retries. A 5xx is safe (device retries); a 4xx is
  dangerous (older firmware deletes — though v2.7.0 now quarantines instead).

---

## 6. Sizes (sanity checks)

- `sizeof(FileHeader) == 40`
- `sizeof(BlockHeader) == 13`
- raw reading = 12 bytes (6 × int16)
- a full block = 10 readings → 120 raw bytes → compressed + 13-byte header

## 7. Worked byte-layout example

A single synced block of 1 reading, all six channels at 12.34 A
(`12.34 × 100 = 1234 = 0x04D2`), epoch `0x6688A0C0`, millis `0x000F4240`
(1,000,000), stored uncompressed:

```
FileHeader:
  4C 53 30 32                                  "LS02"
  64 65 76 30 31 00 ... (32 bytes)             device_id "dev01"
  C0 A0 88 66                                  file_epoch = 0x6688A0C0

Block:
  C0 A0 88 66                                  start_epoch  = 0x6688A0C0
  40 42 0F 00                                  start_millis = 1000000
  00                                           flags = 0 (synced)
  18 80                                        size_flags = 0x8018 = uncompressed, len 24
  ?? ??                                        crc16 over the 24 payload bytes
  D2 04 D2 04 D2 04 D2 04 D2 04 D2 04          6 × int16 LE = 1234 → 12.34 A each
```

`len = 0x8018 & 0x7FFF = 0x18 = 24 = 6 channels × 2 bytes × 1 reading.` ✓

## 8. Upload request headers (v2.7.0)

| Header | Values | Decoder rule |
|---|---|---|
| `X-Format` | `ls02-blocked` / `ls01-blocked` / `rejected-ndjson` | ADVISORY for binary (dispatch on body magic, §1); authoritative only for `rejected-ndjson` |
| `X-Ntp-Epoch` / `X-Ntp-Millis` / `X-Ntp-Valid` | epoch s / device millis / "1"\|"0" | re-stamp reference: `boot_epoch = epoch - millis/1000` |
| `X-Anchor-Valid` | "1"\|"0" (missing ⇒ "0") | re-stamp `clock_unsynced` blocks ONLY when "1" — the device sends "1" only for a file whose blocks were written this boot session (upload-time rotation slot). Otherwise STAGE. |
| `X-Interval` | 1–60 (missing ⇒ 1) | positional stamping is `start_epoch + i × interval` |

## 9. Chunking (v2.7.0 device behavior, no format change)

The device rotates its current file into the upload queue every ~64 KB
(`OFFLINE_CHUNK_BYTES`), so one outage arrives as several small LS02 uploads
instead of one large one. Each chunk is a complete, self-contained LS02 file
(own 40-byte header). Chunk uploads carry `X-Anchor-Valid: 0` (conservative).

## 10. rejected-ndjson (drain of 400'd live readings)

`Content-Type: application/x-ndjson`, `X-Format: rejected-ndjson`, body = one
live-payload JSON per line (exactly what the device once POSTed to /api/data),
≤50 KB per request. Ingest each line through the normal live path,
IDEMPOTENTLY (upsert on device_id+timestamp; duplicates are a no-op 200 —
most of these rows are same-second twins the server rejected pre-v2.7).
Quarantine unparseable lines; return 200 unless nothing was stored.
Reference: `_decode_rejected_ndjson` + self-test 6 in `server_ls02_decoder.py`.
