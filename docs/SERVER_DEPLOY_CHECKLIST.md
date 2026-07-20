# Server deploy checklist — v2.7 rollout prerequisite

Target: the Flask backend on 46.224.90.187 / linesights.com.
Everything here must be LIVE before firmware 2.7.0 is published to the OTA
registry (publishing = fleet-wide OTA within ~1 hour).

References in this folder: `LS02_FORMAT.md` (wire format, single source of
truth), `server_ls02_decoder.py` (drop-in reference decoder, 6 self-tests),
`ls02_host_test.c` (firmware-identical encoder for round-trip testing),
`BACKEND_NOTES.md` (field tables).

---

## 1. LS02 decoder on POST /api/data/bulk

- Dispatch on the 4-byte BODY magic (`LS01` / `LS02`), NEVER on the
  `X-Format` header — a rolled-back v2.6 unit streams an LS02 body under
  `X-Format: ls01-blocked` (explicit test case below).
- LS02 layout: 40 B file header; per-block 13 B header
  (`start_epoch` u32, `start_millis` u32, `flags` u8 bit0=clock_unsynced,
  `size_flags` u16 bit15=uncompressed/len in bits 0-14, `crc16` u16
  CRC-16/CCITT-FALSE over the stored payload); payload = N × 6 × int16
  **centi-amps** little-endian (LS01 was **milli**amps — scale differs 10x).
- Compression: raw deflate, `zlib.decompressobj(-15)` **plus `.flush()`**
  (decompress alone can return a short buffer).
- Positional timestamps: `reading_epoch = start_epoch + i × interval`,
  interval from `X-Interval` header (1 when absent, clamp 1-60).
- Re-stamp `clock_unsynced` blocks ONLY when `X-Ntp-Valid: 1` AND
  `X-Anchor-Valid: 1`:
  `boot_epoch = X-Ntp-Epoch − X-Ntp-Millis/1000`,
  `reading_epoch = boot_epoch + start_millis/1000 + i × interval`.
  Otherwise STAGE those rows (unresolved epoch) — never guess, never 4xx.
- CRC failure or truncation mid-file: ingest the good prefix, stage the
  raw tail bytes, still return 200.

## 2. Status-code contract (THE data-loss lever)

The device deletes its only copy on 200 and quarantines (keeps) on any 4xx.

- Return 200 only when the data was actually stored.
- NEVER 4xx a clock_unsynced / 1970-epoch row → staging table.
- NEVER 4xx a whole upload over a partial/CRC problem → stage the bad part.
- Use 5xx (or refuse the connection) when nothing was stored — the device
  retries safely forever.

## 3. Idempotent ingest (hard requirement, not a nice-to-have)

Upsert on `(device_id, timestamp)`; a duplicate row is a no-op 200.
Three device behaviors depend on it: lost-ACK re-sends of whole files,
rejected-log drains re-submitting previously-400'd rows, and reconnect
flaps double-delivering ring contents. Apply the same rule to live
POST /api/data — this also retires the historical ~1/min duplicate-
timestamp 400 trickle server-side.

## 4. rejected-ndjson drain (same /api/data/bulk route)

`X-Format: rejected-ndjson`, `Content-Type: application/x-ndjson`, one
live-payload JSON object per line, ≤50 KB per POST. Route each line through
the normal live-ingest path idempotently; quarantine unparseable lines;
200 unless nothing stored. (Dispatch this BEFORE any binary length check —
an ndjson body has no 40-byte minimum.) Reference: `_decode_rejected_ndjson`.

## 5. Live POST /api/data additions

- Accept top-level `"clock_unsynced": true` → stage, never 4xx.
- Accept optional waveform fields when a unit has wfstats on:
  `peak_amps` (existing column), `env_peak_ratio` / `ripple_amps` / `env5`
  → JSONB `features.*`. Do NOT map env_peak_ratio to any `crest_factor`
  column — it is not electrical crest factor.

## 6. Staging table

For unresolved-epoch rows: device_id, payload, block metadata, received_at,
plus a reconciliation path (manual or scripted) once real timestamps are
known. Rows re-stamped later must respect the idempotency rule.

## 7. Heartbeat: store the new fields + stop discarding the old ones

Already sent by v2.6 and currently dropped by the handler: `boot_reason`,
`min_heap`, `wifi_rssi_min/avg`, `crash_count`, `queue_depth`,
`total_sent/failed/dropped`, `errors[]`.
New in v2.7: `device_epoch` (skew = received_at − device_epoch),
`clock_synced`, `ntp_sync_age_s`, `offline_stored`, `offline_bytes`,
`offline_backlog_bytes`, `rejected_log_bytes`, `rejected_files`,
`wifi_reconnects`, `last_http_code`, `last_success_age_s`, `cpu_temp_c`,
`heap_largest_block`, `mac`, `build`, `wfstats`.
New heartbeat commands the firmware understands: `wfstats_on`,
`wfstats_off`, `recover_rejected`, `upload_rejected_log`,
`set_reboot_days` {value: 0-45}.

## 8. Acceptance tests (run BEFORE flipping live)

1. `python3 server_ls02_decoder.py` → all 6 self-tests print ok.
2. Round-trip: `cc -o t ls02_host_test.c -lz && ./t > body.bin`, POST
   body.bin through the real route → 10 ingested + 3 staged, amps include
   45.67 A (impossible under the old int16-milliamp scale).
3. Rollback case: same body with `X-Format: ls01-blocked` → still decodes
   as LS02 (magic dispatch).
4. Unsynced: body with clock_unsynced flags and `X-Anchor-Valid: 0` →
   rows staged, response 200.
5. End-to-end with the bench unit (mark_pdc2): it already holds quarantined
   files from testing — send `recover_rejected` then watch the upload land;
   send `upload_rejected_log` and verify the overflow readings appear at
   their original timestamps.

## 9. Security fixes (bundle into the same deploy)

- `/api/ct_sensors` answers WITHOUT auth and enumerates every company's
  devices (verified live). Add auth + company scoping.
- `monitored_machines` without `pi_device_id` returns all companies for an
  admin session — confirm that is intended for the mobile apps' admin use.

## 10. After all of the above

1. Bench unit (mark_pdc2) soaks against the updated server: overnight zero
   400s, one full offline→quarantine→recover cycle.
2. 2-3 pilot units flashed over USB on factory WiFi, a few days.
3. THEN upload 2.7.0 to the OTA registry — that is the fleet rollout.
