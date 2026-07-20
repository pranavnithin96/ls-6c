# v2.7.0 — backend one-pager

Two halves shipped together (shared LS01→LS02 format bump): zero-loss fixes
(C1/C2/C3) and live waveform features (Half-B). Server work lands **before** any
device flashes.

## 1. New live-payload fields (Half-B)

Present **only** when a unit has the waveform flag on (toggle per-unit via
heartbeat command `wfstats_on` / `wfstats_off`). Absent ⇒ behave exactly as v2.6.

| JSON field (per `ct_N`) | Type | Meaning | Suggested DB target |
|-------------------------|------|---------|---------------------|
| `peak_amps` | float (1 dp) | within-second max, manufacturer formula on max count | existing `peak_amps` column |
| `env_peak_ratio` | float (2 dp) | within-second max/mean of the **rectified envelope** — **NOT** electrical crest factor | **do not** use the `crest_factor` column; JSONB `features.env_peak_ratio` |
| `ripple_amps` | float (2 dp) | within-second std (spread), amps | JSONB `features.ripple_amps` |
| `env5` | float[5] (1 dp) | five 100 ms sub-window means → 5 Hz envelope | JSONB `features.env5` |

Top-level `clock_unsynced: true` may appear when the device clock is pre-NTP —
**accept and stage these rows, never 4xx** (see §3).

`peak_amps` is populated **only for live-delivered readings.** Readings that come
back from an outage via the offline path have **no** waveform fields (Half-B is
live-only, D3) — `peak_amps` / `features.*` will be NULL for backfilled rows.
Cycle detection must tolerate feature gaps across outage windows.

Constants unchanged and still authoritative: `pf = 0.90`, `voltage_rms = 230`.
Do not infer real PF — this front end has no voltage phase.

## 2. LS02 offline format

Full spec + reference decoder: `LS02_FORMAT.md`, `server_ls02_decoder.py`
(self-test passes; CRC verified as CRC-16/CCITT-FALSE, check value `0x29B1`).
Cross-language proof: `ls02_host_test.c` encodes with the firmware's verbatim
structs/CRC (`cc -o t ls02_host_test.c -lz && ./t | <feed to decoder>`) and the
Python decoder round-trips it — struct packing, endianness, deflate stream,
centi-amp scale, anchor gating all verified end-to-end.

**Re-stamp gating (`X-Anchor-Valid`)**: the millis-based re-stamp of
`clock_unsynced` blocks is only arithmetically valid when the blocks were written
in the same boot session as the upload's `X-Ntp-Millis`. The device guarantees the
current `offline.dat` never spans sessions (it rotates any pre-boot file to a
legacy slot at startup) and sends `X-Anchor-Valid: 1` only for that current file.
For `0` or a missing header: **stage** unsynced blocks (epoch unresolved), never
re-stamp them. Synced blocks (`start_epoch > 0`, flag clear) are unaffected.

**Interval scaling (`X-Interval`)**: positional stamping within a block is
`start_epoch + i × interval`, NOT `+ i`. The send interval is runtime-configurable
1–60 s (portal / heartbeat `set_interval`); the device sends it per upload.
Missing/invalid header ⇒ assume 1. Best-effort: exact unless the interval was
changed while the file was being written (per-block `start_epoch` still bounds any
skew to within one 10-reading block).
Three decoder cases you must handle, all **dispatched on the 4-byte body magic**:
LS01 (legacy), LS02 (updated unit), and **LS02-body-under-LS01-header** (a unit
that rolled back to v2.6 — test it explicitly). CRC fail / truncation ⇒ ingest the
good prefix, stage the tail, still return 200.

## 3. The status-code contract (this is the data-loss lever)

v2.7.0 deletes its only copy on **200** and **quarantines (keeps) on any 4xx**.
So:
- **Never 4xx** a `clock_unsynced` / 1970-epoch row — route to a staging table.
- **Never 4xx** a whole upload for a partial/CRC problem — stage the bad part.
- Use **5xx** (or just keep failing the TCP connect) when you genuinely stored
  nothing — the device safely retries.
- This must be live **before** devices flash. If the server tightens 422 while a
  device is mid-rollout, you amplify loss instead of fixing it.

## 4. Buffer budget — capacity is finite, be honest about it

`OFFLINE_MAX_BYTES = 550,000`. LS02 block = 13-byte header + compressed payload
(raw payload = 10 readings × 6 ch × 2 bytes = 120 B; centi-amp, same width as
v2.6's milliamp — **no size regression**, which is why we chose int16 centi-amps
over int32 milliamps).

Time-to-full at 1 Hz, by how well steady-load data compresses:

| Compression | Block size | Bytes/reading | Capacity |
|-------------|-----------:|--------------:|---------:|
| optimistic (near-constant load) | ~43 B | 4.3 | **~35 h** |
| typical (varying load) | ~63 B | 6.3 | **~24 h** |
| pessimistic (uncompressible) | ~133 B | 13.3 | **~12 h** |

For comparison v2.6/LS01 typical was ~29 h — LS02's per-block epoch/millis/flags/
CRC costs ~5 h at 10 readings/block. (int32 milliamps would have cut this to
~12–15 h — avoided.)

**The observed 46 h outage (meton_01, 165,156 s) exceeds every column above.** No
1 Hz × 6-channel firmware can hold 46 h in 550 KB of flash — that's physics, not a
bug. Past capacity the device drops the **newest** readings and now surfaces it in
the heartbeat (`OFFLINE flash full — capacity exceeded`). Levers, if >~24 h
coverage is required (owner decision — none applied automatically):
1. `OFFLINE_BLOCK_READINGS` 10→20: amortizes the 13-byte header (claws back most
   of the LS02 overhead) at the cost of the max crash-loss window 9 s→19 s.
2. Raise `OFFLINE_MAX_BYTES` if the LittleFS partition has room.
3. Neither makes 46 h fit; that needs a lower offline cadence or a bigger partition.

## 5. Rollout order

1. Deploy server: LS02 decoder (body-magic dispatch + rollback case),
   `clock_unsynced` staging, the §3 status-code contract.
2. Flash 2–3 test units; run `TEST_PLAN.md` (esp. C/D/E/F).
3. Fleet OTA. Enable waveform features per-unit with `wfstats_on` once stable.

## 6. Heartbeat telemetry — the fleet page is dropping fields it already receives

The fleet dashboard reportedly stores only 6 fields. **Deployed v2.6 already sends
17+** — the handler is discarding the rest. Store these TODAY (no firmware change,
they arrive in every heartbeat from every device):

| Already arriving (v2.6) | Diagnoses |
|---|---|
| `boot_reason` (power_on / brownout / panic / task_watchdog / software) | why short-uptime devices reboot |
| `min_heap` | OOM near-misses that `free_heap` snapshots hide |
| `wifi_rssi_min`, `wifi_rssi_avg` | link quality beyond the instantaneous RSSI |
| `crash_count` (NVS-persisted) | crash frequency across reboots |
| `queue_depth`, `total_sent`, `total_failed`, `total_dropped` | backlog + delivery health |
| `errors[]` (timestamped ring, ≤30) | the device's own error log |

New in v2.7 (add columns when the fleet updates):

| Field | Diagnoses |
|---|---|
| `device_epoch` | clock skew: `received_at - device_epoch` (the mark_pdc +17.7h case) |
| `clock_synced` | NTP state, directly |
| `offline_stored`, `offline_bytes` | flash backlog depth / time-to-full |
| `rejected_log_bytes` | 400'd live readings parked on flash (cap 50,000) |
| `rejected_files` | quarantined offline files awaiting `recover_rejected` |
| `wifi_reconnects` | link flap count since boot |
| `last_http_code`, `last_success_age_s` | "unreachable" vs "server rejecting", cleanly (-1 = never) |
| `cpu_temp_c` | thermal stress (coarse internal sensor; trend, not absolute) |
| `heap_largest_block` | fragmentation (alloc failures despite free heap) |
| `mac`, `build` | stable hardware identity + exact build timestamp |

## 7. Idempotent ingest is now a REQUIREMENT (not a nice-to-have)

Upsert on `(device_id, timestamp)`; a duplicate row is a no-op **200**, never a
4xx. Three device behaviors depend on it:
1. Poor links lose 200-ACKs → whole offline files legitimately re-sent.
2. The `upload_rejected_log` drain re-submits rows the server previously 400'd
   (largely same-second duplicates) — the device deletes its copy on 200.
3. Reconnect flaps can re-queue a saved ring alongside in-RAM copies.

**`rejected-ndjson` uploads** (`X-Format: rejected-ndjson`, `application/x-ndjson`,
one live-payload JSON per line, ≤50KB per POST): route each line through the
normal /api/data ingest logic idempotently; quarantine unparseable lines; 200
unless nothing was stored. Reference: `_decode_rejected_ndjson` in the decoder.

## 8. New heartbeat commands (v2.7) + fields

Commands: `upload_rejected_log` (drain 400'd readings; one file per tick, aborts
after 10 consecutive failures — re-send once the server is fixed),
`set_reboot_days` {value: 0–45, 0=off; >45 rejected, millis-rollover limit}
(scheduled maintenance reboot, default 7d ±3h MAC jitter, safe-gated: online +
queue empty + no upload/drain pending + clock synced).
New field: `ntp_sync_age_s` (-1 = never; SNTP now explicitly re-syncs every 15 min,
re-kicked if stale >2h). Devices no longer emit same-second duplicate labels
(sample-time stamping + monotonic label clamp: a backward SNTP step relabels
forward by 1s rather than dropping or duplicating) — expect the ~1/min 400
trickle to stop fleet-wide after OTA.
