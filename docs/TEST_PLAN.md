# v2.7.0 bench test plan

Run on a bench unit before any field rollout. Rollout order is **server first**
(LS02 decoder + `clock_unsynced` staging live), **then 2–3 test units**, then fleet.

## A. Waveform features (Half-B) — serial monitor

Enable on the bench unit: send heartbeat command `{"action":"wfstats_on"}` (or set
`FEATURE_WAVEFORM_STATS 1` for the bench build).

1. **Sample duration unchanged.** Watch `| <N>ms` at the end of each reading line
   (`sample_duration_ms`). It must read the same as v2.6 (≈500 ms) with the flag
   on or off — the added accumulators are tens of µs. If it grows by >2 ms,
   investigate before proceeding.
2. **Ordering sanity per channel.** With a load on CT1: `peak_amps ≥ amps ≥
   min_amps`. (`min_amps` is exposed in the binary path's reasoning but only
   peak/ratio/ripple/env5 are in the JSON — verify peak ≥ amps in the payload.)
3. **env_peak_ratio.** ~1.0 at idle (max ≈ mean); 1–5 under a real variable load.
   Should never exceed the clamp of 20.
4. **ripple — this is the double-precision regression check.** At idle ≈ a small
   noise floor (measure it; do NOT assume exactly 0 — the diode-rectifier front
   end has a floor). Under a variable load it must rise **clearly above** that
   floor. If ripple reads 0.00 under load, the float32 cancellation bug is back —
   the variance must be computed in `double` (it is, in `readAllCT`).
5. **env5.** Five plausible amps values tracking the load over the second.
6. **Saturation note (expected, not a bug):** drive a channel past ~50 A — ADC
   pins at 4095, so `peak ≈ amps`, `env_peak_ratio → 1`, `ripple → 0`. Features
   intentionally flatten on saturated channels.

Example live payload (flag on), one channel shown:

```json
{
  "device_id": "AABBCCDDEEFF",
  "timestamp": "2026-06-11T10:00:00.000Z",
  "location": "Bench",
  "timezone": "Asia/Kolkata",
  "readings": {
    "cts": {
      "ct_1": {
        "real_power_w": 2550.6, "amps": 12.345, "pf": 0.900,
        "peak_amps": 13.2, "env_peak_ratio": 1.07, "ripple_amps": 0.21,
        "env5": [12.1, 12.4, 12.3, 12.5, 12.2]
      }
    },
    "voltage_rms": 230.0
  }
}
```
With the flag **off**, the payload is byte-identical to v2.6 (no `peak_amps`,
`env_peak_ratio`, `ripple_amps`, `env5`, no `clock_unsynced`).

## B. Heap margin (flag on)

`JSON_BUF_SIZE` grew 640→1536 (+~27 KB BSS). Watch `[DIAG] Heap:<free>(min:<min>)`
for a few minutes with the flag on and a full send queue. Confirm `min` stays well
clear of the 15 KB reboot floor. If a real unit runs tight, the levers are: keep
the flag off on that unit, or reduce `MAX_BUFFER_SIZE`.

## C. Offline buffer fill/drain across a simulated outage

1. Block the server (firewall the device, or point it at a dead port).
2. After `OFFLINE_GRACE_MS` (30 s) the device enters offline mode; watch
   `[OFFLINE] Blk<n>: ... ` lines. Confirm block sizes are sane and the file grows.
3. Let it run several minutes, then restore the server.
4. On reconnect: `[HTTP] Server back` → the stall-time `buffer.json` is re-queued
   and sent **first** (C1, oldest-first), then `[UPLOAD] Sending /offline.dat...`.
5. Server-side: confirm every reading for the outage window arrives, correctly
   timestamped (1 Hz, no gaps, no duplicates at the buffer.json↔offline.dat seam).

## D. 422 injection — backlog must survive (C2)

1. Build offline backlog as in C.
2. Make the server return **422** for the bulk upload.
3. Confirm the device does **NOT** delete `/offline.dat`. Expect
   `[UPLOAD] 422 ... syncing NTP, retry once`, then on a second 422
   `[UPLOAD] Quarantined /offline.dat -> /offline.rejected.0.dat (kept, not deleted)`.
4. Confirm `rejected_files` appears >0 in the next heartbeat.
5. Restore the server to 200, send `{"action":"recover_rejected"}`, confirm the
   quarantined file is re-queued and delivered. **No readings lost.**

## E. clock_unsynced path (D5)

1. Boot the unit with NTP unreachable (block UDP 123) so the clock never syncs.
2. Live payloads must carry `"clock_unsynced": true`; offline blocks must log
   `UNSYNCED`. Server must **accept and stage** these, never 4xx.
3. Restore NTP. Confirm subsequent live readings drop the flag, and that an
   offline file written while unsynced, uploaded after sync, is re-stamped from
   the `X-Ntp-*` headers (server staging shows resolved epochs).

## F. Rollback survival (D2)

1. On a v2.7.0 unit, build an offline `/offline.dat` (LS02), then force a rollback
   to v2.6 (e.g. 3 induced crashes after an OTA, or flash v2.6 directly with the
   LS02 file present).
2. v2.6 streams the LS02 body under `X-Format: ls01-blocked`.
3. Confirm the **server** decodes it correctly by dispatching on the **body magic**
   (`LS02`), not the header — and ingests the readings. This is the explicit
   third decoder test case in `LS02_FORMAT.md` §1.

## G. OTA path

Confirm `isVersionGreater("2.7.0","2.6")` is true (a v2.6 unit offered 2.7.0 takes
it) and that a v2.7.0 unit reports `firmware: "2.7.0"` in heartbeat and diagnostics.

## H. Concurrency + anchor fixes (post-review hardening)

1. **Reconnect re-queue race (C1 fix)**: kill the server (not WiFi) for ~2 min so
   the unit enters offline mode with a saved `buffer.json`, then restore it.
   Confirm serial shows `[BUF] Loaded N readings from flash` BEFORE
   `[OFFLINE] Exited`, queue drains with no `JSON bad` / garbled payloads, and
   the server receives the stall-window readings exactly once.
2. **Ring-full producer safety**: with the server blackholed (connect timeout, no
   offline entry — use a firewall DROP so failures are slow), let the ring fill.
   Confirm drops are logged at most once/60s, readings land in `/rejected.log`,
   and no corrupted POST bodies appear when the server returns.
3. **Boot rotation + anchor gating**: write an offline file while NTP-unsynced,
   power-cycle, let it upload. Upload must carry `X-Anchor-Valid: 0` and the
   server must STAGE (not re-stamp) those rows. Then repeat without a power
   cycle: `X-Anchor-Valid: 1` and re-stamped epochs within seconds of wall clock.
4. **Foreign-file guard**: place a non-LS02 `/offline.dat` plus an occupied
   `/offline.legacy.dat`, trigger offline mode. The foreign file must be renamed
   to `/offline.legacy.1.dat` (never truncated); byte-compare it afterwards.
5. **Corrupt buffer quarantine**: write garbage into `/buffer.json`, reboot.
   Expect `/buffer.corrupt.0.json` to appear and boot to proceed normally.
6. **Heartbeat stall check**: with ~5 quarantined files present, confirm heartbeat
   cadence stays 60s (rejected-count is cached, no 1000-file scan per beat) and
   `rejected_files: 5` reports correctly after `recover_rejected` drains them.
7. **Backlog drain pace**: with 3+ legacy files pending (power-cycle offline a few
   times), confirm successive uploads land back-to-back (next tick after each 200),
   not one per 60 s — and that live 1 Hz readings keep flowing between them.

## I. Pending-fixes batch (2026-07-07)

1. **Duplicate-second fix**: run 24h against the production server; the ~1/min
   `400 — saved to /rejected.log` trickle must stop. Serial shows occasional
   `[DUP] same-second sample skipped` instead (roughly one per minute).
2. **Rejected-log lifecycle**: force 400s (server stub), fill past 50KB — expect
   rotation to `/rejected.1.log` (never a silent stop). Send `upload_rejected_log`;
   files drain oldest-first and vanish only on 200. `rejected_log_bytes` in the
   heartbeat tracks the total across archives.
3. **64KB chunking**: go offline >3h; expect `[OFFLINE] Chunk rotated` every
   ~64KB. On reconnect, chunks upload back-to-back; kill the link mid-drain and
   confirm already-delivered chunks are NOT re-sent (monotonic progress).
4. **Scheduled reboot**: `set_reboot_days {value: 0}` disables; with a short test
   value confirm the reboot only fires when online + queue empty + nothing
   pending, logs `scheduled maintenance reboot`, and `boot_reason` = "software".
5. **NTP cadence**: heartbeat `ntp_sync_age_s` should stay < ~900s in steady
   state; block NTP (firewall UDP 123) for 3h — device must re-kick and recover
   once unblocked, with the age dropping back.
6. **Total capacity**: with chunks + legacy backlog, confirm storage stops at
   OFFLINE_MAX_BYTES TOTAL (not per-file) and the flash-full error surfaces.
