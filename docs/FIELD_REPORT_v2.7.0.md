# v2.7.0 field report — PC Sons fleet, 2026-07-21

Written from the **server session**. The server side of v2.7.0 is deployed and
healthy; these are three firmware defects found in production, two of them
located to exact lines. One is an unrecoverable deadlock.

## TL;DR

All 15 PC Sons devices auto-updated to 2.7.0 at ~17:00 on 07-20 (2.7 is the
active OTA release, so they skipped the USB-pilot stage). Since then each one
is **dropping ~1,890 readings/hour and delivering only ~1,700/hour — ~53% loss**,
roughly 28,000 readings/hour across that site.

The trigger is a **full `/rejected.log`** left over from the pre-v2.7 400-storm.
The drain that would clear it **cannot ever run** (defect 1), and the fallback
lever (`set_interval`) **silently does nothing until reboot** (defect 2). So the
condition is unrecoverable over the air — it needs a firmware fix or a USB
reflash.

The server is not implicated: ingest answers in **6–8 ms** on both the local and
public paths, every device reports `last_http_code: 200`, and the historical
`400 — saved to /rejected.log` errors are **zero in the last 24 h**.

---

## Defect 1 — rejected-log drain deadlock (CRITICAL, unrecoverable OTA)

**`http_sender.h:1048`**

```c
} else if (_rejectedDrainPending && bufCount() == 0 &&
```

The drain is gated on an **empty send ring**. The comment at `http_sender.h:1167-1172`
explains why, and the reasoning is sound — draining inline blocked Core 0 for up
to 150 s, which filled the ring and manufactured new rejected entries.

But the gate creates a deadlock, because a full `/rejected.log` is itself what
keeps the ring full:

```
/rejected.log full
      -> send-ring overflow has nowhere to go -> "rejected store full — dropping"
      -> ring stays pinned at capacity (bufCount() != 0)
      -> drain gate never opens
      -> /rejected.log is never drained
      -> (back to the top, forever)
```

**Field evidence.** `queue_depth` is pinned at **29** on all 15 devices (ring
capacity 30) and `rejected_log_bytes` at **~301,000** on all 15. I queued
`upload_rejected_log` twice from the server:

| Test | Result |
|---|---|
| pcs1 (11.2 h uptime) | command delivered + marked executed; **0 bytes drained**, `rejected_log_bytes` unchanged at 301,791, no ndjson POST reached the server |
| pcs7 (rebooted 36 min earlier, so no latched `drainFails` state) | identical: **0 drained**, 301,027 unchanged |

A reboot does not help: pcs5 and pcs7 both rebooted and came back with full logs
and the same ~1,880/h drop rate.

**The correlation across the whole fleet is near-perfect** — this is the strongest
evidence that a full log *causes* the throughput collapse rather than merely
accompanying it:

| Device | `rejected_log_bytes` | sent/h | dropped/h |
|---|---:|---:|---:|
| mark_new_pdc | 3,759 | **3,602** | **0** |
| mark_pdc | 6,033 | 3,601 | 1 |
| mark_new_1 | 47,597 | 3,602 | 0 |
| aravind_cnc1 | 114,159 | 3,589 | 11 |
| aravind_1 | 131,180 | 3,585 | 15 |
| meton_04 | 197,211 | 3,505 | 92 |
| mark_04 | **301,272** | 3,420 | 181 |
| aravind_02 | **300,770** | 2,398 | 937 |
| **all 15 pcs** | **~301,000** | **~1,700** | **~1,890** |

Empty log → full 1 Hz (3,602/h), zero drops. Full log → collapse. Note this is
independent of link quality: the pcs devices span RSSI −48 to −77 with identical
results, and devices on *other* sites with weaker signal but empty logs run clean.

**Suggested fix.** Break the circular dependency — the drain must be able to run
when the ring is full, since that is exactly when it is needed. Options, roughly
in order of preference:

1. Relax the gate to a low-water mark rather than empty (`bufCount() < N`), so a
   saturated ring can still drain one archive per tick.
2. Add a starvation escape: if `_rejectedDrainPending` has been set for more than
   N ticks without the gate opening, drain one file regardless (the original
   150 s stall concern is bounded by draining a single archive, not all of them).
3. Let ring overflow spill to the **offline buffer** (`OFFLINE_MAX_BYTES`
   550,000, currently near-empty on these devices — `offline_stored` is 12–18)
   instead of `/rejected.log`. `/rejected.log` is a dead-letter store for rows
   the *server* rejected with 4xx; ring overflow is a different condition
   ("could not send") and arguably does not belong there at all.

Any of these would have let the fleet self-heal.

## Defect 2 — `set_interval` does not apply until reboot (HIGH)

**`heartbeat.h:179-184`** stores the new value but never updates the live one:

```c
if (action == "set_interval") {
    int val = cmd["value"] | -1;
    if (val >= 1 && val <= 60) {
        Preferences p; p.begin("lscfg", false);
        p.putInt("interval", val); p.end();     // NVS only
    }
}
```

`_sendInterval` is read **only** in `_loadConfig()` (`wifi_manager.h:184`), which
runs at boot; `getSendInterval()` (`wifi_manager.h:313`) returns that cached
value, and the sample loop uses it at `Line_Sight.ino:334`. So the command
persists but has no effect on the running device.

The AP-portal path gets away with the same write (`wifi_manager.h:161`) only
because it calls `ESP.restart()` immediately afterwards.

**Field evidence.** I sent `set_interval 3` to pcs1. Delivery rate before and
after was **29 readings/minute, unchanged** — if sampling had actually slowed to
3 s the device could not have continued delivering 1,740/h.

This matters beyond tidiness: with defect 1 unrecoverable, throttling the sample
rate was the **only remaining remote mitigation** for the data loss, and it is
also inoperative.

**Suggested fix.** Mirror the pattern already used by `wfstats_on`
(`heartbeat.h:201` → `setWaveformStats()`): add a `setSendInterval(int)` in
`wifi_manager.h` that writes NVS *and* assigns `_sendInterval`, and call it from
the command handler. Same applies to `set_voltage` (`heartbeat.h:186-191`),
which has the identical NVS-only shape and presumably the same latency.

## Defect 3 — `rejected_log_bytes` exceeds its own budget (LOW)

`http_sender.h:166-167` budgets `REJECTED_MAX_BYTES 50000` × (1 active +
`REJECTED_ARCHIVE_MAX 4` archives) = **250,000 bytes**. Devices report a steady
**~301,000** — about one extra file's worth over budget, on every unit.

Also worth aligning the docs: `BACKEND_NOTES.md:127` describes the field as
"cap 50,000", which reads as a total rather than the per-file cap. A server-side
alert threshold set from that line would never fire.

---

## Server side — verified healthy, no action needed there

- Live ingest: **6–8 ms** response, measured both on loopback and via the public
  IP the devices use. `last_http_code: 200` on every device.
- The `400 — saved to /rejected.log` class: **27,569 all-time, 0 in the last
  24 h**. That storm is fixed; the wreckage it left on flash is what remains.
- The `rejected-ndjson` drain endpoint is deployed, idempotent and tested
  (including malformed-line handling), so ~4.5 MB of parked readings across the
  fleet will land at their original timestamps the moment the drain can run.
- Idempotent ingest is live (unique index on `(pi_device_id, time)` +
  `ON CONFLICT DO NOTHING` on every path), so re-sends and drains are safe.

## Recommended sequencing

1. **Deactivate the 2.7 OTA release** until defect 1 is fixed — `sb1`, `sb2` and
   `aravind_cnc2` are still on 2.6 and will auto-upgrade into this state.
2. Fix defects 1 and 2; ship as 2.7.1.
3. The 15 stuck devices need that build delivered by OTA (the update path itself
   still works) or a USB reflash. A plain reboot will **not** clear it.
4. Once drained, expect the pcs fleet to return to ~3,602 readings/h.
