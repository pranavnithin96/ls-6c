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

## Resolution path — fix forward in 2.8

Decision (Pranav, 2026-07-21): no 2.7.1 patch, these fixes land in **2.8**.

**Status now:**

- **2.7 has been DEACTIVATED in the OTA registry** (server session, 2026-07-21
  ~05:10 UTC). `/api/firmware/check` now answers `{"update_available": false}`
  to everyone, so `sb1`, `sb2` and `aravind_cnc2` — still on 2.6 — can no longer
  auto-upgrade into the wedged state. Reversible at any time; publishing 2.8
  with `set_active` supersedes it automatically.
- The 15 pcs devices stay degraded (~53% loss) until 2.8 ships. This is
  degradation, not blindness — ~1,700 readings/h is still one every ~2 s, which
  cycle detection tolerates.

**The OTA channel still reaches the wedged devices — this is the recovery path.**
Verified 2026-07-21: every stuck pcs unit is still polling `/api/firmware/check`
(6 polls each today, most recent 05:07 UTC). The update path is independent of
the jammed send ring, so activating 2.8 will reach them and they will self-heal
on the next poll. **No USB reflash and no site visit are required**, provided 2.8
clears `/rejected.log` (or fixes the gate so the existing drain can).

**Sequencing:**

1. Build 2.8 with defects 1 and 2 fixed (defect 3 optional).
2. Upload to the OTA registry and set active — that re-enables updates fleet-wide
   and supersedes the deactivated 2.7 in one step.
3. Stuck devices update on their next poll; expect the pcs fleet to return to
   ~3,602 readings/h once their logs drain.
4. Consider whether 2.8 should clear a full `/rejected.log` on first boot as a
   one-shot recovery, rather than relying on the drain gate to unwedge itself —
   these devices already have ~301 KB parked, and the parked rows are largely
   same-second duplicates the server now no-ops anyway (idempotent ingest is
   live), so discarding them costs little and guarantees recovery.

**No server-side changes are required for 2.8.** The LS02 decoder, the
`rejected-ndjson` drain endpoint, idempotent ingest and all v2.7 heartbeat
fields/commands are deployed and tested; 2.8's fixes are internal to the
firmware. If 2.8 changes the wire format or adds heartbeat fields, say so and the
server side will follow.

---

# 2.8.0 follow-up — 2026-07-21, after rollout

**The 2.8 backstop works. The underlying cause was misdiagnosed (by me, in the
section above) and is still open.**

## What 2.8 fixed — confirmed in the field

The first-boot recovery backstop does exactly what it promised. Devices that
took 2.8 and can keep up are now clean:

| Device | fw | `rejected_log_bytes` | sent/h | dropped/h |
|---|---|---:|---:|---:|
| mark_pdc | 2.8.0 | **0** | 3,593 | 0 |
| mark_new_pdc | 2.8.0 | **0** | 3,621 | 1 |
| mark_new_1 | 2.8.0 | **0** | 3,621 | 1 |
| meton_04 | 2.8.0 | 462 | 3,595 | 7 |

## What is still broken — corrected analysis

**Retraction:** an earlier revision of this section claimed the full
`/rejected.log` was only a symptom, citing Vasa as a single-device site that was
also slow. **Vasa does not run this hardware** — it has no ESP32 telemetry at all
and should never have been in the comparison. With ESP32-only devices, the
evidence points the other way.

**Within a single site, with the network held constant and all units on 2.8.0,
throughput tracks `rejected_log_bytes` monotonically:**

| Mark device | RSSI | `rejected_log_bytes` | sent/h |
|---|---:|---:|---:|
| mark_new_pdc | −52 | **0** | **3,620** |
| mark_new_1 | −76 | **0** | **3,620** |
| mark_pdc | −53 | **0** | 3,594 |
| mark_04 | −72 | 262,862 | 3,270 |
| mark_pdc1 | −76 | 300,341 | 2,950 |

Same gradient at Aravind (61 KB → 3,528; 70 KB → 3,508; 187 KB → 3,359).

Crucially this is **not signal strength**: mark_new_1 at −76 dBm matches
mark_new_pdc at −52 dBm exactly, and the PC Sons units span −46 to −76 dBm while
all sitting at ~1,700/h. A full log slows the device; a cleared one does not.

**So 2.8's backstop worked, and the devices that could keep up stayed clean.**
The three Mark units at 0 bytes are the proof. The open question is narrower than
I framed it: **why do some units overflow in the first place**, refilling the log
the backstop cleared?

Two things the data shows and one it does not:

- **Shown:** PC Sons carries an *additional* penalty beyond log size — its units
  sit at ~1,700/h while mark_pdc1 and sb1 manage 2,950 and 2,697 at the same
  ~301 KB. Something site-specific compounds it (15 devices is the obvious
  candidate, though this is not proven).
- **Shown:** two units are separately sick on weak signal — aravind_03 (−90 dBm,
  547/h) and sb2 (−87 dBm, 537/h, still on 2.6). Those are a different problem.
- **Not shown:** a single isolated root cause for the initial overflow. I do not
  have RTT-per-site measurements from the device side, and the server cannot see
  them.

## Root cause: no HTTP connection reuse

`http_sender.h` opens and tears down a TCP connection for **every single
reading**:

```
:982   http.begin(_httpServerUrl);   ... POST ...
:986   http.end();
:1023  http.begin(_httpServerUrl);   ... POST ...
:1027  http.end();
```

There is **no `http.setReuse(true)`** anywhere in the file. So each 1 Hz reading
pays a full TCP handshake + POST + response + teardown. On a site with ~200 ms
RTT that is ~500–600 ms of the 1 s budget spent on connection setup, and any
jitter pushes it over — the ring backs up, overflow lands in `/rejected.log`, the
log fills, and the amplifier described above kicks in.

Sites with a fast path to the server (Mark, Meton) stay under budget and run
clean at 3,600/h. Sites with a slower path never do.

**Suggested fix: `http.setReuse(true)`** on the live-POST client so the TCP
connection persists across readings. This removes one full round-trip per reading
and is the cheapest available headroom — it does not depend on which of the
theories above is right, because every device pays this cost on every reading. The server supports keep-alive (nginx
upstream keepalive is already deployed).

If that is not sufficient on the slowest sites, the structural fix is to **batch
N readings per live POST** (the LS02 bulk path already proves the server accepts
batches), rather than one HTTP request per second per device.

## Server side

Unchanged and healthy: 6–8 ms ingest, `last_http_code: 200` fleet-wide, no rate
limiting in nginx, idempotent ingest live. The `rejected-ndjson` drain endpoint
has still never been exercised by a real device — the 2.8 backstop clears the
log locally rather than uploading it, so those parked readings are being
discarded rather than recovered. That is the accepted trade (Pranav: recovery
must be certain), but worth stating plainly: **the drain path remains unproven in
production.**

---

# Resolution — 2026-07-26

Both open questions above are now answered with measurements.

## "PC Sons carries an additional penalty ... still unexplained"

It is **latency**, and the site is not bandwidth-constrained in any sense.

- Whole site consumes **75 kbps** across 19 devices (live + drain) — about 1% of
  the link. Adding bandwidth would change nothing.
- Kernel TCP stats to their gateway: **RTT 500–800 ms, rttvar 170–590 ms**
  (healthy is 20–50 ms). High and very jittery.
- The live path sends **one reading per HTTP request**, serialised on the pooled
  connection, so each device is capped at one reading per round trip. All 19
  devices converge on **0.589–0.600 readings/s** — a 2% spread, which is the
  signature of a deterministic shared limit rather than congestion.

So the per-site "delivery capacity" the report identified is really
round-trips-per-second, not throughput. That is why a single-device site can be
slow while another single-device site is fast: it tracks path latency, not load.

## "The drain path remains unproven in production"

Comprehensively proven, and load-bearing. On the same link, same hour:

| Transport | Readings per request | Share of PC Sons' data |
|---|---|---|
| live | 1 | ~60% |
| bulk drain | 91 | ~40% |

Those sum to 1.0 readings/s/device — the 99.9% the site was achieving. The drain
was never a safety net there; it was carrying 40% of production using 0.7% of
the requests, precisely because it batches.

That also made the site fragile in a way nobody could see. 2.11.1 shrank the
rejected-log rotation 50KB → 8KB (to fit units inside a since-removed flat
deadline), and because **the drain is rate-limited per FILE**, recovery
throughput fell with file size: readings-per-file 91 → 35, recovery 27k → 19.3k
readings/h, and site delivery 99.9% → 88%. Reverting the constant in 2.12.0
restored it (per-file back to ~95, delivery ~99.3%). The invariant is now
documented at the constant itself.

## Consequence for the batching proposal

The recommendation above — N readings per live POST — is correct and is now
quantified: at 10 readings/request a device delivers ~6 readings/s against the
1/s it needs, so a 600 ms RTT stops mattering entirely. This is the difference
between PC Sons balancing on a knife edge at 99.9% and having six times the
headroom it needs. Sampling stays 1 Hz; only delivery is grouped.
