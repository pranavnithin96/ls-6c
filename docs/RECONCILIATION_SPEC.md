# Staged-readings reconciliation — spec for the server session

Companion to `LS02_FORMAT.md` (§4 timestamping, §5 error contract) and the
Task-2 idempotency work (`uq_readings_device_time` on `(pi_device_id, time)`).

This closes the last open link in the zero-loss chain. The firmware and the
bulk-ingest decoder now **preserve** bad-clock data instead of overflowing it
(C3) or colliding it on epoch 0. But preserved is not delivered: rows in
`staged_readings` are invisible to every dashboard, cycle detector, OEE query
and export until something places them on the real timeline. This document
specifies that something.

---

## 0. Why this is load-bearing (read first)

When the server stages an upload and returns **200**, the device **deletes its
only copy** (`LS02_FORMAT.md` §5). There is no re-delivery: the device is done
with that data the instant it sees the 200. Therefore:

- **The server holds the sole copy of every staged row.** If reconciliation
  drops one, it is gone from the entire system.
- The reconciler must obey the same discipline the Task-2 migration proved out:
  **mark and archive, never `DELETE`.** (Recall the 1970 chunk: 164,729 rows
  that *looked* like duplicates were 108,434 distinct measurements. Blind
  deletion would have destroyed real data.)
- The reconciler must **never inject an approximate timestamp into `readings`
  as if it were precise** — that silently poisons OEE and cycle detection.
  Approximate rows stay visibly flagged (§4, Tier B).

Not losing a row and not lying about its timestamp are equally binding here.

---

## 1. What a staged row is, and what it must carry

Two sources feed `staged_readings` (per `LS02_FORMAT.md` §4 and Task-4):

- **LS02 offline blocks** the device could not anchor at upload — `X-Ntp-Valid:0`
  (never synced) **or** `X-Anchor-Valid≠1` (block is from an *earlier boot
  session*, so its `start_millis` base is lost device-side).
- **Live JSON** posts carrying `clock_unsynced: true` (pre-first-sync boot
  window).

For any of these to be reconcilable later, each staged row **must** persist the
raw ingredients of the §4 re-stamp formula — not just a placeholder epoch.
Audit the table and confirm every row keeps:

| Column | Why it is required |
|---|---|
| `pi_device_id` | reconciliation is per-device |
| `start_millis` (block) or device `millis` (live) | the monotonic anchor — **useless if dropped** |
| `sample_index i` **or** the per-row `device_millis = start_millis + i×interval×1000` | per-row offset within the block |
| `interval` (from `X-Interval`, 1–60, default 1) | cadence between samples |
| `boot_session_key` (see §5) | which boot the millis base belongs to |
| the 6 CT centi-amp values (→ amps) + any `features` JSONB | the actual measurement, full fidelity |
| `received_at` | upper time bound + audit |
| `flags` (`clock_unsynced`, source) | tier routing |
| `state` (`pending` / `reconciled_exact` / `reconciled_approx` / `superseded`) | idempotent re-runs |

If any of `start_millis`, `interval`, or a boot-session discriminator is being
dropped at ingest today, that is a **staging bug to fix first** — without them a
row is permanently orphaned regardless of how good the reconciler is.

---

## 2. The anchor model

Every staged row's true time is, per `LS02_FORMAT.md` §4:

```
reading_epoch = boot_epoch + device_millis / 1000
              = boot_epoch + start_millis/1000 + i × interval
```

`device_millis` is known and reliable **within one boot session**. The single
unknown is **`boot_epoch`** — the wall-clock time of `millis()==0` for that boot.
Reconciliation is entirely the problem of learning `boot_epoch` per
`(device, boot_session)` and applying it.

**`millis()` resets to 0 on every reboot.** An anchor learned for boot session B
is meaningless for boot session A. Applying the wrong session's anchor produces
confidently-wrong timestamps — worse than staging. So every anchor is scoped to
a `boot_session_key` and applied **only** to rows from that same session (§5).

### Anchor table

```
device_clock_anchors(
  pi_device_id      text,
  boot_session_key  text,
  boot_epoch        bigint,      -- unix seconds of millis()==0
  source            text,        -- 'restamp_header' | 'heartbeat' | 'bounded'
  confidence        text,        -- 'exact' | 'approx'
  learned_at        timestamptz,
  PRIMARY KEY (pi_device_id, boot_session_key)
)
```

Anchors are **immutable once `exact`**. An `approx` anchor may be upgraded to
`exact` if an authoritative source later arrives; never downgrade.

### Anchor sources (in confidence order)

1. **Re-stamp headers on any upload of the same session** — `X-Ntp-Valid:1`,
   `boot_epoch = X-Ntp-Epoch − X-Ntp-Millis/1000`. **Exact.** These flow past
   ingest continuously; capture them into `device_clock_anchors` at ingest time
   even when the current upload needs no staging, so a later staged block from
   the same session can be resolved. (This is the highest-yield hook: an anchor
   often arrives on a *different* file of the same boot than the staged one.)
2. **Synced heartbeats** — once `clock_synced:true`, derive
   `boot_epoch = device_epoch − uptime_s`. **Requires the heartbeat to carry a
   boot-session key and an uptime/`millis`.** Confirm those fields exist
   (§5 flags a firmware follow-up if they do not). Exact.
3. **Bounded inference (Tier B only, §4)** — no session anchor will ever come
   (prior-boot, device long gone). Bound `boot_epoch` from neighbouring
   real-timestamped readings. **Approx.**

---

## 3. Idempotency (against `uq_readings_device_time` and against itself)

- Insert reconciled rows with **`INSERT … ON CONFLICT (pi_device_id, time) DO
  NOTHING`**. A collision means a precise live reading already owns that
  device-second — keep it, mark the staged row `superseded`. (Safe because
  `readings` is one wide row per device-second at ≥1 s interval; two distinct
  real measurements cannot legitimately share a `(device, time)`.)
- The reconciler itself must be **re-runnable with no side effects**: resolve →
  `ON CONFLICT DO NOTHING` insert → set `state`. Never `DELETE` a staged row on
  success; flip it to `reconciled_exact` / `reconciled_approx` / `superseded`
  and let a retention job archive terminal-state rows after an audit window.
- Anchors are keyed and immutable (§2), so replaying an anchor is a no-op.

Running the job twice back-to-back must produce **zero** new rows and **zero**
state changes on the second pass. Make that an acceptance test (§7b).

---

## 4. The two tiers — exact vs. honest-approximate

**Tier A — a same-session anchor exists (or arrives).** Compute the exact epoch
per §2, insert `ON CONFLICT DO NOTHING`, mark `reconciled_exact`. These become
first-class readings indistinguishable from live ones. This is the common case
whenever a device syncs *at all* during the boot that produced the data.

**Tier B — orphaned: no same-session anchor will ever come** (prior-boot
unsynced data whose `boot_epoch` the device discarded on reboot; e.g. the
epoch-0 RTC pattern). The exact time is unrecoverable. Do **not** drop it and do
**not** fabricate a precise time. Instead:

- Bound the block's real-time window: `earliest = ` the newest real reading from
  the same device strictly *before* this data could have been recorded (or the
  device's prior known-good time); `latest = received_at`. Preserve the block's
  internal spacing (`interval`) inside that window.
- **If the window is tight** — width ≤ block duration + a small margin (the block
  clearly slots into one known gap) — promote to `readings` with
  `features.time_approximate = true` and `features.time_bound_s = <window
  width>`, `ON CONFLICT DO NOTHING`, mark `reconciled_approx`. Downstream
  consumers that care about precision can exclude on the flag; nothing is hidden.
- **If the window is loose** — keep the row in `staged_readings`, but expose it
  through a **`recovered_readings` view / export** flagged `unplaced`, so it is
  *visible and retrievable* (satisfying zero-loss) without contaminating precise
  time-series analytics. Never silently strand it.

The product decision of how tightly to blend approximate rows into OEE is the
server session's call; the invariant is only this: **every staged row ends in a
terminal state that is either exact, flagged-approximate, or visibly-unplaced —
never dropped, never silently precise.**

---

## 5. The boot-session identity gap (the honest hard part + firmware follow-up)

Everything above hinges on a reliable `boot_session_key` so an anchor is matched
to the rows it actually belongs to. Confirm what the firmware supplies today:

- **If uploads and heartbeats already carry a per-boot id** (a random value or a
  monotonic boot counter fixed at boot), use it directly as `boot_session_key`.
- **If they do not**, the only *safe* same-session anchoring is when the anchor
  and the staged block arrive on the **same upload** (same `X-Ntp-*` context),
  which severely limits Tier A and pushes most cross-arrival cases to Tier B.
  Do **not** guess a session from `received_at` proximity — a device can upload
  an old-session file and a current-session file seconds apart.

**Firmware follow-up (v2.7.x, worth doing — it shrinks Tier B):**

1. Emit a `boot_id` (random 32-bit, fixed at boot) on every upload header and
   every heartbeat. This alone makes cross-arrival Tier-A anchoring safe.
2. Better: once NTP locks, **persist the resolved `boot_epoch` into the open
   offline file's header** so a file that outlives the reboot self-anchors and
   never needs server-side inference at all.

Flag these back to me and I'll scope them into a firmware point release. Until
(1) ships, size Tier B honestly in the metrics (§6) rather than assuming it is
small.

---

## 6. Operational shape

- **Trigger:** run periodically (e.g. every 5 min) **and** opportunistically
  whenever a new `exact` anchor is written (an anchor arrival can resolve a
  backlog of that device's pending rows immediately).
- **Batch per device**, oldest `pending` first.
- **Metrics (emit every run):** `staged_pending`, `reconciled_exact`,
  `reconciled_approx`, `superseded`, and an **age histogram of Tier-B orphans**.
- **Alerts:** oldest `pending` row exceeds a threshold (e.g. 24 h) → the data is
  captured but still invisible, which under the zero-loss rule is a page-worthy
  condition, not a silent backlog. Also alert if `staged_pending` grows faster
  than it drains (a device stuck pre-sync, or a staging bug).

---

## 7. Acceptance tests

Reuse the firmware-identical encoder (`ls02_host_test.c`) to fabricate bodies.

- **a. Exact reconcile.** Stage an `X-Anchor-Valid:0` block, then deliver a
  same-session `exact` anchor (a later upload with `X-Ntp-Valid:1`, or a synced
  heartbeat). Run the job → the block's rows appear in `readings` at
  `boot_epoch + start_millis/1000 + i×interval`, staged rows → `reconciled_exact`.
- **b. Idempotent.** Run the job again immediately → **0** new rows, **0** state
  changes.
- **c. Collision / superseded.** Reconcile a block whose computed second already
  holds a live reading → `ON CONFLICT DO NOTHING` keeps the live row, staged →
  `superseded`, no duplicate.
- **d. Tight-bound approx.** Orphan block that slots into one known gap →
  promoted with `features.time_approximate=true` + `time_bound_s`, retrievable,
  excluded from a precision-only query by the flag.
- **e. Loose-bound unplaced.** Orphan with no anchor and a wide window → stays in
  `staged_readings`, surfaces in `recovered_readings` as `unplaced`, **never
  dropped**.
- **f. Regression — the 1970 archive set.** The `mark_pdc` epoch-0 rows already
  archived by the Task-2 migration must be treatable as Tier-B orphans
  (visible/approximate), **not** resurrected into `readings` at epoch 0 or any
  fabricated precise time.

---

## 8. Where this sits in the rollout gate

This is the last zero-loss item before broad rollout. Sequence:

1. Ship the reconciler (Tier A exact + Tier B visible) — un-hides the 203,014
   rows already staged and every future pre-sync window.
2. USB pilots in parallel — measure real `staged_pending` growth per device;
   that tells us how large Tier B actually is on live links and whether the
   firmware `boot_id` follow-up (§5) is urgent or cosmetic.
3. Only then publish v2.7.0 to the OTA registry (fleet rollout).
