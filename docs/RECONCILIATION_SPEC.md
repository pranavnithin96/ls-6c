# Staged readings — monitor, measure, build only if needed

Companion to `LS02_FORMAT.md` §4 (timestamping) and the Task-2 idempotency work.

**Do not build a reconciliation engine right now.** This note replaces an earlier
over-specified version. The short version: staged data is *saved, not lost*, so
it does not gate rollout, and we don't yet know if a real reconciler is even
worth building. Measure first.

## What "staged" means

Rows the server stored but couldn't put a real timestamp on — the device hadn't
NTP-synced when it sent them (`X-Ntp-Valid:0`, or an old-boot file whose millis
base is lost). They live in `staged_readings` with the measurement intact but no
reliable epoch.

## Why it is NOT a zero-loss blocker

The data is durably stored. The zero-loss rule ("don't lose a second") is about
data being *dropped* — staging is the opposite of dropping. The only gap is that
staged rows aren't plotted on dashboards yet. That's **visibility**, not **loss**.
So this does not block the OTA rollout.

## Why new staging should be rare

A v2.7 device only stages if it uploads *before* getting NTP sync. But it won't
upload without WiFi, and once it has WiFi, NTP resolves in seconds — offline data
written pre-sync gets re-stamped at upload time (`LS02_FORMAT.md` §4). So going
forward, staging is an edge case (device stuck connected-but-unsynced, or an old
file surviving a reboot), not a routine path. The 203k rows already staged are
mostly the historical epoch-0 `mark_pdc` batch — old data with unknowable exact
times, low value to reconstruct.

## What to actually do

1. **Add one alert.** Page if `staged_readings` row count crosses a threshold or
   keeps growing. This is the only real risk: staged data silently piling up and
   later getting purged as junk. The alert makes that impossible.
2. **Measure during the USB pilots.** Track how many rows land in staging per
   device on real links.
3. **Then decide, based on the number:**
   - Essentially zero (expected) → don't build a reconciler. A small script to
     place the occasional row, or leaving them archived, is enough.
   - Meaningful ongoing staging → build reconciliation then, and only as complex
     as the measured volume justifies.

## If a reconciler ever is warranted (reference only — not now)

The whole problem is one unknown per boot session: `boot_epoch` (wall-clock of
`millis()==0`). Given it, every row's time is `boot_epoch + start_millis/1000 +
i×interval`. `boot_epoch` comes from a synced upload's re-stamp headers
(`X-Ntp-Epoch − X-Ntp-Millis/1000`) or a synced heartbeat. Insert with
`ON CONFLICT (pi_device_id, time) DO NOTHING`; mark staged rows reconciled,
never DELETE. `millis()` resets per reboot, so an anchor only applies to rows
from the *same* boot — which needs a firmware `boot_id` on uploads/heartbeats to
do safely. That firmware follow-up, not the server job, is the thing to raise
first if the pilot numbers say reconciliation is worth it.
