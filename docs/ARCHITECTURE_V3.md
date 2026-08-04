# v3 architecture and durability contract

## Ownership state machine

Every completed reading has the stable identity `(boot_id, sequence)` and moves
through these states:

```text
sampled in RAM
    -> WAL append + flush + read-back CRC verification
    -> live POST pending
       -> HTTP 200 -> WAL checkpoint may retire it
       -> no ACK   -> checkpoint copies its contiguous group to LS02
    -> immutable bulk file
       -> HTTP 200 -> file may be removed
       -> anything else -> file remains byte-for-byte unchanged
```

The only RAM-only interval is the time needed to acquire a 500 ms sample and
append its WAL record. Once `commit()` returns, Wi-Fi, reboot, task failure, and
lost HTTP responses can cause duplicates but not data loss.

## Write-ahead log

`/v3.wal` contains packed 42-byte records:

| Field | Size |
|---|---:|
| magic `WAL3` | 4 |
| boot ID, sequence, epoch, sample millis | 16 |
| voltage, sample duration | 4 |
| interval, flags | 2 |
| six signed centi-amp values | 12 |
| CRC32 | 4 |

Each append is flushed, closed, reopened, and read back. A partial/corrupt tail is
archived as `/v3.wal.corrupt.N`; the valid prefix is rebuilt and remains active.
The archive is not automatically deleted.

## Checkpoint crash states

Ten WAL records form one compatibility checkpoint. If all ten have confirmed
live ACKs, no bulk copy is required. If any ACK is uncertain, all ten are appended
as one time-contiguous LS02 block so the server can deduplicate confirmed rows.

WAL compaction uses `/v3.wal.next` and `/v3.wal.recovery`:

1. Write and verify the remaining WAL tail as `.next`.
2. Rename the original WAL to `.recovery`.
3. Rename `.next` to the live WAL.
4. Verify the live WAL, then remove `.recovery`.

A reset before step 2 replays the original. A reset between steps 2 and 3 restores
the original. A reset after step 3 uses the committed tail because the retired
prefix was already ACKed or durably copied to LS02. Replays can duplicate rows;
they cannot omit one.

## Compatibility

- Existing `offline.dat` is rotated at boot and never appended across boot
  sessions, preserving the LS02 millis-anchor rule.
- `offline.legacy.dat`, numbered legacy files, and `offline.rejected.N.dat` are
  offered to the bulk endpoint oldest-first. The server dispatches using the body
  magic, not the advisory `X-Format` header.
- Existing `buffer.json` is converted to a separate immutable NDJSON upload file.
  The original remains until that upload receives HTTP 200.
- Existing `rejected.log` and numbered rejected logs are rotated and drained as
  immutable NDJSON. There is no clear/dispose command.
- NVS namespace `lscfg` and keys for Wi-Fi, device metadata, voltage, interval,
  CT ratings, and timezone are unchanged. Per-channel slopes remain in `ctcal`.

## Concurrency

- Core 1: ADC acquisition, WAL commit, serial control, LED, watchdog.
- Core 0: Wi-Fi state, NTP, live/bulk HTTP, heartbeat, OTA.
- `DurableQueue` is the only cross-core data owner and serializes its metadata and
  filesystem transitions with one FreeRTOS mutex.
- Network code only streams immutable files after releasing that mutex.
- Plain-HTTP bulk streams use non-blocking socket writes with a 12-second
  no-progress guard, a five-minute total cap, and a separate 60-second response
  tail wait. Acquisition continues into WAL while a slow upload is in flight.

## Commands and deletion policy

Supported heartbeat commands are `set_interval`, `set_voltage`,
`set_ct_slope`, `update_firmware`, and `reboot`. Legacy remote
`clear_rejected` and `factory_reset` commands are deliberately unsupported.

OTA requires a newer semantic version, exact content length, and a 32-character
MD5 supplied by the update service. A new image stays pending for five healthy
minutes before it is marked valid. MD5 protects against corruption, not a hostile
update server; production OTA should use authenticated HTTPS or a signed manifest.

## Capacity behavior

The firmware reserves 32 KiB of LittleFS headroom and budgets 550,000 bytes for
offline LS02 data. It never overwrites old telemetry to make room. When capacity
is exhausted it holds the pending reading, exposes `capacity_blocked`, and counts
capture windows that could not occur. This is intentionally loud and fail-closed.

For outages longer than internal flash capacity, add storage or change hardware.
Silently lowering cadence or overwriting the oldest data would violate the product
requirement and is not performed automatically.
