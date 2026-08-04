# v3 bench and rollout test plan

No field deployment should begin until every automated and bench test below has
passed against a backend that implements the LS02/idempotency contract.

## 1. Automated gates

1. Run `python3 -m unittest discover -s tests -v`.
2. Compile with warnings enabled for `esp32:esp32:esp32`.
3. Confirm the source guard finds no `LittleFS.begin(true)`, destructive rejected
   data command, or old header-global implementation.
4. Run the existing C/Python LS02 cross-language round trip in `docs/`.

## 2. Normal delivery

1. Boot with working Wi-Fi and NTP.
2. Confirm one increasing `(boot_id, sequence)` per configured interval.
3. Confirm every sample logs `durable` before the server ACK appears.
4. After ten ACKed rows, confirm WAL depth returns toward zero without creating
   an offline block.
5. Reboot and confirm boot ID increments while sequence restarts at one.

## 3. Spotty Wi-Fi and lost ACKs

1. Introduce packet loss, DNS failure, TCP resets, and HTTP timeouts independently.
2. Confirm WAL grows and then checkpoints into LS02; `permanent_losses` stays zero.
3. Drop a server HTTP 200 on the return path. Confirm the reading/file is replayed
   and server idempotency returns 200 for the duplicate.
4. Alternate connectivity every 2–5 seconds for at least one hour. Reconcile all
   generated identities against live plus bulk ingest; there must be no gap.

## 4. Power-cut matrix

Cut power repeatedly at each point: during WAL append, after append verification,
during LS02 append, after LS02 verification, after `.next` creation, after the WAL
to `.recovery` rename, after `.next` becomes the WAL, during bulk upload, and after
server storage but before local deletion. On every reboot:

- all fully committed identities must exist in WAL, LS02, or server storage;
- a corrupt WAL tail must be archived, never erased;
- duplicates are acceptable and must be server-deduplicated;
- `permanent_losses` must remain zero unless physical capacity was exhausted.

## 5. Migration

For each source artifact (`LS01`, `LS02`, `buffer.json`, `rejected.log`, numbered
legacy/rejected files), install it on LittleFS before flashing v3. Confirm:

- bytes remain present until an HTTP 200;
- pre-boot `offline.dat` uploads with `X-Anchor-Valid: 0`;
- body magic chooses LS01 versus LS02 decoding;
- legacy JSON converts to NDJSON without deleting its source first;
- a 4xx/5xx or transport error changes no source file.

## 6. Capacity and corruption

1. Fill LittleFS to each side of the 32 KiB reserve.
2. Confirm the last accepted reading verifies correctly and the next is held,
   with `storage_state=capacity_blocked` and a fault LED.
3. Restore space by delivering an immutable backlog file with HTTP 200. Confirm
   the held reading commits with the same identity.
4. Corrupt every byte position of a WAL record in separate runs. Confirm CRC32
   detects it and the original file is retained in a quarantine slot.
5. Corrupt/truncate LS02 blocks and confirm the backend follows the existing
   good-prefix/staged-tail 200 contract.

## 7. OTA

1. Reject missing MD5, wrong MD5, wrong size, downgrade, and oversized image.
2. Interrupt the download at multiple offsets; running firmware and WAL must remain.
3. Install a valid image, then crash it before five healthy minutes; confirm the
   bootloader rollback path.
4. Let a valid image run healthy for five minutes; confirm it is marked valid.
5. Build backlog during OTA and reconcile it after reboot.

## 8. Pilot and rollout

1. Deploy the compatible backend first.
2. Run two or three USB-observed devices for 72 hours on representative weak links.
3. Alert on `permanent_losses > 0`, non-ready storage, low filesystem reserve,
   growing quarantine bytes, stale NTP, and increasing last-success age.
4. Reconcile device generated/durable counters against server live+bulk rows.
5. Expand rollout only after the pilot shows zero missing identities and expected
   duplicate rates.
