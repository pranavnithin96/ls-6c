# LineSights LS-6C-IOT firmware

This branch is the v3 full refactor of the ESP32 six-channel power monitor. Its
primary invariant is simple: a network failure may delay a reading, but may not
decide whether that reading survives.

## Design at a glance

- Core 1 samples the six CT channels and synchronously commits each completed
  reading to a CRC32 write-ahead log (WAL).
- Core 0 owns Wi-Fi, live delivery, bulk recovery, heartbeat, and OTA.
- A reading leaves the WAL only after a live HTTP 200 or after a verified copy
  has been appended to an LS02 bulk file.
- LS02 and legacy JSON files are immutable while uploading and are deleted only
  after HTTP 200.
- LittleFS is mounted without auto-format. Corrupt or interrupted artifacts are
  retained for forensic recovery.
- Server ingest must be idempotent. Power loss can cause a safe duplicate replay;
  it must never create a gap.

See [docs/ARCHITECTURE_V3.md](docs/ARCHITECTURE_V3.md) for invariants and crash
states, and [docs/TEST_PLAN_V3.md](docs/TEST_PLAN_V3.md) before any field rollout.
The older v2 documents remain as compatibility references for LS01/LS02 and the
backend contract; they are not the v3 implementation guide.

## Build

The checked-in sketch keeps its historical `Line_Sight.ino` name. Arduino CLI
requires the primary `.ino` to match its containing directory, so CI/bench builds
should stage it as `ls_6c/ls_6c.ino` and then run:

```sh
arduino-cli compile --warnings all --fqbn esp32:esp32:esp32 ls_6c
```

The current toolchain target is ESP32 Arduino core 2.0.17 with ArduinoJson 7.x.

## Host tests

```sh
python3 -m unittest discover -s tests -v
```

## Operational limits

Internal flash makes every completed sample power-loss recoverable after its WAL
commit. It cannot provide infinite outage retention or guarantee a sample during
a total power failure. If the filesystem reserve is exhausted, firmware stops
advancing the acquisition sequence, reports `capacity_blocked`, and counts any
missed capture windows. A true indefinite, power-independent guarantee requires
additional nonvolatile capacity (for example FRAM) or backup power.
