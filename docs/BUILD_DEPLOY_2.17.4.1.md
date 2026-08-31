# Build & Deploy — v2.17.4.1 (DC power special case)

Branch: `v2.17.4.1-dc-cal` (cut from tag `v2.17.4`, the fleet-stable release).
Adds per-channel DC count→kW calibration; no changes to the send/offline/OTA paths.
See the commit message of `2.17.4.1: DC power special case` for the feature details.

## Build

Standard Arduino build, same as every 2.x release:

```
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir ./build \
    --libraries <dir-containing-ArduinoJson> <sketch-dir>
```

- **Core:** esp32:esp32 **2.0.17** (the 2.17.4 release core — do not build this branch on 3.x)
- **Library:** ArduinoJson **7.x** (built and verified against 7.4.2)
- **Sketch dir naming:** arduino-cli requires the folder to be named `Line_Sight/`
  (matching `Line_Sight.ino`) — build from a copy/symlink if checking out as `ls-6c/`.
- Linux build hosts work out of the box (Arduino publishes native Linux ctags/esptool).
  macOS arm64 hosts need Rosetta OR shims for the Intel-only bundled `ctags` and
  `esptool` — see "macOS arm64 notes" below.

Build outputs (offsets for a raw USB flash):

| Offset  | File |
|---------|------|
| 0x1000  | `Line_Sight.ino.bootloader.bin` |
| 0x8000  | `Line_Sight.ino.partitions.bin` |
| 0xe000  | `boot_app0.bin` (from the esp32 core: `tools/partitions/boot_app0.bin`) |
| 0x10000 | `Line_Sight.ino.bin` ← **this is also the OTA image** |

## Deploy — OTA (fleet devices already on 2.x)

The device polls hourly (first check 5 min after boot; `update` over serial forces one):

```
GET http://46.224.90.187/api/firmware/check?device_id=<id>&current_version=<ver>
```

Server response to trigger the update:

```json
{ "update_available": true, "version": "2.17.4.1", "url": "http://46.224.90.187/api/firmware/ls6c-2.17.4.1.bin" }
```

- The `url` must be plain HTTP (device forces HTTP) and serve `Line_Sight.ino.bin`
  with HTTP 200, no redirects (device disables redirect following).
- Crash-rollback is built in: repeated crashes after an update roll back automatically.

### ⚠️ Version-comparison gotcha (matters for THIS release)

`isVersionGreater()` parses only **three** components (`%d.%d.%d`):

- `2.17.4.1` parses as `2.17.4` → **a device already on 2.17.4 will NOT accept
  "2.17.4.1" over OTA** (compares equal, skipped as not-greater).
- Devices on ≤2.17.3 accept it fine.
- **If the target device runs 2.17.4, publish this build under version `2.17.5`
  instead** (change `FIRMWARE_VERSION` in config.h and the response JSON — one line).
- Likewise, any future revision of this build must announce as **2.17.5+** to be
  accepted by devices running it.

## Deploy — first flash of the DC bench/production board (`pcs_21`)

That board currently runs a bare ADC test sketch with **no OTA** — its first
2.17.4.1 install must be over USB (offsets table above). Its USB bridge has no
auto-reset wiring (DTR/RTS not connected): enter download mode manually by
holding BOOT/IO0 low through a power-up. Board identity:

- Chip: ESP32-D0WD-V3, MAC `c0:cd:d6:c0:87:40`
- AP setup PIN (last 4 of MAC): **8740** — after first boot it starts the
  captive portal (`LineSights-8740`); provision WiFi + device id **`pcs_21`**.
  Server side is ready to accept `pcs_21`.
- A ready-made flasher exists on the bench Mac: `~/Applications/Flash LineSights
  v2.17.4.1.app` — drop the four bins into `Contents/Resources/firmware/` and
  double-click. After this one USB flash, all future updates are OTA.

## How a board knows it needs DC calibration

Two mechanisms, no site visit required for either:

1. **Boot-time factory default (in this firmware).** `applyDcDefaultsFor()` runs at
   startup: a board provisioned as **`pcs_21`** with no stored DC cal gets the
   provisional CH1 cal (`kW = 0.18868*count + 159.43`) applied and persisted
   automatically, and logs `[DC] CH1 PROVISIONAL cal applied`. It never overwrites
   an existing cal, so later refinement always wins. Accuracy: ~±7 kW in the
   400–500 kW band; **overreads at low load** (+159 kW at zero) — treat kWh totals
   as provisional until refined.

2. **Server push (heartbeat command).** The backend can set/refine any board's DC
   cal remotely, same channel as `set_ct_slope`:

   ```json
   {"action":"set_dc_cal","channel":1,"slope":0.18868,"offset":159.43}
   ```

   Slope+offset `0,0` clears the channel back to the stock AC path. Persisted,
   survives reboot and OTA. This is the intended path for pushing the refined fit
   once real calibration points exist.

## DC calibration (refining the provisional, over serial or on-site)

```
dcpoint <ch> <kW>   # record a point: samples the ADC at that moment, pair with
                    # the meter reading — take 4-5 across a wide load range,
                    # INCLUDING one at low/zero load (pins the offset; a positive
                    # zero-count offset becomes phantom kWh)
dcfit <ch>          # least-squares fit + R^2
dcadopt <ch>        # persist fit as the live cal (survives reboot + OTA)
dcshow / dcclear <ch> / dcset <ch> <slope> <offset>
```

Reference data already gathered on the bench (old sketch, raw counts):
count ≈1487 ↔ 440 kW (synced); count ≈1805 ↔ ~500 kW (synced, load steady).
Use as sanity checks only — record fresh `dcpoint`s for the real fit.

## macOS arm64 notes (bench Mac only — not needed on Linux)

The Arduino-bundled `ctags` and `esptool` are Intel-only. Either install Rosetta,
or shim them (`~/Library/Arduino15/packages/builtin/tools/ctags/.../ctags` →
universal-ctags is NOT output-compatible with arduino-cli's prototype generator;
prefer Rosetta, or move the sketch body into a `.cpp` to skip prototype
generation). `esptool` shims cleanly to any arm64 esptool ≥5.x.
Also: libraries under `~/Documents` may be iCloud-evicted (`dataless`) — reads
hang the build; keep build libraries outside iCloud.
