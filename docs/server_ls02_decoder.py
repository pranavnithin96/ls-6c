"""
Reference decoder for LineSights offline bulk uploads — LS01 (v2.6) and LS02 (v2.7.0).

Drop-in reference for the backend ingest route (POST /api/data/bulk). It is
deliberately framework-agnostic: `decode_bulk_upload(body, headers)` takes the
raw request bytes + headers and returns a result you map onto your DB + HTTP
response. See LS02_FORMAT.md for the authoritative format.

CONTRACT (do not weaken — the device deletes its only copy on 200 and quarantines
on any 4xx, so your status code is a data-loss lever):
  * Dispatch on the 4-byte BODY magic, never the X-Format header (rollback case).
  * Decode every block you can. CRC fail / truncation -> ingest the good prefix,
    stage the bad tail, still return 200.
  * Never 4xx clock_unsynced / 1970 rows -> stage them.
  * Reserve non-200 for "stored nothing" (DB down) so the device retries.
"""

import json
import struct
import zlib

FILE_HEADER = struct.Struct("<4s32sI")        # magic, device_id, file_epoch   = 40 bytes
BLOCK_HEADER = struct.Struct("<IIBHH")        # start_epoch, start_millis, flags, size_flags, crc16 = 13 bytes
READING = struct.Struct("<6h")                # 6 x int16 centi-amps           = 12 bytes
RAW_FLAG = 0x8000
LEN_MASK = 0x7FFF
FLAG_CLOCK_UNSYNCED = 0x01
CENTI = 100.0


def _inflate_raw(payload: bytes) -> bytes:
    """Raw-deflate (wbits=-15, matches the ESP32's tdefl). flush() matters:
    decompress() alone may hold back tail bytes and silently return a short
    buffer, which the %12 check would then misread as a corrupt block."""
    d = zlib.decompressobj(-15)
    return d.decompress(payload) + d.flush()


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class Reading:
    __slots__ = ("device_id", "epoch", "amps", "clock_unsynced")

    def __init__(self, device_id, epoch, amps, clock_unsynced):
        self.device_id = device_id          # str
        self.epoch = epoch                  # int (unix seconds) or None if unresolved
        self.amps = amps                    # list[float] length 6
        self.clock_unsynced = clock_unsynced  # bool -> route to staging when True


class DecodeResult:
    def __init__(self):
        self.format = None                  # "LS01" | "LS02"
        self.device_id = None
        self.readings = []                  # list[Reading] -> ingest (clock_unsynced ones go to staging)
        self.quarantine_offset = None       # byte offset where decoding stopped early (CRC/truncation), else None
        self.quarantine_bytes = b""         # the undecodable tail, kept (never dropped)
        self.notes = []

    def http_response(self):
        """Map to the (status, json) the route should return. Always 200 unless
        truly nothing was stored — see CONTRACT."""
        return 200, {
            "format": self.format,
            "ingested": sum(1 for r in self.readings if not r.clock_unsynced),
            "staged": sum(1 for r in self.readings if r.clock_unsynced),
            "quarantined_tail_bytes": len(self.quarantine_bytes),
            "notes": self.notes,
        }


def _ntp_reference(headers):
    """Returns (boot_epoch, valid). boot_epoch lets us re-stamp clock_unsynced blocks.

    X-Anchor-Valid gates the whole re-stamp: the millis arithmetic only holds when
    the blocks were written in the SAME boot session as this upload's X-Ntp-Millis.
    The device sends "1" only for its current-session file; legacy/recovered files
    (rotated at boot, quarantined, or recovered) get "0" -> stage, never re-stamp.
    Treat a missing header (older firmware) as "0"."""
    def h(name):
        return headers.get(name) or headers.get(name.lower())
    try:
        anchor = str(h("X-Anchor-Valid")).strip() == "1"
        valid = str(h("X-Ntp-Valid")).strip() == "1"
        ntp_epoch = int(h("X-Ntp-Epoch"))
        ntp_millis = int(h("X-Ntp-Millis"))
        if anchor and valid and ntp_epoch > 0:
            return ntp_epoch - (ntp_millis // 1000), True
    except (TypeError, ValueError):
        pass
    return None, False


def _send_interval(headers) -> int:
    """Seconds between readings (X-Interval). Positional stamping is
    interval-relative, not fixed 1Hz — the device's send interval is
    runtime-configurable 1-60s. Missing/invalid header (older firmware) -> 1."""
    try:
        v = int(headers.get("X-Interval") or headers.get("x-interval") or 1)
        return v if 1 <= v <= 60 else 1
    except (TypeError, ValueError):
        return 1


def decode_bulk_upload(body: bytes, headers: dict) -> DecodeResult:
    res = DecodeResult()

    # Rejected-log drain (heartbeat "upload_rejected_log"): NDJSON, one live-payload
    # JSON per line. These are readings the server previously 400'd (e.g. duplicate
    # timestamps) — ingest them IDEMPOTENTLY (upsert on device_id+timestamp; a
    # duplicate is a no-op 200, never a 4xx: the device deletes its copy on 200).
    # Dispatched BEFORE the binary length guard: an ndjson body has no 40-byte
    # minimum, and a short-circuit 200 here would make the device delete its copy.
    fmt_hdr = str(headers.get("X-Format") or headers.get("x-format") or "")
    if fmt_hdr == "rejected-ndjson" or body[:1] in (b"{", b"["):
        res.format = "rejected-ndjson"
        _decode_rejected_ndjson(body, headers, res)
        return res

    if len(body) < FILE_HEADER.size:
        res.notes.append("body shorter than file header")
        return res

    magic, dev_raw, file_epoch = FILE_HEADER.unpack_from(body, 0)
    device_id = dev_raw.split(b"\x00", 1)[0].decode("ascii", "replace")
    res.device_id = device_id

    if magic == b"LS02":
        res.format = "LS02"
        _decode_ls02(body, headers, device_id, res)
    elif magic == b"LS01":
        res.format = "LS01"
        _decode_ls01(body, device_id, file_epoch, _send_interval(headers), res)   # existing legacy path; stub below
    else:
        # Unknown magic. Do NOT 4xx the device into deleting it — keep everything.
        res.notes.append(f"unknown magic {magic!r} — quarantined whole body")
        res.quarantine_offset = 0
        res.quarantine_bytes = body
    return res


def _decode_ls02(body, headers, device_id, res):
    boot_epoch, ntp_valid = _ntp_reference(headers)
    interval = _send_interval(headers)
    off = FILE_HEADER.size
    n = len(body)

    while off + BLOCK_HEADER.size <= n:
        start_epoch, start_millis, flags, size_flags, crc = BLOCK_HEADER.unpack_from(body, off)
        payload_len = size_flags & LEN_MASK
        payload_off = off + BLOCK_HEADER.size

        if payload_off + payload_len > n:
            res.notes.append(f"truncated block at offset {off}")
            res.quarantine_offset = off
            res.quarantine_bytes = body[off:]
            return

        payload = body[payload_off:payload_off + payload_len]

        # CRC bounds the rollback-append hazard: garbage decoded as a block header
        # will fail here, so we stop and keep the tail rather than emit nonsense.
        if crc16_ccitt(payload) != crc:
            res.notes.append(f"crc mismatch at offset {off} — stopping, tail quarantined")
            res.quarantine_offset = off
            res.quarantine_bytes = body[off:]
            return

        raw = payload if (size_flags & RAW_FLAG) else _inflate_raw(payload)
        if len(raw) % READING.size != 0:
            res.notes.append(f"block at {off} payload not a multiple of 12 — tail quarantined")
            res.quarantine_offset = off
            res.quarantine_bytes = body[off:]
            return

        clock_unsynced = bool(flags & FLAG_CLOCK_UNSYNCED)
        count = len(raw) // READING.size
        for i in range(count):
            centi = READING.unpack_from(raw, i * READING.size)
            amps = [c / CENTI for c in centi]

            if not clock_unsynced and start_epoch > 0:
                epoch = start_epoch + i * interval
                unresolved = False
            elif boot_epoch is not None:
                epoch = boot_epoch + (start_millis // 1000) + i * interval   # re-stamp from NTP reference
                unresolved = False
            else:
                epoch = None                                       # stage; reconcile later
                unresolved = True

            res.readings.append(Reading(device_id, epoch, amps, clock_unsynced or unresolved))

        off = payload_off + payload_len


def _decode_rejected_ndjson(body, headers, res):
    """Each line is the exact JSON the device would have POSTed to /api/data —
    route it through the normal live-ingest logic, but idempotently. Unparseable
    lines are quarantined (kept), never dropped."""
    bad = []
    for line in body.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            doc = json.loads(line)
            res.device_id = res.device_id or doc.get("device_id")
            cts = (doc.get("readings") or {}).get("cts") or {}
            amps = [float((cts.get(f"ct_{i+1}") or {}).get("amps", 0.0)) for i in range(6)]
            ts = str(doc.get("timestamp", ""))
            unsynced = bool(doc.get("clock_unsynced")) or ts.startswith("1970")
            epoch = None
            if not unsynced:
                import datetime
                epoch = int(datetime.datetime.strptime(
                    ts[:19], "%Y-%m-%dT%H:%M:%S").replace(tzinfo=datetime.timezone.utc).timestamp())
            res.readings.append(Reading(doc.get("device_id", ""), epoch, amps, unsynced or epoch is None))
        except (ValueError, KeyError, TypeError):
            bad.append(line)
    if bad:
        res.notes.append(f"{len(bad)} unparseable rejected-log lines quarantined")
        res.quarantine_bytes = b"\n".join(bad)


def _decode_ls01(body, device_id, file_epoch, interval, res):
    """LEGACY v2.6 decoder — wire this to your existing LS01 path.

    LS01 differs from LS02: NO per-block epoch/millis/flags/crc. Each block is
    [uint16 size_flags][payload]; amps are int16 MILLIamps (/1000), and timestamps
    are positional from the single file_epoch (file_epoch + global_reading_index).
    Keep your current production implementation; this stub documents the shape.
    """
    off = FILE_HEADER.size
    n = len(body)
    global_idx = 0
    while off + 2 <= n:
        (size_flags,) = struct.unpack_from("<H", body, off)
        payload_len = size_flags & LEN_MASK
        payload_off = off + 2
        if payload_off + payload_len > n:
            res.quarantine_offset = off
            res.quarantine_bytes = body[off:]
            return
        payload = body[payload_off:payload_off + payload_len]
        raw = payload if (size_flags & RAW_FLAG) else _inflate_raw(payload)
        for i in range(len(raw) // READING.size):
            milli = READING.unpack_from(raw, i * READING.size)
            amps = [m / 1000.0 for m in milli]                     # LS01 = milliamps
            epoch = (file_epoch + global_idx * interval) if file_epoch > 0 else None
            res.readings.append(Reading(device_id, epoch, amps, file_epoch == 0))
            global_idx += 1
        off = payload_off + payload_len


if __name__ == "__main__":
    # Self-test 1: one synced LS02 block round-trips.
    payload = READING.pack(*([1234] * 6))                          # 12.34 A x6
    bh = BLOCK_HEADER.pack(0x6688A0C0, 1_000_000, 0, len(payload) | RAW_FLAG, crc16_ccitt(payload))
    fh = FILE_HEADER.pack(b"LS02", b"dev01", 0x6688A0C0)
    body = fh + bh + payload
    out = decode_bulk_upload(body, {"X-Ntp-Valid": "0"})
    assert out.format == "LS02" and len(out.readings) == 1
    assert abs(out.readings[0].amps[0] - 12.34) < 1e-6
    assert out.readings[0].epoch == 0x6688A0C0
    print("ok:", out.http_response())

    # Self-test 2: unsynced block + same-session anchor -> re-stamped and ingested.
    bh_u = BLOCK_HEADER.pack(0, 5_000, FLAG_CLOCK_UNSYNCED, len(payload) | RAW_FLAG, crc16_ccitt(payload))
    body_u = fh + bh_u + payload
    hdrs = {"X-Ntp-Valid": "1", "X-Anchor-Valid": "1", "X-Ntp-Epoch": "1000000", "X-Ntp-Millis": "60000"}
    out = decode_bulk_upload(body_u, hdrs)
    assert out.readings[0].epoch == 1000000 - 60 + 5   # boot_epoch + start_millis/1000
    print("ok (re-stamp):", out.http_response())

    # Self-test 3: same unsynced block but STALE anchor (legacy/rotated file,
    # X-Anchor-Valid=0) -> must be staged with no epoch, never re-stamped.
    hdrs_stale = dict(hdrs, **{"X-Anchor-Valid": "0"})
    out = decode_bulk_upload(body_u, hdrs_stale)
    assert out.readings[0].epoch is None and out.readings[0].clock_unsynced
    assert out.http_response()[1]["staged"] == 1
    print("ok (stale anchor staged):", out.http_response())

    # Self-test 4: COMPRESSED block (the common production case — raw deflate,
    # same wbits=-15 stream the ESP32's tdefl emits). 10 readings, one block.
    raw10 = b"".join(READING.pack(*([100 + i] * 6)) for i in range(10))   # 1.00..1.09 A
    co = zlib.compressobj(9, zlib.DEFLATED, -15)
    comp = co.compress(raw10) + co.flush()
    assert len(comp) < len(raw10)                                  # actually compressed
    bh_c = BLOCK_HEADER.pack(0x6688A0C0, 2_000, 0, len(comp), crc16_ccitt(comp))
    out = decode_bulk_upload(fh + bh_c + comp, {"X-Ntp-Valid": "0"})
    assert len(out.readings) == 10, f"expected 10 readings, got {len(out.readings)}"
    assert abs(out.readings[9].amps[0] - 1.09) < 1e-6
    assert out.readings[9].epoch == 0x6688A0C0 + 9                 # positional stamping
    print("ok (compressed):", out.http_response())

    # Self-test 5: non-default send interval — positional stamping must scale.
    out = decode_bulk_upload(fh + bh_c + comp, {"X-Ntp-Valid": "0", "X-Interval": "5"})
    assert out.readings[9].epoch == 0x6688A0C0 + 9 * 5
    print("ok (interval=5):", out.http_response())

    # Self-test 6: rejected-log NDJSON drain — parse, stamp, quarantine bad lines.
    nd = (b'{"device_id":"dev01","timestamp":"2026-07-06T21:14:52.000Z",'
          b'"readings":{"cts":{"ct_1":{"amps":1.23},"ct_2":{"amps":0.13},"ct_3":{"amps":0.13},'
          b'"ct_4":{"amps":0.13},"ct_5":{"amps":0.13},"ct_6":{"amps":0.13}},"voltage_rms":230.0}}\n'
          b'not json at all\n')
    out = decode_bulk_upload(nd, {"X-Format": "rejected-ndjson"})
    assert out.format == "rejected-ndjson" and len(out.readings) == 1
    assert abs(out.readings[0].amps[0] - 1.23) < 1e-9
    assert out.readings[0].epoch is not None and not out.readings[0].clock_unsynced
    assert out.quarantine_bytes == b"not json at all"
    print("ok (rejected-ndjson):", out.http_response())
