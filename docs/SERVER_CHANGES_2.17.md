# Server changes required for firmware 2.17.0 — APPLY AT PUBLISH TIME

**Do not apply before the 2.17.0 rollout decision.** The backend working tree
auto-deploys on the nightly logrotate SIGHUP, so these edits must not sit in
`/home/pranav/linesights.com/backend` ahead of time. Ship order is otherwise
free: 2.17.0 firmware is fully safe against an old server (batch 404s → device
latches back to single sends and re-probes hourly; unknown heartbeat/telemetry
fields are ignored; commands without ids behave exactly as 2.16.2).

## 1. Migration (run first, `sudo -u postgres psql linesights_db`)

```sql
ALTER TABLE device_telemetry
  ADD COLUMN IF NOT EXISTS post_ms_p50 integer,
  ADD COLUMN IF NOT EXISTS post_ms_p90 integer,
  ADD COLUMN IF NOT EXISTS bulk_bps bigint,
  ADD COLUMN IF NOT EXISTS batch_size integer,
  ADD COLUMN IF NOT EXISTS net_wdt_min integer,
  ADD COLUMN IF NOT EXISTS wedge_reboots integer;
ALTER TABLE device_commands
  ADD COLUMN IF NOT EXISTS acknowledged_at timestamp;
```

## 2. `models/firmware.py`

Add the six new columns to the telemetry model AND to `TELEMETRY_FIELDS`
(same pattern as the ota_* additions of 08-03): `post_ms_p50`, `post_ms_p90`,
`bulk_bps`, `batch_size`, `net_wdt_min`, `wedge_reboots`. Add
`acknowledged_at` to the DeviceCommand model + its `to_dict()`.

## 3. `firmware_api.py` — heartbeat handler

a) Command entries gain their id (enables the firmware ack loop):

```python
entry = {'action': cmd.action, 'id': cmd.id}
```

b) Before the command fetch, process acks from the heartbeat body. This is
the 499-hole closure: `executed_at` still means "sent"; `acknowledged_at`
means the device CONFIRMED execution. (Auto-redelivery of unacked commands is
a later, separate decision — visibility alone ends the uptime-forensics era.)

```python
        acks = data.get('cmd_acks') or []
        if acks:
            try:
                ack_ids = [int(a) for a in acks][:32]
                DeviceCommand.query.filter(
                    DeviceCommand.device_id == device_id,
                    DeviceCommand.id.in_(ack_ids),
                    DeviceCommand.acknowledged_at.is_(None),
                ).update({'acknowledged_at': datetime.utcnow()},
                         synchronize_session=False)
                db.session.commit()
            except (TypeError, ValueError):
                pass
```

## 4. `routes/power_data.py` — batch ingest endpoint

Contract (firmware `_postOneBatch` depends on every clause):
- Body: `application/x-ndjson`, one live-reading JSON per line (identical
  schema to `/api/data` bodies), header `X-Batch-Count`.
- **Never 4xx.** Bad lines → `_quarantine_upload` + continue; unknown device
  → quarantine + 200 (a 404 here means "endpoint missing" to the firmware and
  latches batching off; 400/422 makes it park good data to rejected.log).
- 200 response body: `{"stored": n, "quarantined": m}`.

```python
@power_data_bp.route('/api/data/batch', methods=['POST'])
@csrf_exempt
def receive_batch_data():
    """2.17.0 live batching: N single-reading JSON lines in one POST.
    Wire win is request COUNT on queue-delayed uplinks, not bytes.
    Reuses the single-reading parse/stage/insert helpers line by line."""
    db = get_db()
    models = get_models()
    PiDevice = models['PiDevice']
    raw = request.get_data(cache=False) or b''
    stored = quarantined = 0
    to_insert = []
    device_cache = {}
    for line in raw.split(b'\n'):
        line = line.strip()
        if not line:
            continue
        try:
            data = json.loads(line)
            if not isinstance(data, dict):
                raise ValueError('not an object')
        except (ValueError, UnicodeDecodeError):
            repaired = _repair_device_json(line)
            if repaired is None:
                _quarantine_upload('batch-line', 'unparseable', line[:600],
                                   request.content_type)
                quarantined += 1
                continue
            data = repaired
        device_id = DEVICE_ID_ALIASES.get(data.get('device_id'),
                                          data.get('device_id'))
        if not device_id:
            _quarantine_upload('batch-line', 'no device_id', line[:600])
            quarantined += 1
            continue
        if device_id not in device_cache:
            device_cache[device_id] = PiDevice.query.get(device_id)
        if device_cache[device_id] is None:
            _quarantine_upload(device_id, 'unregistered (batch)', line[:600])
            quarantined += 1
            continue
        raw_ts = data.get('timestamp')
        try:
            ts = datetime.fromisoformat(raw_ts.replace('Z', '+00:00'))
        except (AttributeError, TypeError, ValueError):
            ts = datetime.now(tz=timezone.utc)
        if data.get('clock_unsynced'):
            _stage_readings(device_id, [{'payload': data}], 'batch-unsynced')
            continue
        kw = _reading_kwargs_from_payload(device_id, ts, data)
        if kw:
            to_insert.append(kw)
    if to_insert:
        _bulk_insert_readings_idempotent(db, to_insert)
        stored = len(to_insert)
        db.session.commit()
        # keep the per-device liveness/alert side effects of the single path
        for did in device_cache:
            if device_cache[did] is not None:
                device_cache[did].last_seen = datetime.utcnow()
        db.session.commit()
    return jsonify({'stored': stored, 'quarantined': quarantined}), 200
```

NOTE for the implementer: check `_stage_readings`' exact row shape and
`_reading_kwargs_from_payload`'s signature against current code before
pasting — both were written against the 08-05 versions. Also verify the
single path's alert-evaluation hook (`_evaluate_alerts_for_device`) — if the
fleet moves to batch>1 permanently, that hook should be invoked per batch
here too (it currently rides only the single endpoint).

## 5. Deploy sequence

1. Migration SQL (idempotent).
2. Apply code edits; `sudo kill -HUP <gunicorn master>`.
3. Verify: `curl -s -X POST localhost/api/data/batch -H 'Content-Type: application/x-ndjson' --data-binary $'{}\n'` → 200 `{"stored":0,"quarantined":1}`.
4. Register + activate 2.17.0 only after this is live (or rely on the
   firmware's 404 latch — it works, but why exercise it fleet-wide).
5. Flip `set_batch 5-10` per device ONLY on trickle-uplink sites (Meton
   first); healthy sites can stay at 1 indefinitely.
6. Super Admin fleet page: surface `post_ms_p50/p90` + `bulk_bps` (Layer 2
   of the link-health panel) and `acknowledged_at` on the command log.
