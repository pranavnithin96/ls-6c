#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "config.h"
#include "wifi_manager.h"
#include "diagnostics.h"
#include "http_sender.h"
#include "ct_sensor.h"
#include "waveform_blackbox.h"

// ============================================================================
// Heartbeat — Thread-safe error log with FIXED-SIZE char arrays (no heap
// allocation inside spinlocks), rate-limited commands, bounded storage
// ============================================================================

#define ERROR_LOG_FILE "/errors.json"

void setHTTPDebug(bool on);
void forceOTACheck();

// Fixed-size error entry — NO String objects, safe inside portENTER_CRITICAL
struct ErrorEntry {
    char timestamp[32];
    char message[128];
};

static portMUX_TYPE _errMux = portMUX_INITIALIZER_UNLOCKED;
static ErrorEntry _errorLog[MAX_ERROR_LOG];
static int _errorHead = 0;
static int _errorCount = 0;
static unsigned long _lastHeartbeat = 0;
static unsigned long _lastErrorSave = 0;
static String _heartbeatUrl;
static String _hbBootReason;
static bool _errorsDirty = false;
static unsigned long _lastCommandTime = 0;

int getRSSIMin();
int getRSSIAvg();

void loadErrorsFromFlash() {
    if (!LittleFS.exists(ERROR_LOG_FILE)) return;

    File f = LittleFS.open(ERROR_LOG_FILE, "r");
    if (!f) return;
    String content = f.readString();
    f.close();

    JsonDocument doc;
    if (deserializeJson(doc, content)) {
        LittleFS.remove(ERROR_LOG_FILE);
        return;
    }

    JsonArray arr = doc.as<JsonArray>();
    for (JsonVariant v : arr) {
        if (_errorCount >= MAX_ERROR_LOG) break;
        strlcpy(_errorLog[_errorHead].timestamp, v["t"] | "", sizeof(_errorLog[0].timestamp));
        strlcpy(_errorLog[_errorHead].message, v["m"] | "", sizeof(_errorLog[0].message));
        _errorHead = (_errorHead + 1) % MAX_ERROR_LOG;
        _errorCount++;
    }
    Serial.printf("[ERR] Loaded %d errors from flash\n", _errorCount);
}

void saveErrorsToFlash() {
    if (!_errorsDirty || _errorCount == 0) return;

    // Snapshot under spinlock — only char array copies (no alloc, no I/O)
    char snapTs[MAX_ERROR_LOG][32];
    char snapMsg[MAX_ERROR_LOG][128];
    int snapCount = 0;

    portENTER_CRITICAL(&_errMux);
    int start = (_errorCount >= MAX_ERROR_LOG) ? _errorHead : 0;
    snapCount = min(_errorCount, (int)MAX_ERROR_LOG);
    for (int i = 0; i < snapCount; i++) {
        int idx = (start + i) % MAX_ERROR_LOG;
        memcpy(snapTs[i], _errorLog[idx].timestamp, 32);
        memcpy(snapMsg[i], _errorLog[idx].message, 128);
    }
    portEXIT_CRITICAL(&_errMux);

    // Build JSON OUTSIDE spinlock
    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < snapCount; i++) {
        JsonObject e = arr.add<JsonObject>();
        e["t"] = snapTs[i];
        e["m"] = snapMsg[i];
    }

    // Write file OUTSIDE spinlock
    File f = LittleFS.open("/errors.tmp", "w");
    if (f) {
        serializeJson(doc, f);
        f.close();
        if (LittleFS.exists(ERROR_LOG_FILE)) LittleFS.remove(ERROR_LOG_FILE);
        LittleFS.rename("/errors.tmp", ERROR_LOG_FILE);
        _errorsDirty = false;
    }
}

// Thread-safe error logging — only fixed-size memcpy inside spinlock
void logError(const String& message) {
    // Get timestamp OUTSIDE critical section
    char ts[32];
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
        strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);
    } else {
        strlcpy(ts, "1970-01-01T00:00:00.000Z", sizeof(ts));
    }

    // Only fixed-size copies inside spinlock — NO heap allocation
    portENTER_CRITICAL(&_errMux);
    strlcpy(_errorLog[_errorHead].timestamp, ts, sizeof(_errorLog[0].timestamp));
    strlcpy(_errorLog[_errorHead].message, message.c_str(), sizeof(_errorLog[0].message));
    _errorHead = (_errorHead + 1) % MAX_ERROR_LOG;
    if (_errorCount < MAX_ERROR_LOG) _errorCount++;
    _errorsDirty = true;
    portEXIT_CRITICAL(&_errMux);

    Serial.printf("[ERR] %s\n", message.c_str());
}

void initHeartbeat(const String& serverUrl) {
    _heartbeatUrl = "http://46.224.90.187/api/firmware/heartbeat";
    // Ask for persistent desired CT configuration immediately after boot. A
    // newly upgraded legacy device has no local mask, so waiting 60 seconds
    // would produce a minute of intentionally disabled readings.
    _lastHeartbeat = millis() - HEARTBEAT_INTERVAL_MS;

    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON:   _hbBootReason = "power_on"; break;
        case ESP_RST_SW:        _hbBootReason = "software"; break;
        case ESP_RST_PANIC:     _hbBootReason = "panic"; break;
        case ESP_RST_INT_WDT:   _hbBootReason = "int_watchdog"; break;
        case ESP_RST_TASK_WDT:  _hbBootReason = "task_watchdog"; break;
        case ESP_RST_WDT:       _hbBootReason = "watchdog"; break;
        case ESP_RST_BROWNOUT:  _hbBootReason = "brownout"; break;
        default:                _hbBootReason = "unknown"; break;
    }

    loadErrorsFromFlash();
    Serial.printf("[HB] -> %s\n", _heartbeatUrl.c_str());

    if (reason != ESP_RST_POWERON && reason != ESP_RST_SW) {
        logError("Unexpected boot: " + _hbBootReason);
    }
}

// --- Command acks (2.17.0) --------------------------------------------------
// The 499 hole, closed from the device side: the server may include an "id"
// with each command; every executed id is echoed back as cmd_acks in each
// heartbeat until a heartbeat gets its 200 (ack delivery confirmed). A server
// that sends no ids gets exactly the old behavior. Re-delivered ids (our ack
// got lost) are re-acked but NOT re-executed — _doneRing remembers the last
// 32 — so an ack-mode server can safely re-deliver without double-executing.
// Reboot-class commands persist their id to NVS BEFORE restarting, so the ack
// survives the reboot and a re-delivered reboot can never loop the device.
static uint32_t _ackPending[16]; static uint8_t _ackCount = 0;
static uint32_t _doneRing[32];   static uint8_t _doneIdx = 0; static bool _doneInit = false;
static uint8_t _ackedInFlight = 0;   // how many acks the in-flight heartbeat carried

static void _doneRemember(uint32_t id) {
    _doneRing[_doneIdx] = id;
    _doneIdx = (uint8_t)((_doneIdx + 1) % 32);
}
static bool _doneSeen(uint32_t id) {
    for (int i = 0; i < 32; i++) if (_doneRing[i] == id) return true;
    return false;
}
static void _ackAdd(uint32_t id) {
    if (id == 0) return;
    for (int i = 0; i < _ackCount; i++) if (_ackPending[i] == id) return;
    if (_ackCount < 16) _ackPending[_ackCount++] = id;
}
static void _ackLoadRebootId() {          // called once, lazily, before first use
    if (_doneInit) return;
    _doneInit = true;
    Preferences p;
    if (p.begin("lscfg", true)) {
        uint32_t id = p.getUInt("rebootcmdid", 0);
        p.end();
        if (id != 0) { _doneRemember(id); _ackAdd(id); }
    }
}
static void _ackPersistRebootId(uint32_t id) {
    if (id == 0) return;
    Preferences p;
    if (p.begin("lscfg", false)) { p.putUInt("rebootcmdid", id); p.end(); }
}

void processCommands(const String& responseBody) {
    if (responseBody.length() == 0) return;

    JsonDocument doc;
    if (deserializeJson(doc, responseBody)) return;

    JsonArray commands = doc["commands"];
    if (commands.isNull() || commands.size() == 0) return;

    unsigned long now = millis();
    if (now - _lastCommandTime < 30000) {
        Serial.println("[CMD] Rate limited");
        return;
    }
    _lastCommandTime = now;

    int cmdCount = min((int)commands.size(), 10);
    int processed = 0;
    for (JsonObject cmd : commands) {
        if (processed >= cmdCount) break;
        processed++;

        String action = cmd["action"] | "";
        if (action.length() == 0) continue;
        _ackLoadRebootId();
        uint32_t cmdId = cmd["id"] | 0U;
        if (cmdId != 0 && _doneSeen(cmdId)) {
            _ackAdd(cmdId);     // our ack was lost — re-ack, never re-execute
            Serial.printf("[CMD] duplicate id %u (%s) — re-acking only\n",
                          (unsigned)cmdId, action.c_str());
            continue;
        }
        if (cmdId != 0) { _doneRemember(cmdId); _ackAdd(cmdId); }

        Serial.printf("[CMD] -> %s\n", action.c_str());

        if (action == "set_interval") {
            int val = cmd["value"] | -1;
            setSendInterval(val);          // live + persisted (range-checked inside) — Defect 2
        } else if (action == "set_voltage") {
            float val = cmd["value"] | -1.0f;
            setGridVoltage(val);           // live + persisted (range-checked inside) — Defect 2
        } else if (action == "reboot") {
            _ackPersistRebootId(cmdId);   // ack survives the restart
            flushBeforeRestart();
            delay(500);
            ESP.restart();
        } else if (action == "debug_on") {
            setHTTPDebug(true);
        } else if (action == "debug_off") {
            setHTTPDebug(false);
        } else if (action == "update_firmware") {
            forceOTACheck();
        } else if (action == "wfstats_on") {
            setWaveformStats(true);          // enable waveform features on this unit
        } else if (action == "wfstats_off") {
            setWaveformStats(false);
        } else if (action == "recover_rejected") {
            recoverRejectedFiles();          // re-queue quarantined offline files
        } else if (action == "upload_rejected_log") {
            if (!isDrainEnabled())
                logError("upload_rejected_log ignored — drain paused (send resume_drain)");
            requestRejectedDrain();          // drained one file per tick by processSendQueue
        } else if (action == "pause_drain") {
            setDrainEnabled(false);          // keep backlog on flash, stop spending uplink on it
        } else if (action == "resume_drain") {
            setDrainEnabled(true);
        } else if (action == "set_ct_slope") {
            // Remote per-sensor calibration: {"action":"set_ct_slope","channel":1-6,"value":<A/count>}
            // value 0 clears back to the rating default. Live + persisted (survives OTA).
            int ch = cmd["channel"] | -1;
            float val = cmd["value"] | -1.0f;
            if (!setChannelSlope(ch - 1, val))
                logError("set_ct_slope rejected: bad channel/slope");
        } else if (action == "set_ct_channels") {
            // Atomic six-bit configuration. `mask` is preferred; `value` keeps
            // compatibility with the server's scalar command schema. A higher
            // revision is staged now and applied by Core 1 at the next complete
            // one-second frame boundary.
            int mask = cmd.containsKey("mask") ? (int)cmd["mask"] : (int)(cmd["value"] | -1);
            uint32_t revision = cmd["revision"] | 0U;
            if (mask < 0 || mask > 0x3f || !stageCTChannelConfig((uint8_t)mask, revision))
                logError("set_ct_channels rejected: want mask 0-63 and newer revision");
        } else if (action == "set_window_ms") {
            // v2.18 fixes acquisition at one complete second. Retained only as
            // an idempotent compatibility command for value=1000.
            int val = cmd["value"] | -1;
            if (!setSampleWindowMs(val))
                logError("set_window_ms rejected: v2.18 requires 1000");
        } else if (action == "capture_waveform") {
            // Freeze the raw-sample ring and upload it (2.14.0). The capture
            // itself runs on the sampling task between windows; this only
            // raises the request flag. Rate-limited to one per 10 min.
            wbRequestCapture();
        } else if (action == "wfstream_on") {
            // Continuous WFS2 export is authenticated independently of the
            // public device id. The bearer credential is generated server-side
            // per enable, kept in NVS, and never printed in diagnostics.
            String token = cmd["value"] | "";
            wbSetStream(true, token);
        } else if (action == "wfstream_off") {
            wbSetStream(false);
        } else if (action == "clear_rejected") {
            clearRejectedStore();            // DESTRUCTIVE: discards parked backlog
        } else if (action == "set_reboot_days") {
            int val = cmd["value"] | -1;     // 0 disables; max 45 (millis rollover at 49.7d)
            if (val >= 0 && val <= 45) {
                Preferences p; p.begin("lscfg", false);
                p.putInt("rebootdays", val); p.end();
                initScheduledReboot();       // recompute immediately, no reboot needed
            }
        } else if (action == "set_batch") {
            // Live-batching (2.17.0): N readings per POST. 1 = classic single
            // sends; raise on trickle uplinks (request count is what kills them).
            int val = cmd["value"] | -1;
            if (!setBatchSize(val))
                logError("set_batch rejected: want 1-" + String(BATCH_MAX));
        } else if (action == "set_net_watchdog") {
            // Connectivity watchdog window in minutes, 0 = off (2.17.0).
            int val = cmd["value"] | -1;
            if (!setNetWatchdogMin(val))
                logError("set_net_watchdog rejected: want 0-1440 min");
        } else if (action == "factory_reset") {
            flushBeforeRestart();
            Preferences p; p.begin("lscfg", false);
            p.clear(); p.end();
            // Review MED-6: persist AFTER the clear (clear() wipes lscfg, and
            // under an ack-mode re-delivering server a lost factory_reset ack
            // is a re-provision->wipe brick loop).
            _ackPersistRebootId(cmdId);
            delay(500);
            ESP.restart();
        }
    }
}

void sendHeartbeat() {
    if (_heartbeatUrl.length() == 0) return;
    _ackLoadRebootId();   // review MED-6: a persisted reboot ack must ride the
                          // FIRST heartbeat after boot, not wait for a response
                          // that happens to contain another command.

    JsonDocument doc;
    doc["device_id"] = getDeviceId();
    doc["firmware_version"] = FIRMWARE_VERSION;
    doc["wifi_rssi"] = WiFi.RSSI();
    doc["wifi_rssi_min"] = getRSSIMin();
    doc["wifi_rssi_avg"] = getRSSIAvg();
    doc["wifi_ssid"] = WiFi.SSID();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["min_heap"] = ESP.getMinFreeHeap();
    doc["uptime_seconds"] = getUptimeSeconds();
    doc["ip_address"] = WiFi.localIP().toString();
    doc["boot_reason"] = _hbBootReason;
    doc["queue_depth"] = getQueueSize();
    doc["total_sent"] = getTotalSent();
    doc["total_failed"] = getTotalFailed();
    doc["total_dropped"] = getTotalDropped();
    doc["ct_calibrated"] = isCTCalibrated();
    doc["crash_count"] = getCrashCount();
    doc["wfstats"] = waveformStatsEnabled();          // waveform feature state
    struct tm _tsync; doc["clock_synced"] = getLocalTime(&_tsync, 0);
    doc["rejected_files"] = getRejectedFileCount();   // quarantined offline files awaiting recovery
    // --- Diagnosis fields (v2.7): turn the fleet page into a root-cause board ---
    doc["device_epoch"] = (uint32_t)time(nullptr);    // server: offset = received_at - this = clock skew
    doc["offline_stored"] = getOfflineStored();       // readings in the CURRENT chunk only
    doc["offline_bytes"] = getOfflineFileSize();      // current chunk size
    doc["offline_backlog_bytes"] = getBacklogBytes(); // rotated chunks + quarantined files awaiting upload
    doc["rejected_log_bytes"] = getRejectedLogBytes();// 400'd live readings parked on flash (cap 50KB)
    doc["wifi_reconnects"] = getWiFiReconnectCount(); // link flakiness since boot
    doc["last_http_code"] = getLastHttpCode();        // "unreachable" vs "server rejecting", cleanly
    doc["last_success_age_s"] = getLastSuccessAgeS(); // seconds since a data POST succeeded (-1 never)
    float _tc = temperatureRead();                    // thermal stress (coarse internal sensor)
    if (isnan(_tc) || isinf(_tc)) _tc = 0.0f;         // raw "nan" would break the whole JSON body
    doc["cpu_temp_c"] = serialized(String(_tc, 1));
    doc["heap_largest_block"] = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);  // fragmentation
    doc["mac"] = WiFi.macAddress();                   // stable hardware identity
    doc["build"] = __DATE__ " " __TIME__;             // exact build, beyond version string
    doc["ntp_sync_age_s"] = getNtpSyncAgeS();         // seconds since last SNTP sync (-1 never)

    // --- Per-channel CT calibration state (2.13.0) ---
    // The rating lives only in device NVS and was never reported, so the server
    // could not tell which channels use which slope. That made a calibration
    // change unverifiable and its blast radius unknowable: when the 50A default
    // moved 0.0090 -> 0.0421, nothing server-side could say which channels would
    // shift 4.7x. ct_slopes is the EFFECTIVE A/count actually applied (measured
    // per-channel slope if calibrated, else the rating default), so the server
    // can confirm an OTA landed and recompute thresholds for exactly the
    // channels that moved. ~90 bytes on a 5-minute heartbeat.
    doc["window_ms"] = getSampleWindowMs();           // sampling window (2.14.0)
    doc["ct_active_mask"] = getActiveCTMask();        // applied physical-input mask
    doc["ct_config_revision"] = getCTConfigRevision();// applied remote/provisioning revision
    doc["ct_config_required"] = isCTConfigRequired(); // fresh device has no active inputs
    doc["ct_config_pending"] = isCTConfigPending();   // staged; applies next frame boundary
    doc["wb_pending"] = wbDumpPending();              // black-box dump awaiting upload
    doc["wfstream"] = wbStreamOn();                   // complete-frame WFS2 export
    doc["wfstream_seq"] = wbStreamSeq();              // delivered frames this uptime
    doc["wfs2_generated"] = wbFramesGenerated();
    doc["wfs2_delivered"] = wbFramesDelivered();
    doc["wfs2_dropped"] = wbFramesDropped();
    doc["wfs2_queued"] = wbQueuedFrames();
    doc["sampling_overruns_last"] = getSamplingOverrunsLast();
    doc["sampling_overruns_total"] = getSamplingOverrunsTotal();
    doc["ota_status"] = getOTAStatus();
    doc["ota_progress"] = getOTAProgress();
    doc["ota_total"] = getOTATotal();
    doc["ota_target"] = getOTATarget();
    // --- 2.17.0: the device's own internet-speed report + new feature state ---
    doc["post_ms_p50"] = getPostMsP50();              // live POST round-trip, median
    doc["post_ms_p90"] = getPostMsP90();              // ...and tail (queue-delay signal)
    doc["bulk_bps"] = getBulkBps();                   // last bulk upload throughput
    doc["batch_size"] = getBatchSize();               // live batching factor (1 = off)
    doc["net_wdt_min"] = getNetWatchdogMin();         // connectivity watchdog window
    doc["drain_enabled"] = isDrainEnabled();          // pause_drain state — 2.17.4:
                                                      // invisible pause cost a day of
                                                      // meton_07 misdiagnosis
    doc["wedge_reboots"] = getWedgeReboots();         // times the net watchdog fired
    // Executed-command ids not yet confirmed delivered (2.17.0 ack loop). An
    // ack-mode server marks them executed on receipt; cleared on our 200.
    if (_ackCount > 0) {
        JsonArray _acks = doc["cmd_acks"].to<JsonArray>();
        for (int i = 0; i < _ackCount; i++) _acks.add(_ackPending[i]);
    }
    _ackedInFlight = _ackCount;

    JsonArray _ctRat = doc["ct_ratings"].to<JsonArray>();
    JsonArray _ctSlp = doc["ct_slopes"].to<JsonArray>();
    JsonArray _ctCal = doc["ct_slope_calibrated"].to<JsonArray>();
    for (int _c = 0; _c < NUM_CT_CHANNELS; _c++) {
        int _r = getCtRating(_c);
        _ctRat.add(_r);
        _ctSlp.add(serialized(String(ctSlope(_c, _r), 5)));
        _ctCal.add(getChannelSlope(_c) > 0.0f);       // false = running on the rating default
    }

    // Snapshot errors under spinlock — only fixed-size char copies
    char snapTs[MAX_ERROR_LOG][32];
    char snapMsg[MAX_ERROR_LOG][128];
    int errCount = 0;

    portENTER_CRITICAL(&_errMux);
    if (_errorCount > 0) {
        int start = (_errorCount >= MAX_ERROR_LOG) ? _errorHead : 0;
        errCount = min(_errorCount, (int)MAX_ERROR_LOG);
        for (int i = 0; i < errCount; i++) {
            int idx = (start + i) % MAX_ERROR_LOG;
            memcpy(snapTs[i], _errorLog[idx].timestamp, 32);
            memcpy(snapMsg[i], _errorLog[idx].message, 128);
        }
    }
    portEXIT_CRITICAL(&_errMux);

    // Build error JSON OUTSIDE spinlock
    JsonArray errors = doc["errors"].to<JsonArray>();
    for (int i = 0; i < errCount; i++) {
        JsonObject e = errors.add<JsonObject>();
        e["timestamp"] = snapTs[i];
        e["message"] = snapMsg[i];
    }

    String json;
    serializeJson(doc, json);

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(_heartbeatUrl);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(HEARTBEAT_TIMEOUT_MS);
    int httpCode = http.POST(json);

    if (httpCode == 200) {
        recordHeartbeatOk();               // counts as server contact (net watchdog)
        // The 200 confirms the server received this heartbeat's cmd_acks —
        // retire exactly the ones that were in flight; anything queued by
        // processCommands() below rides the NEXT heartbeat. Also safe to
        // clear the persisted reboot-command id: its ack has now landed.
        if (_ackedInFlight > 0) {
            uint8_t remaining = (uint8_t)(_ackCount - _ackedInFlight);
            for (uint8_t i = 0; i < remaining; i++)
                _ackPending[i] = _ackPending[_ackedInFlight + i];
            _ackCount = remaining;
            _ackedInFlight = 0;
            Preferences p;
            if (p.begin("lscfg", false)) {
                // Most command acknowledgements have no persisted reboot id.
                // Avoid an unnecessary NVS mutation while WFS2 is sampling.
                if (p.isKey("rebootcmdid")) p.remove("rebootcmdid");
                p.end();
            }
        }
        String response = http.getString();
        Serial.printf("[HB] OK | heap:%u | Q:%d | S:%u\n",
            ESP.getFreeHeap(), getQueueSize(), getTotalSent());
        processCommands(response);

        // Clear error log after successful send — prevents stale errors repeating
        if (errCount > 0) {
            portENTER_CRITICAL(&_errMux);
            _errorHead = 0;
            _errorCount = 0;
            _errorsDirty = true;
            portEXIT_CRITICAL(&_errMux);
            saveErrorsToFlash();  // Persist the cleared state
        }
    } else if (httpCode > 0) {
        Serial.printf("[HB] HTTP %d\n", httpCode);
    } else {
        char errBuf[64];
        snprintf(errBuf, sizeof(errBuf), "HB fail: %s", http.errorToString(httpCode).c_str());
        logError(errBuf);
    }

    http.end();
}

void heartbeatLoop() {
    unsigned long now = millis();
    if (now - _lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
        _lastHeartbeat = now;
        sendHeartbeat();
        // Black-box dump upload rides the heartbeat cadence (networkTask):
        // file read + one HTTP POST, no ring access, ~24KB. Retries each
        // minute until a 2xx clears it.
        if (wbDumpPending())
            wbUploadDump("http://46.224.90.187", getDeviceId());
    }

    if (now - _lastErrorSave >= ERROR_SAVE_INTERVAL_MS) {
        _lastErrorSave = now;
        saveErrorsToFlash();
    }
}

int getErrorCount() { return _errorCount; }
