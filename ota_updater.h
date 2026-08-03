#pragma once
#include <Arduino.h>
#include <HTTPClient.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_ota_ops.h>
#include "config.h"
#include "led_status.h"

// ============================================================================
// OTA Updater 2.16 (v3, post-review) — resumable, health-gated, force-capable,
// self-narrating. Both modes run as SLICES from otaLoop, returning to
// networkTask between slices so heartbeats, ring flushes, drains and
// diagnostics keep running — nothing in this file ever blocks the task for
// more than one slice budget (~25s) plus one HTTP open (~30s), each preceded
// by a watchdog feed.
//
//   AUTO  — health-gated (recent live 200, quiet ring, heap floor); backoff
//           1m/10m/1h/6h persisted per version and RESTORED at boot; 30-min
//           journey TTL checked BEFORE the gates so a parked journey can
//           never become immortal; first check ~5 min after boot (hourly
//           after) so short-uptime machine-powered devices still update.
//   FORCE — update_firmware command. Skips the gates, restarts its 15-min
//           cap window, and suspends the live pipeline: processSendQueue
//           early-returns (flushing the ring to flash), and Core 1 diverts
//           fresh readings into the COMPRESSED OFFLINE STORE (550KB budget —
//           the real outage machinery), not the small rejected log. Between
//           slices heartbeatLoop narrates progress naturally.
//
// Journey integrity:
//   - One Update session per journey; resume via Range + If-Range(ETag).
//   - Resume REQUIRES a captured ETag; without one the journey restarts
//     from byte 0 instead of risking a spliced binary.
//   - 200-on-resume (file changed / validator mismatch) restarts the
//     journey in place WITHOUT a backoff charge (environmental, not the
//     device's fault). Content-Range mismatch and 416 abort with charge.
//   - Permanent HTTP errors (4xx/5xx) abort the journey immediately — no
//     hammer loops; transport errors retry paced (OTA_OPEN_PACE_MS).
//   - setReuse(false): an abandoned mid-body socket is never reused (a
//     kept-alive socket with an unread body poisons the next request).
//   - Optional md5 (lowercased) -> Update.setMD5; URL host is PINNED to the
//     known server (the path is taken from the check response, the host is
//     not) so a poisoned/absolute-elsewhere URL cannot redirect the fetch.
// ============================================================================

static String _otaDeviceId;
static const char* OTA_HOST = "http://46.224.90.187";   // pinned download host
static unsigned long _otaBootMs = 0;
static unsigned long _lastCheckMs = 0;
static bool _firstCheckDone = false;
static volatile bool _otaInProgress = false;
static volatile bool _otaForceCheck = false;      // light nudge (check_update)
static volatile bool _otaForceRequested = false;  // update_firmware command
static volatile bool _otaForcedActive = false;    // read from Core 1 (divert)

// -- journey state (statics, RAM-only: a reboot voids everything) -----------
static bool     _jActive = false;
static bool     _jForced = false;
static bool     _jSession = false;
static String   _jVersion;
static String   _jPath;                 // server path, host is pinned
static String   _jMd5;
static String   _jEtag;
static uint32_t _jTotal = 0;
static uint32_t _jWritten = 0;
static uint16_t _jConnects = 0;
static unsigned long _jStartedMs = 0;   // journey epoch (TTL / force cap)
static unsigned long _jLastOpenMs = 0;  // open pacing
static const char* _otaStatus = "idle"; // idle|downloading|flashing|failed

// force-check retry bookkeeping (a transient check must not eat the command)
static uint8_t _forceTries = 0;
static unsigned long _lastForceTryMs = 0;

// backoff: wait duration + epoch, elapsed-form; tier count persisted in NVS
static const uint32_t OTA_BACKOFF_S[] = {60, 600, 3600, 21600};
static unsigned long _backoffSetMs = 0;
static uint32_t _backoffWaitMs = 0;

void forceOTACheck() { _otaForceCheck = true; }

void logError(const String& message);
void disconnectHTTP();
void feedWatchdog();
void flushBeforeRestart();
void sendHeartbeat();
int  getQueueSize();
long getLastSuccessAgeS();

const char* otaStatusStr()   { return _otaStatus; }
uint32_t    otaProgress()    { return _jWritten; }
uint32_t    otaTotal()       { return _jTotal; }
bool        otaForcedActive(){ return _otaForcedActive; }
String      otaJourneyVersion() { return _jActive ? _jVersion : String(""); }

static bool isVersionGreater(const String& newVer, const String& oldVer) {
    int nMaj = 0, nMin = 0, nPat = 0;
    int oMaj = 0, oMin = 0, oPat = 0;
    sscanf(newVer.c_str(), "%d.%d.%d", &nMaj, &nMin, &nPat);
    sscanf(oldVer.c_str(), "%d.%d.%d", &oMaj, &oMin, &oPat);
    if (nMaj != oMaj) return nMaj > oMaj;
    if (nMin != oMin) return nMin > oMin;
    return nPat > oPat;
}

// Rollback check — called FIRST in setup().
// Review fix: esp_ota_get_last_invalid_partition() is a no-op without
// CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE (off in stock arduino-esp32), so the
// old code could print "no previous partition" and crash-loop forever. The
// other OTA slot still holds the previous app after an Update.h flash —
// fall back to booting it directly.
void checkFirmwareRollback() {
    Preferences otaPrefs;
    otaPrefs.begin("otastate", false);

    int crashCount = otaPrefs.getInt("crashes", 0);
    bool justUpdated = otaPrefs.getBool("updated", false);

    esp_reset_reason_t reason = esp_reset_reason();
    bool wasCrash = (reason != ESP_RST_POWERON && reason != ESP_RST_SW &&
                     reason != ESP_RST_DEEPSLEEP);

    if (wasCrash && justUpdated) {
        crashCount++;
        otaPrefs.putInt("crashes", crashCount);
        Serial.printf("[OTA] Post-update crash #%d/%d (reason: %d)\n",
                      crashCount, MAX_CRASH_COUNT, (int)reason);

        if (crashCount >= MAX_CRASH_COUNT) {
            Serial.println("[OTA] Too many crashes — rolling back!");
            otaPrefs.putBool("updated", false);
            otaPrefs.putInt("crashes", 0);
            otaPrefs.end();

            const esp_partition_t* prev = esp_ota_get_last_invalid_partition();
            if (prev == NULL) {
                // stock-core path: previous app lives in the other OTA slot
                const esp_partition_t* running = esp_ota_get_running_partition();
                const esp_partition_t* other = esp_ota_get_next_update_partition(NULL);
                if (other != NULL && other != running) prev = other;
            }
            if (prev != NULL) {
                esp_ota_set_boot_partition(prev);
                Serial.println("[OTA] Rollback set. Rebooting...");
                delay(1000);
                ESP.restart();
            } else {
                Serial.println("[OTA] No previous partition for rollback");
            }
            return;
        }
    } else if (!wasCrash && justUpdated) {
        Serial.println("[OTA] Firmware stable — marking valid");
        otaPrefs.putBool("updated", false);
        otaPrefs.putInt("crashes", 0);
        esp_ota_mark_app_valid_cancel_rollback();
    }

    otaPrefs.end();
}

// -- backoff -----------------------------------------------------------------

static uint32_t _backoffTierMs(int attempts) {
    int idx = attempts - 1;
    int maxIdx = (int)(sizeof(OTA_BACKOFF_S) / sizeof(OTA_BACKOFF_S[0])) - 1;
    if (idx < 0) idx = 0;
    if (idx > maxIdx) idx = maxIdx;
    return OTA_BACKOFF_S[idx] * 1000UL;
}

static bool _backoffActive() {
    return _backoffWaitMs != 0 && (millis() - _backoffSetMs) < _backoffWaitMs;
}

static void _backoffRecordFailure(const String& version) {
    Preferences p; p.begin("otastate", false);
    String v = p.getString("bkver", "");
    int n = (v == version) ? p.getInt("bkcnt", 0) : 0;
    if (n < 100) n++;
    p.putString("bkver", version);
    p.putInt("bkcnt", n);
    p.end();
    _backoffSetMs = millis();
    _backoffWaitMs = _backoffTierMs(n);
    Serial.printf("[OTA] Backoff: attempt %d, next auto in %lus\n",
                  n, (unsigned long)(_backoffWaitMs / 1000));
}

static void _backoffClear() {
    Preferences p; p.begin("otastate", false);
    p.remove("bkver");
    p.remove("bkcnt");
    p.end();
    _backoffWaitMs = 0;
}

void initOTAUpdater(const String& deviceId, const String& serverBaseUrl) {
    _otaDeviceId = deviceId;
    _otaBootMs = millis();
    // Review fix: restore the persisted backoff tier at boot so a machine-
    // power reboot doesn't grant an instant retry (the tier count survived;
    // the wait now does too, measured from boot).
    Preferences p; p.begin("otastate", false);
    int n = p.getInt("bkcnt", 0);
    p.end();
    if (n > 0) {
        _backoffSetMs = millis();
        _backoffWaitMs = _backoffTierMs(n);
    }
    Serial.printf("[OTA] Base: %s/api/firmware/ | Version: %s | bk=%d\n",
                  OTA_HOST, FIRMWARE_VERSION, n);
}

String getCurrentVersion() { return FIRMWARE_VERSION; }
bool isUpdateInProgress() { return _otaInProgress; }

// update_firmware command entry. Ignored while a forced journey is already
// running (review: prevents a re-arm chaining a second 15-min dark window).
void otaForceUpdate() {
    if (_jActive && _jForced) return;
    _otaForceRequested = true;
}

// -- journey engine ----------------------------------------------------------

static HTTPClient _jHttp;

static void _journeyAbort(const char* why, bool countFailure) {
    Serial.printf("[OTA] Journey aborted: %s (%u/%u bytes)\n", why, _jWritten, _jTotal);
    logError(String("OTA abort: ") + why);
    if (_jSession) Update.abort();
    if (countFailure && _jVersion.length()) _backoffRecordFailure(_jVersion);
    _jActive = false;
    _jSession = false;
    _jWritten = 0;
    _jTotal = 0;
    _otaStatus = "failed";
    _otaInProgress = false;
    _otaForcedActive = false;
    setLEDState(LED_ERROR);
}

// Restart the download from byte 0 WITHOUT killing the journey or charging
// backoff — used when resume isn't safe (no ETag) or the file changed
// server-side (200 on a ranged request).
static void _journeyRestartInPlace(const char* why) {
    Serial.printf("[OTA] Restarting from 0: %s\n", why);
    if (_jSession) Update.abort();
    _jSession = false;
    _jWritten = 0;
    _jTotal = 0;
    _jEtag = "";
}

// Open (or re-open with Range) the download connection.
// Returns: 1 connected+stream ready, 0 transient/paced (journey alive),
//          -1 journey aborted.
static int _openDownload() {
    if (_jConnects > 0 && (millis() - _jLastOpenMs) < OTA_OPEN_PACE_MS)
        return 0;                                   // pace re-dials, both modes
    _jLastOpenMs = millis();

    // resume requires a validator; without one, restart clean (review fix)
    if (_jWritten > 0 && _jEtag.length() == 0)
        _journeyRestartInPlace("no ETag to validate resume");

    String url = String(OTA_HOST) + _jPath;
    _jHttp.setReuse(false);   // review fix: never reuse a socket whose body we abandoned
    _jHttp.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    if (!_jHttp.begin(url)) {
        _journeyAbort("bad url", true);
        return -1;
    }
    _jHttp.setTimeout(30000);
    const char* hdrs[] = {"ETag", "Content-Range"};
    _jHttp.collectHeaders(hdrs, 2);

    bool resuming = (_jWritten > 0);
    if (resuming) {
        char range[48];
        snprintf(range, sizeof(range), "bytes=%u-", _jWritten);
        _jHttp.addHeader("Range", range);
        _jHttp.addHeader("If-Range", _jEtag);
    }

    feedWatchdog();
    int code = _jHttp.GET();
    _jConnects++;

    if (!resuming) {
        if (code == 200) {
            int len = _jHttp.getSize();
            if (len <= 0 || len > OTA_MAX_SIZE) {
                _jHttp.end();
                _journeyAbort("bad size", true);
                return -1;
            }
            _jTotal = (uint32_t)len;
            _jEtag = _jHttp.header("ETag");
            if (!Update.begin(_jTotal)) {
                _jHttp.end();
                _journeyAbort(Update.errorString(), true);
                return -1;
            }
            _jSession = true;
            if (_jMd5.length() == 32) {
                _jMd5.toLowerCase();                 // review fix: core compares case-sensitively
                if (!Update.setMD5(_jMd5.c_str())) Serial.println("[OTA] setMD5 rejected");
            }
            Serial.printf("[OTA] Journey: %s, %u bytes, etag=%s md5=%s\n",
                          _jVersion.c_str(), _jTotal,
                          _jEtag.length() ? "yes" : "NO",
                          _jMd5.length() ? "yes" : "no");
            return 1;
        }
        _jHttp.end();
        if (code >= 400) {                           // review fix: permanent — no hammer
            _journeyAbort("permanent HTTP error on open", true);
            return -1;
        }
        return 0;                                    // transport blip — paced retry
    }

    // resume path
    if (code == 206) {
        uint32_t rs = 0, re = 0, rt = 0;
        String cr = _jHttp.header("Content-Range");
        if (sscanf(cr.c_str(), "bytes %u-%u/%u", &rs, &re, &rt) != 3 ||
            rs != _jWritten || rt != _jTotal) {
            _jHttp.end();
            _journeyAbort("bad Content-Range on resume", true);
            return -1;
        }
        Serial.printf("[OTA] Resumed at %u/%u (connect #%u)\n", _jWritten, _jTotal, _jConnects);
        return 1;
    }
    _jHttp.end();
    if (code == 200) {
        // If-Range mismatch: the file changed (or validator formats differ).
        // Environmental — restart clean, no backoff charge (review fix).
        _journeyRestartInPlace("file changed (200 on resume)");
        return 0;
    }
    if (code == 416 || code >= 400) {
        _journeyAbort("permanent HTTP error on resume", true);
        return -1;
    }
    return 0;
}

// Pump stream->flash until done, stall, budget, or (auto) ring pressure.
// Returns 1 done, 0 yielded (journey alive), -1 aborted.
static int _pump() {
    WiFiClient* stream = _jHttp.getStreamPtr();
    uint8_t buf[1024];
    unsigned long sliceStart = millis();
    unsigned long lastByteMs = millis();
    uint32_t sinceYield = 0;

    while (_jWritten < _jTotal) {
        feedWatchdog();
        unsigned long now = millis();

        if ((now - sliceStart) > OTA_AUTO_PASS_BUDGET_MS) { _jHttp.end(); return 0; }
        if (!_jForced && getQueueSize() >= MAX_BUFFER_SIZE - 3) {
            _jHttp.end();                            // review fix: yield before ring overflow
            return 0;
        }

        int avail = stream->available();
        if (avail <= 0) {
            if (!stream->connected()) { _jHttp.end(); return 0; }
            if ((now - lastByteMs) > OTA_STALL_MS) { _jHttp.end(); return 0; }
            delay(10);
            continue;
        }

        uint32_t remaining = _jTotal - _jWritten;    // review fix: clamp to remaining
        int toRead = min((uint32_t)min(avail, (int)sizeof(buf)), remaining);
        int n = stream->readBytes(buf, toRead);
        if (n > 0) {
            size_t w = Update.write(buf, n);
            if (w != (size_t)n) {
                _jHttp.end();
                _journeyAbort("flash write error", true);
                return -1;
            }
            _jWritten += n;
            lastByteMs = millis();
            sinceYield += n;
            if (sinceYield >= 65536) {               // review fix: let IDLE0 breathe
                sinceYield = 0;
                delay(1);
            }
        }
    }
    _jHttp.end();
    return 1;
}

// Finalize: verify FIRST, then persist + narrate + reboot. The narration
// heartbeat runs AFTER Update.end() (review fix): if its command processing
// reboots us, we boot the NEW image — completion, not waste.
static void _finalize() {
    _otaStatus = "flashing";
    Serial.printf("[OTA] Download complete (%u bytes, %u connects) — finalizing\n",
                  _jTotal, _jConnects);

    if (!Update.end()) {
        _journeyAbort(Update.errorString(), true);   // md5/image validation lands here
        return;
    }
    if (!Update.isFinished()) {
        _journeyAbort("finalize incomplete", true);
        return;
    }

    Preferences p; p.begin("otastate", false);
    p.putBool("updated", true);
    p.putInt("crashes", 0);
    p.putString("rep_f", FIRMWARE_VERSION);
    p.putString("rep_t", _jVersion);
    p.putUInt("rep_ms", (uint32_t)(millis() - _jStartedMs));
    p.putUShort("rep_at", _jConnects);
    p.end();
    _backoffClear();

    sendHeartbeat();   // best-effort narration; boot partition already set

    Serial.printf("[OTA] Success! Rebooting to %s...\n", _jVersion.c_str());
    flushBeforeRestart();
    delay(1000);
    ESP.restart();
}

static void _journeyStart(const String& version, const String& path,
                          const String& md5, bool forced) {
    _jActive = true;
    _jForced = forced;
    _jSession = false;
    _jVersion = version;
    _jPath = path;
    _jMd5 = md5;
    _jEtag = "";
    _jTotal = 0;
    _jWritten = 0;
    _jConnects = 0;
    _jStartedMs = millis();
    _jLastOpenMs = 0;
    _otaStatus = "downloading";
    _otaInProgress = true;
    _otaForcedActive = forced;
    setLEDState(LED_OTA_UPDATING);
}

static bool _autoGatesOpen() {
    long okAge = getLastSuccessAgeS();
    if (okAge < 0 || okAge > 30) return false;
    if (getQueueSize() > BG_UPLOAD_LOWATER) return false;
    if (ESP.getFreeHeap() < OTA_MIN_HEAP) return false;
    return true;
}

// One slice of the active journey. TTL/cap are checked BEFORE the gates
// (review fix: a parked journey must age out, never become immortal).
static void _runSlice() {
    if (_jForced) {
        if ((millis() - _jStartedMs) > OTA_FORCE_MAX_MS) {
            _journeyAbort("force cap reached", true);
            return;
        }
        if (ESP.getFreeHeap() < OTA_MIN_HEAP) return;   // soft skip, cap still runs
    } else {
        if ((millis() - _jStartedMs) > OTA_AUTO_JOURNEY_TTL_MS) {
            _journeyAbort("auto journey TTL", true);
            return;
        }
        if (!_autoGatesOpen()) return;                  // parked; TTL above bounds it
    }

    int o = _openDownload();
    if (o <= 0) return;                                 // paced/transient/aborted
    int r = _pump();
    if (r == 1) _finalize();                            // reboots on success
}

// Query the server; on a valid offer start a journey.
// Returns 1 started, 0 definitive no-update, -1 transient failure.
static int _checkAndMaybeStart(bool forced) {
    disconnectHTTP();
    char checkUrlBuf[256];
    snprintf(checkUrlBuf, sizeof(checkUrlBuf),
             "%s/api/firmware/check?device_id=%s&current_version=%s",
             OTA_HOST, _otaDeviceId.c_str(), FIRMWARE_VERSION);

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(String(checkUrlBuf));
    http.setTimeout(10000);
    feedWatchdog();
    int httpCode = http.GET();
    if (httpCode != 200) {
        Serial.printf("[OTA] Check failed: HTTP %d\n", httpCode);
        http.end();
        return -1;
    }
    String payload = http.getString();
    http.end();
    if (payload.length() == 0) return -1;

    JsonDocument doc;
    if (deserializeJson(doc, payload)) return -1;
    if (!(doc["update_available"] | false)) {
        Serial.println("[OTA] Up to date");
        return 0;
    }

    String newVersion = doc["version"] | "unknown";
    String url = doc["url"] | "";
    String md5 = doc["md5"] | "";

    if (!isVersionGreater(newVersion, FIRMWARE_VERSION)) {
        Serial.printf("[OTA] Version %s not greater than %s — skipping\n",
            newVersion.c_str(), FIRMWARE_VERSION);
        return 0;
    }

    // Pin the host: take only the path from the server's url (review fix —
    // the url travels over plain HTTP; we never let it choose the host).
    int slash = url.indexOf("/api/");
    if (slash < 0) {
        logError("OTA: check url has no /api/ path");
        return 0;
    }
    String path = url.substring(slash);

    // a different offered version invalidates any persisted backoff tier
    Preferences p; p.begin("otastate", false);
    if (p.getString("bkver", "") != newVersion) {
        p.remove("bkver"); p.remove("bkcnt");
        _backoffWaitMs = 0;
    }
    p.end();

    _journeyStart(newVersion, path, md5, forced);
    return 1;
}

// otaLoop — one small step per networkTask pass. Never blocks beyond one
// slice; all long work yields back to the task loop.
void otaLoop() {
    // FORCE command: act even if an auto journey is parked (convert it).
    if (_otaForceRequested) {
        if (_jActive) {
            _otaForceRequested = false;
            _jForced = true;
            _jStartedMs = millis();                 // fresh 15-min cap window
            _otaForcedActive = true;
            Serial.println("[OTA] FORCE: converting active journey");
        } else if ((millis() - _lastForceTryMs) >= OTA_FORCE_CHECK_RETRY_MS ||
                   _forceTries == 0) {
            _lastForceTryMs = millis();
            int r = _checkAndMaybeStart(true);
            if (r == 1) { _otaForceRequested = false; _forceTries = 0; }
            else if (r == 0) {
                _otaForceRequested = false; _forceTries = 0;
                Serial.println("[OTA] Force: nothing to update");
            } else if (++_forceTries >= 3) {        // review fix: retry, don't
                _otaForceRequested = false;         // swallow the command on
                _forceTries = 0;                    // one transient blip
                logError("OTA: force check failed 3x");
            }
        }
    }

    if (_jActive) {
        _runSlice();
        return;
    }

    // First check ~5 min after boot (review fix: short-uptime devices used
    // to wait a full hour and often never checked at all), then hourly.
    bool due = _otaForceCheck;
    if (!_firstCheckDone) {
        if ((millis() - _otaBootMs) >= OTA_FIRST_CHECK_MS) due = true;
    } else if ((millis() - _lastCheckMs) >= OTA_CHECK_INTERVAL_MS) {
        due = true;
    }
    if (!due) return;

    bool nudge = _otaForceCheck;
    _otaForceCheck = false;
    _firstCheckDone = true;
    _lastCheckMs = millis();

    if (!nudge && _backoffActive()) return;         // nudge bypasses backoff
    if (!_autoGatesOpen()) return;
    if (_checkAndMaybeStart(false) == 1) _runSlice();
}
