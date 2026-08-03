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
// OTA Updater 2.16 — resumable, health-gated, force-capable, self-narrating.
//
// Two modes share one "journey" engine (a journey = downloading one specific
// version file into one open Update session, resumable across connection
// drops via HTTP Range):
//
//   AUTO  — runs from the hourly check. Only starts/continues when the link
//           is provably healthy (recent live 200, quiet ring, heap floor):
//           a struggling device never self-harms with a megabyte pull. Each
//           otaLoop pass spends at most OTA_AUTO_PASS_BUDGET_MS downloading,
//           then yields, so live data keeps flowing between passes. Failed
//           journeys back off exponentially (1m/10m/1h/6h), persisted per
//           version in NVS so machine-powered reboots don't reset the clock.
//
//   FORCE — the deliberate quiet-window hammer (update_firmware command).
//           Suspends live sends/drains (processSendQueue early-returns; the
//           Core-1 enqueue path parks every reading to flash exactly as it
//           does in an outage — zero-loss by the existing machinery), gives
//           the entire uplink to the download, retries stalls immediately,
//           and gives up at OTA_FORCE_MAX_MS. A full heartbeat goes out
//           every OTA_HB_INTERVAL_MS from inside the loop so the device
//           narrates progress instead of going dark.
//
// Integrity, both modes:
//   - ETag captured on first connect; resumes send If-Range so a file that
//     changed server-side mid-journey (re-pin!) restarts cleanly instead of
//     flashing a spliced binary.
//   - A 200 (not 206) on a resume attempt means the server ignored/refused
//     the Range or the file changed: journey aborts, next attempt is fresh.
//   - Optional md5 from the check response feeds Update.setMD5(), so a
//     corrupt image is rejected at finalize, before it can boot.
//
// Lifecycle narration (read by heartbeat.h): idle / downloading / flashing /
// failed, plus progress bytes, and a persisted "updated" report the first
// post-reboot heartbeat carries (from->to, duration, attempts).
// ============================================================================

static String _otaDeviceId;
static String _otaBaseUrl;
static unsigned long _lastOTACheck = 0;
static volatile bool _otaInProgress = false;   // any journey active (flash-write window included)
static volatile bool _otaForceCheck = false;
static volatile bool _otaForceRequested = false;  // update_firmware command arrived
static bool _otaForcedActive = false;             // FORCE journey running right now

// -- journey state (one at a time; statics, RAM-only: a reboot voids it) ----
static bool     _jActive = false;
static bool     _jForced = false;
static bool     _jSession = false;      // Update.begin() done for this journey
static String   _jVersion;
static String   _jUrl;
static String   _jMd5;
static String   _jEtag;
static uint32_t _jTotal = 0;
static uint32_t _jWritten = 0;
static uint16_t _jConnects = 0;         // connection (re)opens this journey
                                        // (uint16: forced mode can re-dial
                                        // ~450x against a dead server)
static unsigned long _jStartedMs = 0;
static const char* _otaStatus = "idle"; // idle|downloading|flashing|failed

// backoff (persisted per version so reboots don't reset it)
static const uint32_t OTA_BACKOFF_S[] = {60, 600, 3600, 21600};
static unsigned long _otaNextAutoMs = 0;

void forceOTACheck() { _otaForceCheck = true; }

void logError(const String& message);
void disconnectHTTP();
void feedWatchdog();
void flushBeforeRestart();
void sendHeartbeat();
int  getQueueSize();
long getLastSuccessAgeS();

// heartbeat.h reads these
const char* otaStatusStr()   { return _otaStatus; }
uint32_t    otaProgress()    { return _jWritten; }
uint32_t    otaTotal()       { return _jTotal; }
bool        otaForcedActive(){ return _otaForcedActive; }
String      otaJourneyVersion() { return _jActive ? _jVersion : String(""); }

static String forceHTTP(const String& url) {
    String out = url;
    out.replace("https://", "http://");
    return out;
}

// Semantic version comparison: returns true if newVer > oldVer
static bool isVersionGreater(const String& newVer, const String& oldVer) {
    int nMaj = 0, nMin = 0, nPat = 0;
    int oMaj = 0, oMin = 0, oPat = 0;
    sscanf(newVer.c_str(), "%d.%d.%d", &nMaj, &nMin, &nPat);
    sscanf(oldVer.c_str(), "%d.%d.%d", &oMaj, &oMin, &oPat);
    if (nMaj != oMaj) return nMaj > oMaj;
    if (nMin != oMin) return nMin > oMin;
    return nPat > oPat;
}

// Rollback check — called FIRST in setup() (unchanged semantics)
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
        Serial.printf("[OTA] Post-update crash #%d/%d (reason: %d)\n", crashCount, MAX_CRASH_COUNT, (int)reason);

        if (crashCount >= MAX_CRASH_COUNT) {
            Serial.println("[OTA] Too many crashes — rolling back!");
            otaPrefs.putBool("updated", false);
            otaPrefs.putInt("crashes", 0);
            otaPrefs.end();

            const esp_partition_t* prev = esp_ota_get_last_invalid_partition();
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

void initOTAUpdater(const String& deviceId, const String& serverBaseUrl) {
    _otaDeviceId = deviceId;
    // Bypass Cloudflare — direct IP
    _otaBaseUrl = "http://46.224.90.187/api/firmware/";
    Serial.printf("[OTA] Base: %s | Version: %s\n", _otaBaseUrl.c_str(), FIRMWARE_VERSION);
}

String getCurrentVersion() { return FIRMWARE_VERSION; }
bool isUpdateInProgress() { return _otaInProgress; }

// update_firmware command entry (called from heartbeat command handler on
// Core 0; the journey itself runs from otaLoop on the same task)
void otaForceUpdate() { _otaForceRequested = true; }

// -- backoff bookkeeping -----------------------------------------------------

static int _backoffAttempts(const String& version) {
    Preferences p; p.begin("otastate", true);
    String v = p.getString("bkver", "");
    int n = (v == version) ? p.getInt("bkcnt", 0) : 0;
    p.end();
    return n;
}

static void _backoffRecordFailure(const String& version) {
    Preferences p; p.begin("otastate", false);
    String v = p.getString("bkver", "");
    int n = (v == version) ? p.getInt("bkcnt", 0) : 0;
    if (n < 100) n++;
    p.putString("bkver", version);
    p.putInt("bkcnt", n);
    p.end();
    int idx = min(n - 1, (int)(sizeof(OTA_BACKOFF_S) / sizeof(OTA_BACKOFF_S[0])) - 1);
    _otaNextAutoMs = millis() + OTA_BACKOFF_S[idx] * 1000UL;
    Serial.printf("[OTA] Backoff: attempt %d, next auto in %lus\n", n, OTA_BACKOFF_S[idx]);
}

static void _backoffClear() {
    Preferences p; p.begin("otastate", false);
    p.remove("bkver");
    p.remove("bkcnt");
    p.end();
    _otaNextAutoMs = 0;
}

// -- journey engine ----------------------------------------------------------

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

// Open (or re-open with Range) the download connection. Returns the client on
// success with the stream positioned at _jWritten; nullptr on failure.
// On failure the journey may have been aborted (check _jActive).
static HTTPClient _jHttp;
static bool _openDownload() {
    _jHttp.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    _jHttp.begin(_jUrl);
    _jHttp.setTimeout(30000);
    const char* hdrs[] = {"ETag", "Content-Range"};
    _jHttp.collectHeaders(hdrs, 2);

    if (_jWritten > 0) {
        char range[48];
        snprintf(range, sizeof(range), "bytes=%u-", _jWritten);
        _jHttp.addHeader("Range", range);
        if (_jEtag.length()) _jHttp.addHeader("If-Range", _jEtag);
    }

    int code = _jHttp.GET();
    _jConnects++;

    if (_jWritten == 0) {
        if (code != 200) {
            Serial.printf("[OTA] Download HTTP %d\n", code);
            _jHttp.end();
            return false;
        }
        int len = _jHttp.getSize();
        if (len <= 0 || len > OTA_MAX_SIZE) {
            _jHttp.end();
            _journeyAbort("bad size", true);
            return false;
        }
        _jTotal = (uint32_t)len;
        _jEtag = _jHttp.header("ETag");

        if (!Update.begin(_jTotal)) {
            _jHttp.end();
            _journeyAbort(Update.errorString(), true);
            return false;
        }
        _jSession = true;
        if (_jMd5.length() == 32) Update.setMD5(_jMd5.c_str());
        Serial.printf("[OTA] Journey start: %s, %u bytes, etag=%s md5=%s\n",
                      _jVersion.c_str(), _jTotal, _jEtag.c_str(),
                      _jMd5.length() ? "yes" : "no");
        return true;
    }

    // resume path
    if (code == 206) {
        // Content-Range: bytes W-E/T — the start MUST be exactly where we
        // stopped and the total must be unchanged, or we'd splice binaries.
        uint32_t rs = 0, re = 0, rt = 0;
        String cr = _jHttp.header("Content-Range");
        if (sscanf(cr.c_str(), "bytes %u-%u/%u", &rs, &re, &rt) != 3 ||
            rs != _jWritten || rt != _jTotal) {
            _jHttp.end();
            _journeyAbort("bad Content-Range on resume", true);
            return false;
        }
        Serial.printf("[OTA] Resumed at %u/%u (connect #%u)\n", _jWritten, _jTotal, _jConnects);
        return true;
    }
    if (code == 200) {
        // Server ignored the Range or If-Range said the file changed.
        // Either way our partial flash no longer matches this response.
        _jHttp.end();
        _journeyAbort("file changed mid-journey (200 on resume)", true);
        return false;
    }
    Serial.printf("[OTA] Resume HTTP %d\n", code);
    _jHttp.end();
    return false;   // transient — journey stays alive for another attempt
}

// Pump bytes stream->flash until: done, stall, or deadline. Returns:
//   1 done, 0 stalled/connection died (journey alive), -1 journey aborted
static int _pump(unsigned long deadlineMs) {
    WiFiClient* stream = _jHttp.getStreamPtr();
    uint8_t buf[1024];
    unsigned long lastByteMs = millis();
    unsigned long lastHbMs = millis();

    while (_jWritten < _jTotal) {
        feedWatchdog();
        unsigned long now = millis();

        if (now > deadlineMs) { _jHttp.end(); return 0; }
        if (_jForced && (now - lastHbMs) >= OTA_HB_INTERVAL_MS) {
            lastHbMs = now;
            sendHeartbeat();          // narrate progress; commands still land
        }

        int avail = stream->available();
        if (avail <= 0) {
            if (!stream->connected()) { _jHttp.end(); return 0; }
            if ((now - lastByteMs) > OTA_STALL_MS) { _jHttp.end(); return 0; }
            delay(10);
            continue;
        }

        int toRead = min(avail, (int)sizeof(buf));
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
        }
    }
    _jHttp.end();
    return 1;
}

// Finalize a completed journey: verify, mark, reboot.
static void _finalize() {
    _otaStatus = "flashing";
    Serial.printf("[OTA] Download complete (%u bytes, %u connects) — finalizing\n",
                  _jTotal, _jConnects);
    // best-effort "I am flashing" narration; never block the update on it
    sendHeartbeat();

    if (!Update.end()) {
        _journeyAbort(Update.errorString(), true);   // md5/validation failures land here
        return;
    }
    if (!Update.isFinished()) {
        _journeyAbort("finalize incomplete", true);
        return;
    }

    Preferences p; p.begin("otastate", false);
    p.putBool("updated", true);
    p.putInt("crashes", 0);
    // persisted "done" report — the first heartbeat on the new firmware sends it
    p.putString("rep_f", FIRMWARE_VERSION);
    p.putString("rep_t", _jVersion);
    p.putUInt("rep_ms", (uint32_t)(millis() - _jStartedMs));
    p.putUShort("rep_at", _jConnects);
    p.end();
    _backoffClear();

    Serial.printf("[OTA] Success! Rebooting to %s...\n", _jVersion.c_str());
    flushBeforeRestart();
    delay(1000);
    ESP.restart();
}

static void _journeyStart(const String& version, const String& url,
                          const String& md5, bool forced) {
    _jActive = true;
    _jForced = forced;
    _jSession = false;
    _jVersion = version;
    _jUrl = forceHTTP(url);
    _jMd5 = md5;
    _jEtag = "";
    _jTotal = 0;
    _jWritten = 0;
    _jConnects = 0;
    _jStartedMs = millis();
    _otaStatus = "downloading";
    _otaInProgress = true;
    _otaForcedActive = forced;
    setLEDState(LED_OTA_UPDATING);
}

// FORCE journey: owns the task until done or capped. Data-safe: Core 1 parks
// every reading to flash while processSendQueue is suspended.
static void _runForced() {
    unsigned long capMs = millis() + OTA_FORCE_MAX_MS;
    while (_jActive && millis() < capMs) {
        feedWatchdog();
        if (!_openDownload()) {
            if (!_jActive) return;         // aborted inside open
            delay(2000);                    // brief pause before re-dial
            continue;
        }
        int r = _pump(capMs);
        if (r < 0) return;                  // aborted
        if (r == 1) { _finalize(); return; }// reboots on success
        // r == 0: stalled — loop re-opens with Range immediately
    }
    if (_jActive) _journeyAbort("force cap reached", true);
}

// AUTO journey slice: at most OTA_AUTO_PASS_BUDGET_MS per otaLoop pass, and
// only while the link is healthy — between passes live data flows normally.
static bool _autoGatesOpen() {
    long okAge = getLastSuccessAgeS();
    if (okAge < 0 || okAge > 30) return false;
    if (getQueueSize() > BG_UPLOAD_LOWATER) return false;
    if (ESP.getFreeHeap() < OTA_MIN_HEAP) return false;
    return true;
}

static void _runAutoSlice() {
    if (!_autoGatesOpen()) return;          // journey parks, resumes when clean
    if ((millis() - _jStartedMs) > OTA_AUTO_JOURNEY_TTL_MS) {
        _journeyAbort("auto journey TTL", true);
        return;
    }
    if (!_openDownload()) return;           // transient or aborted; either way yield
    int r = _pump(millis() + OTA_AUTO_PASS_BUDGET_MS);
    if (r == 1) _finalize();                // reboots on success
}

// Query the server; on an offer, validate and start/refresh a journey.
static void _checkAndMaybeStart(bool forced) {
    disconnectHTTP();
    char checkUrlBuf[256];
    snprintf(checkUrlBuf, sizeof(checkUrlBuf), "%scheck?device_id=%s&current_version=%s",
        _otaBaseUrl.c_str(), _otaDeviceId.c_str(), FIRMWARE_VERSION);

    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.begin(String(checkUrlBuf));
    http.setTimeout(10000);
    int httpCode = http.GET();
    if (httpCode != 200) {
        Serial.printf("[OTA] Check failed: HTTP %d\n", httpCode);
        http.end();
        return;
    }
    String payload = http.getString();
    http.end();
    if (payload.length() == 0) return;

    JsonDocument doc;
    if (deserializeJson(doc, payload)) return;
    if (!(doc["update_available"] | false)) {
        Serial.println("[OTA] Up to date");
        return;
    }

    String newVersion = doc["version"] | "unknown";
    String url = doc["url"] | "";
    String md5 = doc["md5"] | "";           // optional; server may not send it yet

    if (!isVersionGreater(newVersion, FIRMWARE_VERSION)) {
        Serial.printf("[OTA] Version %s not greater than %s — skipping\n",
            newVersion.c_str(), FIRMWARE_VERSION);
        return;
    }
    if (url.length() == 0) return;

    if (_jActive && _jVersion != newVersion) {
        // offer changed under an in-flight journey (re-pin): drop the stale one
        _journeyAbort("offer changed", false);
    }
    if (!_jActive) _journeyStart(newVersion, url, md5, forced);
    else _jForced = forced || _jForced;
}

// OTA runs on Core 0 inside networkTask
void otaLoop() {
    unsigned long now = millis();

    // FORCE request from the command handler: check + run to completion/cap
    if (_otaForceRequested) {
        _otaForceRequested = false;
        Serial.println("[OTA] FORCED update requested");
        _checkAndMaybeStart(true);
        if (_jActive) {
            _jForced = true;
            _otaForcedActive = true;
            _runForced();
            _otaForcedActive = false;
        } else {
            Serial.println("[OTA] Force: nothing to update");
        }
        return;
    }

    // In-flight AUTO journey: spend one slice on it
    if (_jActive) {
        _runAutoSlice();
        return;
    }

    // Hourly (or nudged) check — backoff applies only to auto starts
    if (_otaForceCheck || (now - _lastOTACheck >= OTA_CHECK_INTERVAL_MS)) {
        _lastOTACheck = now;
        _otaForceCheck = false;
        if (_otaNextAutoMs && now < _otaNextAutoMs) return;   // backing off
        if (!_autoGatesOpen()) return;                        // sick link: wait
        _checkAndMaybeStart(false);
        if (_jActive) _runAutoSlice();
    }
}
