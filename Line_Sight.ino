/*
 * LineSights LS-6C-IOT v2.0.0 — Industrial Power Monitor
 * ESP32-WROOM-32E | 6-channel CT current sensing
 *
 * Architecture:
 *   Core 1 (main):  Phase-locked 1s CT sampling, serial commands, button
 *   Core 0 (net):   Serialized HTTP POST, heartbeat, OTA, diagnostics
 *   Shared data:    Ring buffer protected by mutex
 *
 * Key fixes in v2.0.0:
 *   - Mutex-protected ring buffer (was created but never used)
 *   - Guaranteed http.end() on all code paths (socket leak fix)
 *   - Fixed calibration math: 0.01627 A/mV (was 0.01526, -6.2% error)
 *   - Timestamps at sample time, not queue time (was 950ms off)
 *   - LittleFS with atomic writes (was SPIFFS with corruption risk)
 *   - Removed web server in station mode (was leaking sockets)
 *   - WiFi uses reconnect() with exponential backoff + jitter
 *   - OTA: partition size check, version comparison, download integrity
 *   - AP portal: PIN authentication (was open to anyone)
 *   - Thread-safe LED and error log (was unprotected cross-core)
 *   - Reduced WDT to 60s, uint32_t counters, broader crash detection
 *   - Non-blocking NTP, mDNS lifecycle cleanup, reconnect jitter
 */

#include <Preferences.h>
#include "config.h"
#include "ct_channel_config.h"
#include "led_status.h"
#include "diagnostics.h"
#include "wifi_manager.h"
#include "ct_sensor.h"
#include "http_sender.h"
#include "ota_updater.h"
#include "web_status.h"
#include "heartbeat.h"

// ============================================================================
// Shared State
// ============================================================================
static SemaphoreHandle_t bufferMutex = NULL;
static volatile bool networkReady = false;
static volatile bool wifiConnected = false;  // Updated ONLY by Core 0
static unsigned long lastReadingTime = 0;
static unsigned long bootButtonPressStart = 0;
static AllCTReadings lastReadings = {};

// ============================================================================
// Network Task — Core 0, serialized operations, vTaskDelayUntil
// ============================================================================
void networkTask(void* param) {
    Serial.println("[NET] Core 0 network task started");
    esp_task_wdt_add(NULL);

    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(100);  // 100ms deterministic cycle

    for (;;) {
        vTaskDelayUntil(&xLastWake, xPeriod);
        esp_task_wdt_reset();

        // WiFi management — ONLY Core 0 calls isWiFiConnected()
        wifiConnected = isWiFiConnected();

        if (wifiConnected) {
            // Feed the WDT BETWEEN subsystems, not just at the top of the
            // cycle. Under a stalled router every endpoint stalls at once,
            // and the serialized worst cases stack: _livePost ~40s (2
            // attempts x connect 5s + write 10 retries x 1s select + read
            // 5s, measured from the 2.0.17 core) + heartbeat ~20s + drain
            // ~20s > the 60s WDT — the reboot-every-~100s loop observed at
            // Meton 2026-07-25. No SINGLE call can reach 60s, so feeding
            // between calls removes the accumulation trip while a genuine
            // hang inside one call still resets, as a watchdog should.
            wdtCheckpoint(WDT_CP_SENDQUEUE);
            processSendQueue();   // Data POST — highest priority
            esp_task_wdt_reset();
            wdtCheckpoint(WDT_CP_HEARTBEAT);
            heartbeatLoop();      // Heartbeat — every 60s
            esp_task_wdt_reset();
            wdtCheckpoint(WDT_CP_OTA);
            otaLoop();            // OTA check — every 1h
            esp_task_wdt_reset();
            wdtCheckpoint(WDT_CP_WFSTREAM);
            wbStreamLoop();       // lossless batched 1kHz frames (2.18.3 pilot)
            esp_task_wdt_reset();
        }

        // Buffer save runs ALWAYS, even when WiFi is down
        // (processSendQueue skips this when disconnected)
        wdtCheckpoint(WDT_CP_BUFSAVE);
        periodicBufferSave();
        wdtCheckpoint(WDT_CP_IDLE);

        diagnosticsLoop();
        scheduledRebootLoop();    // 7-day maintenance reboot, safe-gated
    }
}

// ============================================================================
// Status Print
// ============================================================================
void printStatus() {
    AllCTReadings r = getLastReadings();

    Serial.println("\n=== Device Status ===");
    Serial.printf("  Firmware:  v%s\n", FIRMWARE_VERSION);
    Serial.printf("  Device:    %s @ %s\n", getDeviceId().c_str(), getLocationName().c_str());
    Serial.printf("  WiFi:      %s (RSSI: %d dBm)\n",
        WiFi.status() == WL_CONNECTED ? "connected" : "disconnected", WiFi.RSSI());
    Serial.printf("  IP:        %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Server:    %s\n", getServerUrl().c_str());
    Serial.printf("  Uptime:    %lus | Heap: %u\n", getUptimeSeconds(), ESP.getFreeHeap());
    Serial.printf("  Queue:     %d | Sent: %u | Failed: %u | Dropped: %u\n",
        getQueueSize(), getTotalSent(), getTotalFailed(), getTotalDropped());
    Serial.printf("  Cal:       %s | MultiCal: %s\n",
        isCTCalibrated() ? "yes" : "no", isMultiCalLoaded() ? "yes" : "no");
    Serial.printf("  CT config: rev=%u mask=0x%02X%s\n",
        (unsigned)getCTConfigRevision(), getActiveCTMask(),
        isCTConfigRequired() ? " REQUIRED" : "");
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        if (isCTChannelEnabled(i))
            Serial.printf("  CT%d: %.3fA | %.1fW | %dcount\n",
                i + 1, r.ct[i].amps, r.ct[i].watts, r.ct[i].avg_mv);
        else
            Serial.printf("  CT%d: disabled\n", i + 1);
    }
    Serial.printf("  Total: %.1fW\n", r.total_watts);
    Serial.println("=====================\n");
}

// ============================================================================
// Setup — Proper boot order: mutex FIRST, then everything else
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.printf("  LineSights Power Monitor v%s\n", FIRMWARE_VERSION);
    Serial.println("  LS-6C-IOT_V1.0 | ESP32-WROOM-32E");
    Serial.println("========================================");

    // 1. Firmware rollback check (must be first)
    checkFirmwareRollback();

    // 2. Create mutex BEFORE anything that might use it
    bufferMutex = xSemaphoreCreateMutex();

    // 3. Hardware init
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    initLED();
    setLEDState(LED_BOOTING);
    initReadingsMutex();

    // 4. Diagnostics & watchdog
    initDiagnostics();
    initScheduledReboot();
    feedWatchdog();

    // 5. CT channel configuration, then WiFi/provisioning. A fresh unit starts
    // with no active ADC inputs until the installer explicitly selects them.
    initCTChannelConfig();

    // 6. WiFi
    setLEDState(LED_WIFI_CONNECTING);
    initWiFiManager();
    feedWatchdog();

    if (isAPMode()) {
        setLEDState(LED_WIFI_AP_MODE);
        Serial.println("\n*** SETUP MODE ***");
        Serial.println("Connect to WiFi shown above, open http://192.168.4.1");
        Serial.println("Enter the PIN shown above to save configuration");
        Serial.println("******************\n");
        return;
    }

    // 7. ALWAYS init sender + LittleFS (even without WiFi — needed for offline storage)
    initHTTPSender(getServerUrl(), getDeviceId());
    setBufferMutex(bufferMutex);
    initHeartbeat(getServerUrl());
    initOTAUpdater(getDeviceId(), getServerUrl());
    initStatusServer();

    if (isWiFiConnected()) {
        syncNTP();
        feedWatchdog();
    }

    networkReady = true;  // ALWAYS true — Core 0 checks WiFi per-operation

    // 8. CT sensors
    initCTSensors();
    wbInit();               // WFS2 complete-frame queue — one-time heap alloc
    feedWatchdog();

    // 9. ALWAYS launch Core 0 task — it handles WiFi reconnect + data POST when ready
    xTaskCreatePinnedToCore(networkTask, "Net", NETWORK_TASK_STACK, NULL, 1, NULL, 0);

    setLEDState(LED_RUNNING);

    Serial.println("\n--- Configuration ---");
    Serial.printf("  Device:    %s\n", getDeviceId().c_str());
    Serial.printf("  Location:  %s\n", getLocationName().c_str());
    Serial.printf("  Server:    %s\n", getServerUrl().c_str());
    Serial.printf("  Voltage:   %.0fV | Interval: %ds\n", getGridVoltage(), getSendInterval());
    Serial.printf("  Active CT: 0x%02X | Config rev: %u%s\n", getActiveCTMask(),
                  (unsigned)getCTConfigRevision(),
                  isCTConfigRequired() ? " | CONFIGURATION REQUIRED" : "");
    Serial.println("---------------------");
    Serial.println("Commands: status | setslope | slopes | calpoint | debug | reset | update");
    Serial.println("Monitoring started...\n");
}

// ============================================================================
// Main Loop — Core 1, phase-locked sampling
// ============================================================================
void loop() {
    feedWatchdog();
    updateLED();

    // --- Serial Commands (non-blocking — never stalls sampling) ---
    static char _cmdBuf[129];
    static int _cmdLen = 0;
    static bool _cmdReady = false;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (_cmdLen > 0) _cmdReady = true;
            break;
        }
        if (_cmdLen < 128) _cmdBuf[_cmdLen++] = c;
    }
    if (_cmdReady) {
        _cmdBuf[_cmdLen] = '\0';
        String cmd = String(_cmdBuf);
        _cmdLen = 0;
        _cmdReady = false;
        cmd.trim();
        feedWatchdog();

        if (cmd == "status") {
            printStatus();
        } else if (cmd == "debug") {
            setHTTPDebug(!getHTTPDebug());
            Serial.printf("[CMD] HTTP debug: %s\n", getHTTPDebug() ? "ON" : "OFF");
        } else if (cmd.startsWith("calpoint ")) {
            int ch = 0, pt = 0; float amps = 0;
            if (sscanf(cmd.c_str(), "calpoint %d %d %f", &ch, &pt, &amps) == 3) {
                setCalPoint(ch - 1, pt, amps);
            } else {
                Serial.println("Usage: calpoint <ch 1-6> <point 0-2> <amps>");
            }
        } else if (cmd.startsWith("setslope ")) {
            int ch = 0; float slope = -1;
            if (sscanf(cmd.c_str(), "setslope %d %f", &ch, &slope) == 2 &&
                setChannelSlope(ch - 1, slope)) {
                // setChannelSlope prints the confirmation
            } else {
                Serial.println("Usage: setslope <ch 1-6> <A/count, 0 clears>  e.g. setslope 1 0.0421");
            }
        } else if (cmd == "slopes") {
            Serial.println("Per-channel CT slopes (0 = rating default):");
            for (int i = 0; i < NUM_CT_CHANNELS; i++) {
                float s = getChannelSlope(i);
                if (s > 0) Serial.printf("  CH%d: %.5f A/count (calibrated)\n", i + 1, s);
                else       Serial.printf("  CH%d: rating default (%dA)\n", i + 1, getCtRating(i));
            }
        } else if (cmd == "test_offline") {
            int testSecs = 25;  // 25 readings = 2 full blocks (10 each) + 5 partial
            Serial.printf("[TEST] Storing %d offline readings...\n", testSecs);
            _offlineTestLock = true;
            enterOfflineMode(getDeviceId());
            int count = 0;
            while (count < testSecs) {
                feedWatchdog();
                if (millis() - lastReadingTime >= 1000) {
                    lastReadingTime = millis();
                    lastReadings = readAllCT(getGridVoltage());
                    storeOfflineReading(lastReadings.ct);
                    count++;
                    Serial.printf("[TEST] %d/%d stored:%u file:%uB heap:%u\n",
                        count, testSecs, getOfflineStored(),
                        getOfflineFileSize(), ESP.getFreeHeap());
                }
            }
            Serial.printf("[TEST] Done. Releasing offline mode — upload should follow\n");
            _offlineTestLock = false;
            exitOfflineMode();
        } else if (cmd == "update") {
            forceOTACheck();
        } else if (cmd == "reset") {
            Serial.println("Type 'reset_confirm' to factory reset");
        } else if (cmd == "reset_confirm") {
            Serial.println("[RESET] Factory reset!");
            flushBeforeRestart();
            Preferences p; p.begin("lscfg", false); p.clear(); p.end();
            delay(500);
            ESP.restart();
        }
    }

    // --- Factory Reset Button (hold BOOT 5s) ---
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        if (bootButtonPressStart == 0) {
            bootButtonPressStart = millis();
        } else if (millis() - bootButtonPressStart >= FACTORY_RESET_HOLD_MS) {
            Serial.println("[RESET] Button factory reset!");
            flushBeforeRestart();
            feedWatchdog();
            Preferences p; p.begin("lscfg", false); p.clear(); p.end();
            delay(1000);
            ESP.restart();
        }
    } else {
        bootButtonPressStart = 0;
    }

    // --- AP Mode Loop ---
    if (isAPMode()) {
        isWiFiConnected();  // Processes DNS + AP server
        delay(10);
        return;
    }

    // --- WiFi State + Offline Mode (reads volatile wifiConnected from Core 0) ---
    static unsigned long wifiDownStart = 0;
    static bool wasConnected = false;
    if (!wifiConnected) {
        if (wasConnected) {
            // Blink while reconnecting, not off — a dark LED reads as a dead board.
            // Solid = connected, blinking = trying to reconnect, off = no power.
            setLEDState(LED_WIFI_CONNECTING);
            recordWiFiReconnect();
            logError("WiFi disconnected");
            wasConnected = false;
        }
        if (wifiDownStart == 0) wifiDownStart = millis();

        // Enter offline mode after grace period
        if (!isOfflineMode() && (millis() - wifiDownStart > OFFLINE_GRACE_MS)) {
            // Only set the flag — don't do saveBufferToFlash here (Core 0 handles it)
            enterOfflineMode(getDeviceId());
        }
    } else {
        if (!wasConnected) {
            setLEDState(LED_RUNNING);
            wasConnected = true;
        }
        wifiDownStart = 0;
    }

    // --- NTP retry: never give up while unsynced ---
    // Post-outage the router/WAN often comes up minutes AFTER the ESP32; the old
    // max-5-then-stop left the clock unsynced for the whole uptime, so every
    // offline block was flagged UNSYNCED and staged unresolved server-side.
    // First 5 tries at 30s, then every 15 min. Once synced, zero further cost.
    static unsigned long lastNTPRetry = 0;
    static int ntpRetries = 0;
    static bool ntpSynced = false;
    if (wifiConnected && !ntpSynced &&
        millis() - lastNTPRetry > (ntpRetries < 5 ? 30000UL : 900000UL)) {
        struct tm t;
        if (getLocalTime(&t, 0)) {
            ntpSynced = true;
        } else {
            lastNTPRetry = millis();
            ntpRetries++;
            syncNTP();
        }
    } else if (wifiConnected && ntpSynced &&
               (getNtpSyncAgeS() < 0 || getNtpSyncAgeS() > (long)NTP_STALE_RESYNC_S) &&
               millis() - lastNTPRetry > 60000) {
        // SNTP should auto-resync every 15 min; if it silently stalls for 2h,
        // re-kick it. age < 0 (never synced this session) matters after a
        // SOFTWARE reboot: the RTC carries time across ESP.restart, so the
        // clock reads synced without SNTP ever being started — without this,
        // a post-scheduled-reboot uptime would run on pure RTC drift.
        lastNTPRetry = millis();
        syncNTP();
    }

    // ===== CT SAMPLING — complete one-second frame, continuously =====
    // Sampling cadence is no longer coupled to telemetry cadence. A device may
    // POST the summary every N seconds, but WFS2 still observes every second.
    unsigned long now = millis();
    const unsigned long intervalMs = 1000;

    if (now - lastReadingTime >= (unsigned long)intervalMs) {
        lastReadingTime += intervalMs;
        if (now - lastReadingTime > (unsigned long)intervalMs) {
            lastReadingTime = now;
        }

        lastReadings = readAllCT(getGridVoltage());
        updateLastReadings(lastReadings);
        // Black-box capture request (if any) is honored HERE, between sampling
        // windows, so its flash write can't distort a window in progress.
        wbServiceCapture(getSampleWindowMs());

        static unsigned long lastTelemetrySampleMs = 0;
        const unsigned long telemetryIntervalMs = (unsigned long)getSendInterval() * 1000UL;
        bool telemetryDue = lastTelemetrySampleMs == 0 ||
                            lastReadings.timestamp_ms - lastTelemetrySampleMs >= telemetryIntervalMs;
        if (telemetryDue) lastTelemetrySampleMs = lastReadings.timestamp_ms;

        if (telemetryDue && isOfflineMode()) {
            // OFFLINE: Store to compressed binary on flash
            storeOfflineReading(lastReadings.ct);

            // Log every 10th reading
            static int offlineLogCount = 0;
            if (++offlineLogCount >= 10) {
                offlineLogCount = 0;
                Serial.printf("[OFFLINE] %.1fW | Stored:%u | File:%uKB | %lums\n",
                    lastReadings.total_watts, getOfflineStored(),
                    getOfflineFileSize() / 1024, lastReadings.sample_duration_ms);
            }
        } else if (telemetryDue) {
            // ONLINE: Queue for immediate POST
            Serial.printf("[%s] %.1fW | Q:%d S:%u%s | %lums\n",
                getUTCTimestamp().c_str(), lastReadings.total_watts,
                getQueueSize(), getTotalSent(),
                isUploadPending() ? " UPL" : "",
                lastReadings.sample_duration_ms);

            // Stamp at SAMPLE time (not queue time): queue-time labels collided
            // when phase drift compressed spacing under 1s and the server 400'd
            // the twin (observed fleet-wide at a ~61s cadence). With sample-time
            // stamps the only remaining collision source is a BACKWARD SNTP step
            // (oscillator ran fast) re-issuing an already-used second. Never drop
            // and never emit a duplicate label — clamp the label forward by 1s.
            // Skew is bounded by the step size (<1s in steady state) and clears
            // at the next forward step or send gap.
            static time_t lastQueuedEpoch = 0;
            time_t sampleEpoch = getEpochAt(lastReadings.timestamp_ms);
            if (sampleEpoch != 0 && lastQueuedEpoch != 0 && sampleEpoch <= lastQueuedEpoch) {
                sampleEpoch = lastQueuedEpoch + 1;
            }
            if (networkReady && bufferMutex) {
                // 100ms timeout — Core 0 holds mutex <10ms, so this always succeeds.
                // Old design used timeout=0 with offline fallback, but that created
                // orphaned readings in _blockBuf that never flushed and had wrong timestamps.
                if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    if (sampleEpoch != 0) lastQueuedEpoch = sampleEpoch;
                    String ts = formatUTCEpoch(sampleEpoch);
                    queueReading(getDeviceId(), getLocationName(), getTimezone(),
                                 getGridVoltage(), lastReadings.ct, ts,
                                 lastReadings.mains_hz,
                                 (int)lastReadings.sample_duration_ms);
                    xSemaphoreGive(bufferMutex);
                    // Ring-full overflow (if any) is written to /rejected.log HERE,
                    // outside the mutex — file I/O under bufferMutex starved Core 0's
                    // saveBufferToFlash and broke the <10ms hold invariant.
                    flushOverflowReading();
                    // Only force solid when actually connected — otherwise this
                    // ran every reading and stomped the reconnecting blink back to
                    // solid within ~1s (LED "blinked once then went solid").
                    if (wifiConnected) setLEDState(LED_RUNNING);
                } else {
                    // Should never happen (Core 0 holds <10ms, we wait 100ms)
                    // But if it does: store offline with proper timestamp tracking
                    storeOfflineReading(lastReadings.ct);
                    Serial.println("[WARN] Mutex timeout — reading stored offline");
                }
            }
        }
    }
}
