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
 *   - WiFi reset uses WIFI_OFF/ON (was disconnect which left orphans)
 *   - OTA: partition size check, version comparison, download integrity
 *   - AP portal: PIN authentication (was open to anyone)
 *   - Thread-safe LED and error log (was unprotected cross-core)
 *   - Reduced WDT to 60s, uint32_t counters, broader crash detection
 *   - Non-blocking NTP, mDNS lifecycle cleanup, reconnect jitter
 */

#include <Preferences.h>
#include "config.h"
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

        // 1. Data POST — highest priority
        if (isWiFiConnected()) {
            processSendQueue();
        }

        // 2. Heartbeat — every 60s
        if (isWiFiConnected()) {
            heartbeatLoop();
        }

        // 3. OTA check — every 1h
        if (isWiFiConnected()) {
            otaLoop();
        }

        // 4. Diagnostics
        diagnosticsLoop();
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
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        Serial.printf("  CT%d: %.3fA | %.1fW | %dmV\n",
            i + 1, r.ct[i].amps, r.ct[i].watts, r.ct[i].avg_mv);
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
    feedWatchdog();

    // 5. WiFi
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

    // 6. Network services (only if WiFi connected)
    if (isWiFiConnected()) {
        syncNTP();
        feedWatchdog();

        initHTTPSender(getServerUrl());
        setBufferMutex(bufferMutex);

        initOTAUpdater(getDeviceId(), getServerUrl());
        initHeartbeat(getServerUrl());
        initStatusServer();  // No-op in station mode (saves sockets)

        networkReady = true;
    }

    // 7. CT sensors
    initCTSensors();
    feedWatchdog();

    // 8. Launch network task on Core 0
    if (networkReady) {
        xTaskCreatePinnedToCore(networkTask, "Net", NETWORK_TASK_STACK, NULL, 1, NULL, 0);
    }

    setLEDState(LED_RUNNING);

    Serial.println("\n--- Configuration ---");
    Serial.printf("  Device:    %s\n", getDeviceId().c_str());
    Serial.printf("  Location:  %s\n", getLocationName().c_str());
    Serial.printf("  Server:    %s\n", getServerUrl().c_str());
    Serial.printf("  Voltage:   %.0fV | Interval: %ds\n", getGridVoltage(), getSendInterval());
    Serial.println("---------------------");
    Serial.println("Commands: status | calibrate | calpoint | debug | reset | update");
    Serial.println("Monitoring started...\n");
}

// ============================================================================
// Main Loop — Core 1, phase-locked sampling
// ============================================================================
void loop() {
    feedWatchdog();
    updateLED();

    // --- Serial Commands ---
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        feedWatchdog();

        if (cmd == "status") {
            printStatus();
        } else if (cmd == "calibrate") {
            calibrateCTZero();
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
        } else if (cmd == "update") {
            forceOTACheck();
        } else if (cmd == "reset") {
            Serial.println("Type 'reset_confirm' to factory reset");
        } else if (cmd == "reset_confirm") {
            Serial.println("[RESET] Factory reset!");
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

    // --- WiFi State + Offline Mode ---
    static unsigned long wifiDownStart = 0;
    if (!isWiFiConnected()) {
        if (getLEDState() != LED_WIFI_DISCONNECTED) {
            setLEDState(LED_WIFI_DISCONNECTED);
            recordWiFiReconnect();
            logError("WiFi disconnected");
        }
        if (wifiDownStart == 0) wifiDownStart = millis();

        // Enter offline mode after grace period
        if (!isOfflineMode() && (millis() - wifiDownStart > OFFLINE_GRACE_MS)) {
            enterOfflineMode(getDeviceId());
        }
    } else {
        wifiDownStart = 0;
        if (getLEDState() == LED_WIFI_DISCONNECTED) {
            setLEDState(LED_RUNNING);
            syncNTP();
        }
    }

    // --- NTP retry if timestamps are still epoch ---
    static unsigned long lastNTPRetry = 0;
    if (isWiFiConnected() && getUTCTimestamp().startsWith("1970") && millis() - lastNTPRetry > 30000) {
        lastNTPRetry = millis();
        syncNTP();
    }

    // ===== CT SAMPLING — Always 1Hz, route depends on online/offline =====
    unsigned long now = millis();
    int intervalMs = getSendInterval() * 1000;

    if (now - lastReadingTime >= (unsigned long)intervalMs) {
        lastReadingTime += intervalMs;
        if (now - lastReadingTime > (unsigned long)intervalMs) {
            lastReadingTime = now;
        }

        lastReadings = readAllCT(getGridVoltage());
        updateLastReadings(lastReadings);

        if (isOfflineMode()) {
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
        } else {
            // ONLINE: Queue for immediate POST
            Serial.printf("[%s] %.1fW | Q:%d S:%u%s | %lums\n",
                getUTCTimestamp().c_str(), lastReadings.total_watts,
                getQueueSize(), getTotalSent(),
                isUploadPending() ? " UPL" : "",
                lastReadings.sample_duration_ms);

            if (networkReady && bufferMutex) {
                if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    String ts = getUTCTimestamp();
                    queueReading(getDeviceId(), getLocationName(), getTimezone(),
                                 getGridVoltage(), lastReadings.ct, ts);
                    xSemaphoreGive(bufferMutex);
                    setLEDState(LED_SENDING);
                } else {
                    logError("Mutex timeout — reading not queued");
                }
            }
        }
    }
}
