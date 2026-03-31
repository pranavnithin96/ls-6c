/*
 * LineSights LS-6C-IOT_V1.0 Power Monitor Firmware
 * ESP32-WROOM-32E | 6-channel CT current sensing
 *
 * Dual-core architecture:
 *   Core 1 (main loop): CT sampling, LED, serial commands, watchdog
 *   Core 0 (network task): HTTP POST, heartbeat, OTA, SPIFFS, web server
 */

#define FIRMWARE_VERSION "1.7.4"

#include <Preferences.h>
#include "led_status.h"
#include "diagnostics.h"
#include "wifi_manager.h"
#include "ct_sensor.h"
#include "http_sender.h"
#include "ota_updater.h"
#include "web_status.h"
#include "heartbeat.h"

static unsigned long lastReadingTime = 0;
static unsigned long bootButtonPressStart = 0;
static AllCTReadings lastReadings = {};
static volatile bool networkReady = false;

#define BOOT_BUTTON_PIN 0
#define FACTORY_RESET_HOLD_MS 5000
#define NETWORK_TASK_STACK 8192

// Shared queue: main loop writes readings, network task sends them
#define READING_QUEUE_SIZE 10
static QueueHandle_t readingQueue = NULL;

struct QueuedReading {
    AllCTReadings readings;
    char timestamp[32];
};

// =============================================
// Network Task — runs on Core 0
// =============================================
void networkTask(void* param) {
    Serial.println("[NET] Network task started on Core 0");

    // Add Core 0 to watchdog (120s timeout matches Core 1)
    esp_task_wdt_add(NULL);

    for (;;) {
        esp_task_wdt_reset();  // Feed Core 0 watchdog
        // Send queued CT readings
        QueuedReading qr;
        while (xQueueReceive(readingQueue, &qr, 0) == pdTRUE) {
            if (isWiFiConnected()) {
                String ts = String(qr.timestamp);
                queueReading(getDeviceId(), getLocationName(), getTimezone(),
                             getGridVoltage(), qr.readings.ct, ts);
            }
        }

        // Process HTTP send queue (up to 5 per iteration)
        if (isWiFiConnected()) {
            processSendQueue();
        }

        // Heartbeat
        if (isWiFiConnected()) {
            heartbeatLoop();
        }

        // OTA
        if (isWiFiConnected()) {
            otaLoop();
        }

        // Web status server
        handleStatusServer();

        // Diagnostics health report
        diagnosticsLoop();

        // Small yield to prevent watchdog on Core 0
        vTaskDelay(1);
    }
}

// =============================================
// Status Print
// =============================================
void printStatus() {
    Serial.println("\n=== Device Status ===");
    Serial.printf("  Firmware:   v%s (dual-core)\n", FIRMWARE_VERSION);
    Serial.printf("  Device ID:  %s\n", getDeviceId().c_str());
    Serial.printf("  Location:   %s\n", getLocationName().c_str());
    Serial.printf("  WiFi:       %s (RSSI: %d dBm)\n",
        WiFi.status() == WL_CONNECTED ? "connected" : "disconnected", WiFi.RSSI());
    Serial.printf("  IP:         %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Server:     %s\n", getServerUrl().c_str());
    Serial.printf("  Uptime:     %lus\n", getUptimeSeconds());
    Serial.printf("  Free heap:  %u bytes\n", ESP.getFreeHeap());
    Serial.printf("  Queue:      %d | Sent: %d | Failed: %d | Dropped: %d\n",
        getQueueSize(), getTotalSent(), getTotalFailed(), getTotalDropped());
    Serial.printf("  CT cal:     %s | Multi-point: %s\n",
        isCTCalibrated() ? "yes" : "no", isMultiCalLoaded() ? "yes" : "no");
    Serial.println("  --- CT Readings ---");
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        Serial.printf("  CT%d: %.3fA | %.1fW | PF:%.1f | %dmV | var:%d\n",
            i + 1, lastReadings.ct[i].amps, lastReadings.ct[i].watts,
            lastReadings.ct[i].pf, lastReadings.ct[i].avg_mv, lastReadings.ct[i].variation);
    }
    Serial.printf("  Total: %.1fW\n", lastReadings.total_watts);
    Serial.println("=====================\n");
}

// =============================================
// Setup — runs on Core 1
// =============================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.printf("  LineSights Power Monitor v%s\n", FIRMWARE_VERSION);
    Serial.println("  LS-6C-IOT_V1.0 | ESP32-WROOM-32E");
    Serial.println("  Dual-core: CT on Core 1, HTTP on Core 0");
    Serial.println("========================================");

    // Check firmware rollback (before anything else)
    checkFirmwareRollback();

    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    initLED();
    setLEDState(LED_BOOTING);

    initDiagnostics();

    feedWatchdog();
    setLEDState(LED_WIFI_CONNECTING);
    initWiFiManager();
    feedWatchdog();

    if (isAPMode()) {
        setLEDState(LED_WIFI_AP_MODE);
        Serial.println();
        Serial.println("*** SETUP MODE ***");
        Serial.println("Connect to WiFi: LineSights-Setup");
        Serial.println("Open browser: http://192.168.4.1");
        Serial.println("******************");
        return;
    }

    // Init networking
    if (isWiFiConnected()) {
        syncNTP();
        initHTTPSender(getServerUrl());
        initOTAUpdater(getDeviceId(), getServerUrl());
        initHeartbeat(getServerUrl());
        initStatusServer();
        networkReady = true;
    }

    // CT sensors
    initCTSensors();

    // Create reading queue
    readingQueue = xQueueCreate(READING_QUEUE_SIZE, sizeof(QueuedReading));

    // Launch network task on Core 0
    if (networkReady) {
        xTaskCreatePinnedToCore(
            networkTask,          // function
            "NetworkTask",        // name
            NETWORK_TASK_STACK,   // stack size
            NULL,                 // parameter
            1,                    // priority
            NULL,                 // handle
            0                     // Core 0
        );
    }

    setLEDState(LED_RUNNING);

    Serial.println();
    Serial.println("--- Configuration ---");
    Serial.printf("  Device ID:  %s\n", getDeviceId().c_str());
    Serial.printf("  Location:   %s\n", getLocationName().c_str());
    Serial.printf("  Server:     %s\n", getServerUrl().c_str());
    Serial.printf("  Voltage:    %.0fV\n", getGridVoltage());
    Serial.printf("  Interval:   %ds\n", getSendInterval());
    Serial.printf("  Timezone:   %s\n", getTimezone().c_str());
    Serial.println("---------------------");
    Serial.println("Commands: status | calibrate | calpoint | debug | reset | update");
    Serial.println("Monitoring started...\n");
}

// =============================================
// Main Loop — runs on Core 1 (CT sampling only)
// =============================================
void loop() {
    unsigned long loopStart = millis();

    feedWatchdog();
    updateLED();

    // Serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "status") {
            printStatus();
        } else if (cmd == "calibrate") {
            Serial.println("[CMD] Re-running CT zero calibration...");
            calibrateCTZero();
        } else if (cmd == "debug") {
            setHTTPDebug(!getHTTPDebug());
            Serial.printf("[CMD] HTTP debug: %s\n", getHTTPDebug() ? "ON" : "OFF");
        } else if (cmd == "update") {
            Serial.println("[OTA] Forcing update check...");
            checkForUpdate();
        } else if (cmd.startsWith("calpoint ")) {
            // Usage: calpoint <ch 1-6> <point 0-2> <known_amps>
            // Example: calpoint 1 0 0.5   (CH1, low point, 0.5A actual)
            //          calpoint 1 1 5.0   (CH1, mid point, 5.0A actual)
            //          calpoint 1 2 25.0  (CH1, high point, 25.0A actual)
            int ch = 0, pt = 0;
            float amps = 0;
            if (sscanf(cmd.c_str(), "calpoint %d %d %f", &ch, &pt, &amps) == 3) {
                Serial.printf("[CMD] Setting cal: CH%d point %d = %.3fA\n", ch, pt, amps);
                setCalPoint(ch - 1, pt, amps);  // ch is 1-based from user
            } else {
                Serial.println("[CMD] Usage: calpoint <ch 1-6> <point 0-2> <amps>");
                Serial.println("  Example: calpoint 1 0 0.5  (low)");
                Serial.println("           calpoint 1 1 5.0  (mid)");
                Serial.println("           calpoint 1 2 25.0 (high)");
            }
        } else if (cmd == "reset") {
            Serial.println("\n[RESET] Factory reset via serial command!");
            Preferences resetPrefs;
            resetPrefs.begin("lscfg", false);
            resetPrefs.clear();
            resetPrefs.end();
            delay(500);
            ESP.restart();
        }
    }

    // Factory reset button
    if (digitalRead(BOOT_BUTTON_PIN) == LOW) {
        if (bootButtonPressStart == 0) {
            bootButtonPressStart = millis();
        } else if (millis() - bootButtonPressStart >= FACTORY_RESET_HOLD_MS) {
            Serial.println("\n[RESET] Factory reset triggered!");
            Preferences resetPrefs;
            resetPrefs.begin("lscfg", false);
            resetPrefs.clear();
            resetPrefs.end();
            delay(1000);
            ESP.restart();
        }
    } else {
        bootButtonPressStart = 0;
    }

    // AP mode
    if (isAPMode()) {
        isWiFiConnected();
        delay(10);
        return;
    }

    // WiFi reconnect
    if (!isWiFiConnected()) {
        if (getLEDState() != LED_WIFI_DISCONNECTED) {
            setLEDState(LED_WIFI_DISCONNECTED);
            recordWiFiReconnect();
            logError("WiFi disconnected, reconnecting");
        }
    } else if (getLEDState() == LED_WIFI_DISCONNECTED) {
        setLEDState(LED_RUNNING);
        syncNTP();
    }

    // ===== CT SAMPLING (this is all Core 1 does in the timed section) =====
    unsigned long now = millis();
    int intervalMs = getSendInterval() * 1000;

    if (now - lastReadingTime >= (unsigned long)intervalMs) {
        lastReadingTime += intervalMs;
        if (now - lastReadingTime > (unsigned long)intervalMs) {
            lastReadingTime = now;
        }

        // Sample all 6 CT channels (~500ms interleaved)
        lastReadings = readAllCT(getGridVoltage());
        updateLastReadings(lastReadings);

        // Log
        bool anyActive = false;
        for (int i = 0; i < NUM_CT_CHANNELS; i++) {
            if (lastReadings.ct[i].amps > 0) {
                anyActive = true;
                Serial.printf("  CT%d: %.3fA | %.1fW | %dmV | var:%d\n",
                    i + 1,
                    lastReadings.ct[i].amps,
                    lastReadings.ct[i].watts,
                    lastReadings.ct[i].avg_mv,
                    lastReadings.ct[i].variation);
            }
        }

        if (anyActive) {
            Serial.printf("[%s] Total:%.1fW | %lums | Q:%d | S:%d\n",
                getUTCTimestamp().c_str(),
                lastReadings.total_watts,
                lastReadings.sample_duration_ms,
                getQueueSize(),
                getTotalSent());
        } else {
            Serial.printf("[%s] Idle | Q:%d | S:%d\n",
                getUTCTimestamp().c_str(),
                getQueueSize(),
                getTotalSent());
        }

        // Push to network task queue (non-blocking)
        if (networkReady && readingQueue != NULL) {
            QueuedReading qr;
            qr.readings = lastReadings;
            String ts = getUTCTimestamp();
            strncpy(qr.timestamp, ts.c_str(), sizeof(qr.timestamp) - 1);
            qr.timestamp[sizeof(qr.timestamp) - 1] = '\0';
            if (xQueueSend(readingQueue, &qr, 0) != pdTRUE) {
                Serial.println("[WARN] Reading queue full, network task lagging");
            }
            setLEDState(LED_SENDING);
        }
    }

    // Record loop timing
    unsigned long loopTime = millis() - loopStart;
    recordLoopTime(loopTime);
}
