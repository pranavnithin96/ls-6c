/*
 * LineSights LS-6C-IOT_V1.0 Power Monitor Firmware
 * ESP32-WROOM-32E | 6-channel CT current sensing
 *
 * Features: eFuse-calibrated ADC, auto-zero calibration, buffered HTTP,
 * WiFi AP setup portal, OTA updates, web dashboard, watchdog.
 */

#define FIRMWARE_VERSION "1.1.0"

#include <Preferences.h>
#include "led_status.h"
#include "diagnostics.h"
#include "wifi_manager.h"
#include "ct_sensor.h"
#include "http_sender.h"
#include "ota_updater.h"
#include "web_status.h"

static unsigned long lastReadingTime = 0;
static unsigned long bootButtonPressStart = 0;
static AllCTReadings lastReadings = {};
#define BOOT_BUTTON_PIN 0
#define FACTORY_RESET_HOLD_MS 5000

void printStatus() {
    Serial.println("\n=== Device Status ===");
    Serial.printf("  Firmware:   v%s\n", FIRMWARE_VERSION);
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
    Serial.printf("  CT cal:     %s\n", isCTCalibrated() ? "yes" : "no");
    Serial.println("  --- CT Readings ---");
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        Serial.printf("  CT%d: %.3fA | %.1fW | %dmV\n",
            i + 1, lastReadings.ct[i].amps, lastReadings.ct[i].watts, lastReadings.ct[i].avg_mv);
    }
    Serial.printf("  Total: %.1fW\n", lastReadings.total_watts);
    Serial.println("=====================\n");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.printf("  LineSights Power Monitor v%s\n", FIRMWARE_VERSION);
    Serial.println("  LS-6C-IOT_V1.0 | ESP32-WROOM-32E");
    Serial.println("========================================");

    // 0. Boot button for factory reset
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

    // 1. LED
    initLED();
    setLEDState(LED_BOOTING);

    // 2. Diagnostics & watchdog
    initDiagnostics();

    // 3. WiFi & config (may enter AP mode)
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

    // 4. If WiFi connected, init networking
    if (isWiFiConnected()) {
        syncNTP();
        initHTTPSender(getServerUrl());
        initOTAUpdater(getDeviceId(), getServerUrl());
        initStatusServer();
    }

    // 5. CT sensors (auto-zero happens on first read if uncalibrated)
    initCTSensors();

    // 6. Ready
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
    Serial.println("Commands: status | calibrate | debug | reset | update");
    Serial.println("Monitoring started...\n");
}

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

    // Factory reset: hold BOOT button for 5 seconds
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

    // AP mode: only serve config page
    if (isAPMode()) {
        isWiFiConnected();
        delay(10);
        return;
    }

    // Check WiFi
    if (!isWiFiConnected()) {
        if (getLEDState() != LED_WIFI_DISCONNECTED) {
            setLEDState(LED_WIFI_DISCONNECTED);
            recordWiFiReconnect();
        }
    } else if (getLEDState() == LED_WIFI_DISCONNECTED) {
        setLEDState(LED_RUNNING);
        syncNTP();
    }

    // Read CT sensors at configured interval
    unsigned long now = millis();
    int intervalMs = getSendInterval() * 1000;

    if (now - lastReadingTime >= (unsigned long)intervalMs) {
        lastReadingTime = now;

        lastReadings = readAllCT(getGridVoltage());
        updateLastReadings(lastReadings);

        // Log active channels
        bool anyActive = false;
        for (int i = 0; i < NUM_CT_CHANNELS; i++) {
            if (lastReadings.ct[i].amps > 0) {
                anyActive = true;
                Serial.printf("  CT%d: %.3fA | %.1fW | %dmV\n",
                    i + 1,
                    lastReadings.ct[i].amps,
                    lastReadings.ct[i].watts,
                    lastReadings.ct[i].avg_mv);
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

        // Queue for sending
        if (isWiFiConnected()) {
            String ts = getUTCTimestamp();
            queueReading(getDeviceId(), getLocationName(), getTimezone(),
                         getGridVoltage(), lastReadings.ct, ts);
            setLEDState(LED_SENDING);
        }
    }

    // Process send queue
    if (isWiFiConnected()) {
        processSendQueue();
        if (getQueueSize() == 0 && getLEDState() == LED_SENDING) {
            setLEDState(LED_RUNNING);
        }
    }

    // OTA check
    if (isWiFiConnected()) {
        otaLoop();
    }

    // Web status server
    handleStatusServer();

    // Diagnostics
    diagnosticsLoop();

    unsigned long loopTime = millis() - loopStart;
    recordLoopTime(loopTime);
}
