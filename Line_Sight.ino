/*
 * LineSights LS-6C-IOT_V1.0 Power Monitor Firmware
 * ESP32-WROOM-32E | 6-channel CT current sensing
 *
 * Reads CT sensors, calculates RMS power, sends to server via HTTPS.
 * Features: WiFi AP setup portal, OTA updates, buffered sending, watchdog.
 */

#define FIRMWARE_VERSION "1.0.0"

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
#define BOOT_BUTTON_PIN 0
#define FACTORY_RESET_HOLD_MS 5000

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
        return;  // Stay in AP mode, loop() will handle it
    }

    // 4. If WiFi connected, init networking
    if (isWiFiConnected()) {
        syncNTP();
        initHTTPSender(getServerUrl());
        initOTAUpdater(getDeviceId(), getServerUrl());
        initStatusServer();
    }

    // 5. CT sensors
    initCTSensors();

    // 6. Ready
    setLEDState(LED_RUNNING);

    Serial.println();
    Serial.println("--- Configuration ---");
    Serial.printf("  Device ID:  %s\n", getDeviceId().c_str());
    Serial.printf("  Location:   %s\n", getLocationName().c_str());
    Serial.printf("  Server:     %s\n", getServerUrl().c_str());
    Serial.printf("  Voltage:    %.0fV\n", getGridVoltage());
    Serial.printf("  CT Ratings: %dA/%dA/%dA/%dA/%dA/%dA\n",
        getCtRating(0), getCtRating(1), getCtRating(2),
        getCtRating(3), getCtRating(4), getCtRating(5));
    Serial.printf("  Interval:   %ds\n", getSendInterval());
    Serial.printf("  Timezone:   %s\n", getTimezone().c_str());
    Serial.println("---------------------");
    Serial.println("Monitoring started...");
    Serial.println();
}

void loop() {
    unsigned long loopStart = millis();

    // Always: watchdog + LED
    feedWatchdog();
    updateLED();

    // Serial command: type "reset" to factory reset
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "update") {
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
            Serial.println("[RESET] Clearing config and rebooting to AP mode...");
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
        isWiFiConnected();  // This handles AP server
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
        // Re-init services after reconnect
        syncNTP();
    }

    // Read CT sensors at configured interval
    unsigned long now = millis();
    int intervalMs = getSendInterval() * 1000;

    if (now - lastReadingTime >= (unsigned long)intervalMs) {
        lastReadingTime = now;

        AllCTReadings readings = readAllCT(getGridVoltage());
        updateLastReadings(readings);

        // Log summary with enhanced metrics
        bool anyActive = false;
        for (int i = 0; i < NUM_CT_CHANNELS; i++) {
            if (readings.ct[i].rms_power_w > 0) {
                anyActive = true;
                Serial.printf("  CT%d: %.1fW | %.3fA rms | %.3fA peak | CF:%.2f | %.1fHz\n",
                    i + 1,
                    readings.ct[i].rms_power_w,
                    readings.ct[i].rms_current_a,
                    readings.ct[i].peak_current_a,
                    readings.ct[i].crest_factor,
                    readings.ct[i].frequency_hz);
            }
        }

        if (anyActive) {
            Serial.printf("[%s] Total:%.1fW | Sampled:%lums | Queue:%d\n",
                getUTCTimestamp().c_str(),
                readings.total_power_w,
                readings.sample_duration_ms,
                getQueueSize());
        } else {
            Serial.printf("[%s] No active loads | Queue:%d\n",
                getUTCTimestamp().c_str(),
                getQueueSize());
        }

        // Queue for sending
        if (isWiFiConnected()) {
            String ts = getUTCTimestamp();
            queueReading(getDeviceId(), getLocationName(), getTimezone(),
                         getGridVoltage(), readings.ct, ts);
            setLEDState(LED_SENDING);
        }
    }

    // Process send queue (non-blocking, sends one per loop)
    if (isWiFiConnected()) {
        processSendQueue();
        if (getQueueSize() == 0 && getLEDState() == LED_SENDING) {
            setLEDState(LED_RUNNING);
        }
    }

    // OTA check (periodic, non-blocking)
    if (isWiFiConnected()) {
        otaLoop();
    }

    // Web status server
    handleStatusServer();

    // Diagnostics
    diagnosticsLoop();

    // Record loop timing
    unsigned long loopTime = millis() - loopStart;
    recordLoopTime(loopTime);
}
