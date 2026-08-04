/*
 * LineSights LS-6C-IOT v3 refactor
 *
 * Core 1 owns acquisition. A completed reading is synchronously committed to
 * the CRC-protected WAL before its sequence advances. Core 0 owns every network
 * operation. Wi-Fi availability therefore changes delivery latency, never data
 * ownership.
 */

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "app_types.h"
#include "clock_service.h"
#include "config.h"
#include "durable_queue.h"
#include "heartbeat_service.h"
#include "led_service.h"
#include "measurement_service.h"
#include "ota_service.h"
#include "settings_service.h"
#include "transport_service.h"
#include "wifi_service.h"

namespace {

fw::SettingsService settings;
fw::ClockService clockService;
fw::DurableQueue durableQueue;
fw::MeasurementService measurement;
fw::WifiService wifi;
fw::TransportService transport;
fw::HeartbeatService heartbeat;
fw::OtaService ota;
fw::LedService led;
fw::ControlFlags controls;

TaskHandle_t networkTaskHandle = nullptr;
uint32_t nextSequence = 1;
uint32_t nextSampleMillis = 0;
fw::ElectricalReading pendingReading;
bool pendingCommit = false;
bool applicationReady = false;
uint32_t lastStorageRecoveryMillis = 0;
uint32_t lastReadingEpoch = 0;

const char* storageStateName(fw::StorageState state) {
    switch (state) {
        case fw::StorageState::Ready: return "ready";
        case fw::StorageState::CapacityBlocked: return "CAPACITY BLOCKED";
        case fw::StorageState::Fault: return "FAULT";
        default: return "uninitialized";
    }
}

void printStatus() {
    const fw::DeliveryStats stats = durableQueue.stats();
    Serial.println("\n=== LineSights status ===");
    Serial.printf("Firmware: %s | boot:%u | next sequence:%u\n",
                  fw::kFirmwareVersion, settings.bootId(), nextSequence);
    Serial.printf("Device: %s @ %s\n", settings.config().deviceId.c_str(),
                  settings.config().location.c_str());
    Serial.printf("Wi-Fi: %s | RSSI:%d | reconnects:%u\n",
                  wifi.connected() ? "connected" : "disconnected",
                  wifi.rssi(), wifi.reconnects());
    Serial.printf("Storage: %s | WAL:%u records/%u bytes | offline:%u | legacy JSON:%u | free:%u\n",
                  storageStateName(durableQueue.state()), durableQueue.walDepth(),
                  durableQueue.walBytes(), durableQueue.offlineBytes(),
                  durableQueue.legacyJsonBytes(), durableQueue.freeBytes());
    Serial.printf("Generated:%u durable:%u live ACK:%u bulk ACK:%u retries:%u local failures:%u losses:%u\n",
                  stats.generated, stats.durable, stats.liveAcknowledged,
                  stats.bulkAcknowledged, stats.retries, stats.localWriteFailures,
                  stats.permanentLosses);
    Serial.println("=========================\n");
}

void processSerial() {
    static char buffer[128];
    static size_t length = 0;
    while (Serial.available()) {
        const char value = Serial.read();
        if (value != '\n' && value != '\r') {
            if (length + 1 < sizeof(buffer)) buffer[length++] = value;
            continue;
        }
        if (length == 0) continue;
        buffer[length] = '\0';
        String command(buffer);
        length = 0;
        command.trim();
        if (command == "status") {
            printStatus();
        } else if (command == "update") {
            controls.otaRequested = true;
        } else if (command == "reboot") {
            controls.rebootRequested = true;
        } else if (command.startsWith("setinterval ")) {
            const int value = command.substring(12).toInt();
            Serial.println(settings.setInterval(value) ? "Interval updated" : "Invalid interval/write failure");
        } else if (command.startsWith("setvoltage ")) {
            const float value = command.substring(11).toFloat();
            Serial.println(settings.setVoltage(value) ? "Voltage updated" : "Invalid voltage/write failure");
        } else if (command.startsWith("setslope ")) {
            int channel = 0;
            float slope = -1;
            const bool parsed = sscanf(command.c_str(), "setslope %d %f", &channel, &slope) == 2;
            Serial.println(parsed && settings.setChannelSlope(channel - 1, slope)
                               ? "Slope updated" : "Usage: setslope <1-6> <0 or 0.001-0.5>");
        } else {
            Serial.println("Commands: status | update | reboot | setinterval N | setvoltage V | setslope CH VALUE");
        }
    }
}

void networkTask(void*) {
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        wifi.loop();
        const bool connected = wifi.connected();
        clockService.loop(connected);
        transport.loop(connected);
        heartbeat.loop(connected);
        ota.loop(connected, durableQueue.state() == fw::StorageState::Ready);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(fw::kNetworkLoopMs));
    }
}

void restartSafely() {
    // There is no RAM-only send queue: every completed reading is already in the
    // WAL. A best-effort checkpoint reduces duplicate replay after the reboot.
    durableQueue.checkpoint();
    delay(250);
    ESP.restart();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(250);
    Serial.printf("\nLineSights LS-6C-IOT %s\n", fw::kFirmwareVersion);
    led.begin();
    led.set(fw::LedState::Booting);

    if (!settings.begin()) {
        Serial.println("[FATAL] NVS state unavailable; acquisition is fail-closed");
        led.set(fw::LedState::StorageFault);
        return;
    }
    const bool storageReady = durableQueue.begin(settings.config().deviceId);
    if (!storageReady) {
        Serial.printf("[FATAL] Durable storage %s; no reading will be sampled and discarded\n",
                      storageStateName(durableQueue.state()));
    }

    measurement.begin(settings.config());
    clockService.begin();
    wifi.begin(settings);
    transport.begin(settings, clockService, durableQueue);
    heartbeat.begin(transport.heartbeatUrl(), settings, wifi, clockService,
                    durableQueue, controls);
    ota.begin(transport.firmwareBaseUrl(), settings.config().deviceId,
              durableQueue, controls);

    esp_task_wdt_init(fw::kWatchdogSeconds, true);
    esp_task_wdt_add(nullptr);
    xTaskCreatePinnedToCore(networkTask, "network", fw::kNetworkTaskStack, nullptr,
                            1, &networkTaskHandle, 0);
    nextSampleMillis = millis();
    applicationReady = storageReady && settings.isProvisioned();
    led.set(settings.isProvisioned() ? fw::LedState::Connecting : fw::LedState::Provisioning);
    Serial.println("Commands: status | update | reboot | setinterval N | setvoltage V | setslope CH VALUE");
}

void loop() {
    esp_task_wdt_reset();
    led.loop();
    processSerial();

    if (controls.rebootRequested) restartSafely();
    if (!applicationReady) {
        if (settings.isProvisioned() && durableQueue.filesystemReady() &&
            millis() - lastStorageRecoveryMillis >= 5000UL) {
            lastStorageRecoveryMillis = millis();
            applicationReady = durableQueue.recover();
            if (applicationReady) nextSampleMillis = millis();
        }
        led.set(wifi.provisioning() ? fw::LedState::Provisioning : fw::LedState::StorageFault);
        delay(10);
        return;
    }

    if (durableQueue.state() != fw::StorageState::Ready) {
        led.set(fw::LedState::StorageFault);
    } else if (ota.updating()) {
        led.set(fw::LedState::Updating);
    } else {
        led.set(wifi.connected() ? fw::LedState::Running : fw::LedState::Connecting);
    }

    if (pendingCommit) {
        if (durableQueue.commit(pendingReading)) {
            pendingCommit = false;
            ++nextSequence;
            Serial.printf("[SAMPLE] boot:%u seq:%u durable | WAL:%u\n",
                          pendingReading.bootId, pendingReading.sequence,
                          durableQueue.walDepth());
        } else {
            // Do not overwrite the only RAM copy and do not pretend later samples
            // are complete. Retry until storage accepts it or an operator expands
            // capacity; the heartbeat/LED expose this fail-closed state.
            delay(20);
            return;
        }
    }

    const uint32_t now = millis();
    const uint32_t period = settings.config().intervalSeconds * 1000UL;
    if (static_cast<int32_t>(now - nextSampleMillis) < 0) return;
    const uint32_t lateness = now - nextSampleMillis;
    if (lateness >= period) {
        durableQueue.recordCaptureGap(lateness / period);
        nextSampleMillis = now + period;
    } else {
        nextSampleMillis += period;
    }

    uint32_t epoch = clockService.epochAt(now);
    if (epoch != 0 && lastReadingEpoch != 0 && epoch <= lastReadingEpoch) {
        epoch = lastReadingEpoch + settings.config().intervalSeconds;
    }
    if (epoch != 0) lastReadingEpoch = epoch;
    const uint8_t flags = epoch == 0 ? fw::kFlagClockUnsynced : 0;
    pendingReading = measurement.sample(settings.bootId(), nextSequence, epoch, flags);
    pendingCommit = true;
}
