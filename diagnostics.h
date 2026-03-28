#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

#define WDT_TIMEOUT_S 120
#define HEALTH_REPORT_INTERVAL_MS 60000
#define LOW_HEAP_THRESHOLD 20480

static unsigned long _bootTime = 0;
static unsigned long _lastHealthReport = 0;
static unsigned long _lastLoopTime = 0;
static unsigned long _maxLoopTime = 0;
static int _sendSuccessCount = 0;
static int _sendFailCount = 0;
static int _sendDropCount = 0;
static int _wifiReconnectCount = 0;
static int _ctErrorCount = 0;

void initDiagnostics() {
    _bootTime = millis();

    // Initialize watchdog (ESP32 Arduino v2.x API)
    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    // Log boot reason
    esp_reset_reason_t reason = esp_reset_reason();
    const char* reasonStr;
    switch (reason) {
        case ESP_RST_POWERON: reasonStr = "Power-on"; break;
        case ESP_RST_SW: reasonStr = "Software reset"; break;
        case ESP_RST_PANIC: reasonStr = "Panic/crash"; break;
        case ESP_RST_INT_WDT: reasonStr = "Interrupt watchdog"; break;
        case ESP_RST_TASK_WDT: reasonStr = "Task watchdog"; break;
        case ESP_RST_WDT: reasonStr = "Other watchdog"; break;
        case ESP_RST_DEEPSLEEP: reasonStr = "Deep sleep wake"; break;
        default: reasonStr = "Unknown"; break;
    }
    Serial.printf("[DIAG] Boot reason: %s, Free heap: %d bytes\n", reasonStr, ESP.getFreeHeap());
}

void feedWatchdog() {
    esp_task_wdt_reset();
}

void recordLoopTime(unsigned long ms) {
    _lastLoopTime = ms;
    if (ms > _maxLoopTime) _maxLoopTime = ms;
}

void recordSendSuccess() { _sendSuccessCount++; }
void recordSendFailure() { _sendFailCount++; }
void recordSendDrop() { _sendDropCount++; }
void recordWiFiReconnect() { _wifiReconnectCount++; }
void recordCTError() { _ctErrorCount++; }

unsigned long getUptimeSeconds() {
    return (millis() - _bootTime) / 1000;
}

void diagnosticsLoop() {
    unsigned long now = millis();
    if (now - _lastHealthReport < HEALTH_REPORT_INTERVAL_MS) return;
    _lastHealthReport = now;

    unsigned long uptime = getUptimeSeconds();
    uint32_t freeHeap = ESP.getFreeHeap();
    int rssi = WiFi.RSSI();

    Serial.println("=== Health Report ===");
    Serial.printf("  Uptime: %luh %lum %lus\n", uptime / 3600, (uptime % 3600) / 60, uptime % 60);
    Serial.printf("  Free heap: %u bytes%s\n", freeHeap, freeHeap < LOW_HEAP_THRESHOLD ? " (LOW!)" : "");
    Serial.printf("  WiFi RSSI: %d dBm, Reconnects: %d\n", rssi, _wifiReconnectCount);
    Serial.printf("  Sent: %d, Failed: %d, Dropped: %d\n", _sendSuccessCount, _sendFailCount, _sendDropCount);
    Serial.printf("  Last loop: %lums, Max loop: %lums\n", _lastLoopTime, _maxLoopTime);
    Serial.printf("  CT errors: %d\n", _ctErrorCount);
    Serial.println("=====================");

    if (freeHeap < LOW_HEAP_THRESHOLD) {
        Serial.println("[DIAG] WARNING: Low memory!");
    }
}

String getDiagnosticsJSON() {
    String json = "{";
    json += "\"uptime_s\":" + String(getUptimeSeconds());
    json += ",\"free_heap\":" + String(ESP.getFreeHeap());
    json += ",\"wifi_rssi\":" + String(WiFi.RSSI());
    json += ",\"wifi_reconnects\":" + String(_wifiReconnectCount);
    json += ",\"sent\":" + String(_sendSuccessCount);
    json += ",\"failed\":" + String(_sendFailCount);
    json += ",\"dropped\":" + String(_sendDropCount);
    json += ",\"last_loop_ms\":" + String(_lastLoopTime);
    json += ",\"max_loop_ms\":" + String(_maxLoopTime);
    json += ",\"firmware\":\"" FIRMWARE_VERSION "\"";
    json += "}";
    return json;
}
