#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#include "config.h"

// ============================================================================
// Diagnostics — Watchdog, Health Counters (uint32_t), Boot Tracking
// ============================================================================

static unsigned long _bootTime = 0;
static unsigned long _lastHealthReport = 0;
static uint32_t _sendSuccessCount = 0;
static uint32_t _sendFailCount = 0;
static uint32_t _sendDropCount = 0;
static uint32_t _wifiReconnectCount = 0;
static uint32_t _ctErrorCount = 0;
static String _bootReasonStr = "Unknown";

void flushBeforeRestart();  // Defined in http_sender.h

void initDiagnostics() {
    _bootTime = millis();

    esp_task_wdt_init(WDT_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_POWERON:   _bootReasonStr = "Power-on"; break;
        case ESP_RST_SW:        _bootReasonStr = "Software reset"; break;
        case ESP_RST_PANIC:     _bootReasonStr = "Panic/crash"; break;
        case ESP_RST_INT_WDT:   _bootReasonStr = "Interrupt watchdog"; break;
        case ESP_RST_TASK_WDT:  _bootReasonStr = "Task watchdog"; break;
        case ESP_RST_WDT:       _bootReasonStr = "Other watchdog"; break;
        case ESP_RST_DEEPSLEEP: _bootReasonStr = "Deep sleep wake"; break;
        case ESP_RST_BROWNOUT:  _bootReasonStr = "Brownout"; break;
        default:                _bootReasonStr = "Unknown (" + String((int)reason) + ")"; break;
    }
    Serial.printf("[DIAG] Boot reason: %s | Heap: %u bytes\n", _bootReasonStr.c_str(), ESP.getFreeHeap());
}

void feedWatchdog() { esp_task_wdt_reset(); }

void recordSendSuccess()    { _sendSuccessCount++; }
void recordSendFailure()    { _sendFailCount++; }
void recordSendDrop()       { _sendDropCount++; }
void recordWiFiReconnect()  { _wifiReconnectCount++; }
void recordCTError()        { _ctErrorCount++; }

unsigned long getUptimeSeconds() { return (millis() - _bootTime) / 1000; }
String getBootReason()      { return _bootReasonStr; }

void diagnosticsLoop() {
    unsigned long now = millis();
    if (now - _lastHealthReport < HEALTH_REPORT_INTERVAL_MS) return;
    _lastHealthReport = now;

    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t minFreeHeap = ESP.getMinFreeHeap();
    unsigned long uptime = getUptimeSeconds();

    Serial.printf("[DIAG] Up:%lus | Heap:%u(min:%u) | S:%u F:%u D:%u | WiFi:%u | RSSI:%d\n",
        uptime, freeHeap, minFreeHeap,
        _sendSuccessCount, _sendFailCount, _sendDropCount,
        _wifiReconnectCount, WiFi.RSSI());

    if (freeHeap < 15000) {
        Serial.println("[DIAG] CRITICAL: Heap exhausted — rebooting to recover");
        flushBeforeRestart();
        delay(100);
        ESP.restart();
    } else if (freeHeap < LOW_HEAP_THRESHOLD) {
        Serial.printf("[DIAG] WARNING: Low heap %u — clearing buffers\n", freeHeap);
        // Force save and compact
    }
}

// FIX #5: snprintf instead of String concatenation (prevents heap fragmentation)
String getDiagnosticsJSON() {
    char buf[384];
    snprintf(buf, sizeof(buf),
        "{\"uptime_s\":%lu,\"free_heap\":%u,\"min_heap\":%u,\"wifi_rssi\":%d,"
        "\"sent\":%u,\"failed\":%u,\"dropped\":%u,\"wifi_reconnects\":%u,"
        "\"boot_reason\":\"%s\",\"firmware\":\"" FIRMWARE_VERSION "\"}",
        getUptimeSeconds(), ESP.getFreeHeap(), ESP.getMinFreeHeap(), WiFi.RSSI(),
        _sendSuccessCount, _sendFailCount, _sendDropCount, _wifiReconnectCount,
        _bootReasonStr.c_str());
    return String(buf);
}
