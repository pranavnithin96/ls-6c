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
// Safe-gate inputs for the scheduled reboot (defined later in the sketch)
void logError(const String& message);
int getQueueSize();
bool isUploadPending();
bool isOfflineMode();
bool isRejectedDrainPending();

static uint32_t _crashCount = 0;

// WDT breadcrumb: which network call was executing when the task watchdog
// fired. RTC_NOINIT survives every reset except full power loss, so the
// post-reboot boot can NAME the blocking site instead of us theorizing
// about it (the 2.10.1 inter-call feeds disproved the accumulation theory
// at Meton — a SINGLE call blocks >60s; this instrument identifies which).
// Written by the network task around each subsystem call; reported through
// logError -> device_error_log on the next boot after a task-WDT reset.
RTC_NOINIT_ATTR uint8_t _wdtCheckpoint;
RTC_NOINIT_ATTR uint8_t _wdtCheckpointValid;   // 0xA5 = checkpoint is real
#define WDT_CP_IDLE        0
#define WDT_CP_SENDQUEUE   1
#define WDT_CP_HEARTBEAT   2
#define WDT_CP_OTA         3
#define WDT_CP_BUFSAVE     4
#define WDT_CP_WIFI        5
// Sub-sites inside processSendQueue — "during: processSendQueue" (meton_01,
// 2026-07-25 08:10) left two suspects standing: the live POST's per-byte
// response timeout and the backlog upload's unbounded streaming write. These
// distinguish them.
#define WDT_CP_LIVEPOST    6
#define WDT_CP_OFFLINE_UP  7
#define WDT_CP_REJ_DRAIN   8
static inline void wdtCheckpoint(uint8_t cp) {
    _wdtCheckpoint = cp;
    _wdtCheckpointValid = 0xA5;
}
static const char* _wdtCheckpointName(uint8_t cp) {
    switch (cp) {
        case WDT_CP_SENDQUEUE:  return "processSendQueue";
        case WDT_CP_HEARTBEAT:  return "heartbeatLoop";
        case WDT_CP_OTA:        return "otaLoop";
        case WDT_CP_BUFSAVE:    return "periodicBufferSave";
        case WDT_CP_WIFI:       return "wifi check";
        case WDT_CP_LIVEPOST:   return "livePost";
        case WDT_CP_OFFLINE_UP: return "offline upload";
        case WDT_CP_REJ_DRAIN:  return "rejected drain";
        default:                return "idle/unknown";
    }
}

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

    // Track crash count in NVS — persists across reboots
    Preferences crashPrefs;
    crashPrefs.begin("diag", false);
    _crashCount = crashPrefs.getUInt("crashes", 0);
    if (reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT ||
        reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT || reason == ESP_RST_BROWNOUT) {
        _crashCount++;
        crashPrefs.putUInt("crashes", _crashCount);
    } else if (reason == ESP_RST_POWERON) {
        _crashCount = 0;  // Reset on clean power cycle
        crashPrefs.putUInt("crashes", 0);
    }
    crashPrefs.end();

    Serial.printf("[DIAG] Boot: %s | Crashes: %u | Heap: %u\n",
        _bootReasonStr.c_str(), _crashCount, ESP.getFreeHeap());

    // Report the WDT breadcrumb from the PREVIOUS boot, then clear it. Queued
    // via logError so it reaches device_error_log on the first heartbeat.
    if (reason == ESP_RST_TASK_WDT && _wdtCheckpointValid == 0xA5) {
        char cpMsg[48];
        snprintf(cpMsg, sizeof(cpMsg), "WDT during: %s",
                 _wdtCheckpointName(_wdtCheckpoint));
        logError(cpMsg);
    }
    _wdtCheckpoint = WDT_CP_IDLE;
    _wdtCheckpointValid = 0xA5;
}

void feedWatchdog() { esp_task_wdt_reset(); }

void recordSendSuccess()    { _sendSuccessCount++; }
void recordSendFailure()    { _sendFailCount++; }
void recordSendDrop()       { _sendDropCount++; }
void recordWiFiReconnect()  { _wifiReconnectCount++; }
void recordCTError()        { _ctErrorCount++; }

unsigned long getUptimeSeconds() { return (millis() - _bootTime) / 1000; }
String getBootReason()      { return _bootReasonStr; }
uint32_t getCrashCount()    { return _crashCount; }
uint32_t getWiFiReconnectCount() { return _wifiReconnectCount; }

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

// ============================================================================
// Scheduled maintenance reboot — planned beats unplanned (heap fragmentation,
// WiFi-stack rot). Costs ~5-20s of sampling per event: a deliberate,
// owner-approved zero-loss exception, taken only at a provably safe moment.
// ============================================================================
static uint32_t _rebootAfterS = 0;   // 0 = disabled

void initScheduledReboot() {
    Preferences p;
    p.begin("lscfg", true);
    int days = p.getInt("rebootdays", SCHEDULED_REBOOT_DAYS);
    p.end();
    if (days <= 0) { _rebootAfterS = 0; return; }
    // getUptimeSeconds() is millis()-based and wraps at 49.7 days — a threshold
    // beyond that would silently never fire. Clamp with margin.
    if (days > 45) days = 45;
    // Deterministic per-device jitter (±3h from the MAC) so the fleet never
    // gaps simultaneously — same reasoning as the WiFi reconnect jitter.
    int32_t jitterS = (int32_t)(ESP.getEfuseMac() % 21600ULL) - 10800;
    _rebootAfterS = (uint32_t)days * 86400UL + jitterS;
    Serial.printf("[DIAG] Scheduled reboot after %u s of uptime\n", _rebootAfterS);
}

void scheduledRebootLoop() {
    if (_rebootAfterS == 0 || getUptimeSeconds() < _rebootAfterS) return;
    static unsigned long lastGateCheck = 0;
    if (millis() - lastGateCheck < 60000) return;   // re-try the gate 1x/min
    lastGateCheck = millis();
    // Safe gate: never reboot with undelivered data or an unsynced clock.
    // If conditions never clear, the device just keeps running — reboot is
    // opportunistic maintenance, not a deadline.
    if (WiFi.status() != WL_CONNECTED) return;
    if (isOfflineMode() || isUploadPending() || getQueueSize() > 0) return;
    if (isRejectedDrainPending()) return;   // finish an operator-requested drain first
    struct tm t;
    if (!getLocalTime(&t, 0)) return;
    logError("scheduled maintenance reboot");
    Serial.println("[DIAG] Scheduled maintenance reboot — flushing and restarting");
    // Flush twice, 1.1s apart: Core 1 may queue one more reading between the
    // safe-gate check and the first flush's snapshot; the second flush catches
    // it (the next 1Hz reading lands within the delay). Shrinks per-reboot RAM
    // loss from ≤1 reading to ~zero.
    flushBeforeRestart();
    delay(1100);
    flushBeforeRestart();
    delay(200);
    ESP.restart();
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
