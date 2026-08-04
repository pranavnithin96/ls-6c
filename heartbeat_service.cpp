#include "heartbeat_service.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_system.h>

#include "config.h"

namespace fw {

void HeartbeatService::begin(const String& url, SettingsService& settings,
                             WifiService& wifi, ClockService& clock,
                             DurableQueue& queue, ControlFlags& controls) {
    url_ = url;
    settings_ = &settings;
    wifi_ = &wifi;
    clock_ = &clock;
    queue_ = &queue;
    controls_ = &controls;
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: bootReason_ = "power_on"; break;
        case ESP_RST_SW: bootReason_ = "software"; break;
        case ESP_RST_PANIC: bootReason_ = "panic"; break;
        case ESP_RST_INT_WDT: bootReason_ = "int_watchdog"; break;
        case ESP_RST_TASK_WDT: bootReason_ = "task_watchdog"; break;
        case ESP_RST_WDT: bootReason_ = "watchdog"; break;
        case ESP_RST_BROWNOUT: bootReason_ = "brownout"; break;
        default: bootReason_ = "unknown"; break;
    }
}

const char* HeartbeatService::storageStateName(StorageState state) const {
    switch (state) {
        case StorageState::Ready: return "ready";
        case StorageState::CapacityBlocked: return "capacity_blocked";
        case StorageState::Fault: return "fault";
        default: return "uninitialized";
    }
}

void HeartbeatService::loop(bool connected) {
    if (!connected || millis() - lastHeartbeatMillis_ < kHeartbeatIntervalMs) return;
    lastHeartbeatMillis_ = millis();
    send();
}

void HeartbeatService::send() {
    JsonDocument document;
    const DeliveryStats stats = queue_->stats();
    document["device_id"] = settings_->config().deviceId;
    document["firmware_version"] = kFirmwareVersion;
    document["build"] = __DATE__ " " __TIME__;
    document["boot_id"] = settings_->bootId();
    document["boot_reason"] = bootReason_;
    document["uptime_seconds"] = millis() / 1000;
    document["wifi_rssi"] = wifi_->rssi();
    document["wifi_rssi_min"] = wifi_->minimumRssi();
    document["wifi_rssi_avg"] = wifi_->averageRssi();
    document["wifi_reconnects"] = wifi_->reconnects();
    document["wifi_ssid"] = WiFi.SSID();
    document["ip_address"] = WiFi.localIP().toString();
    document["free_heap"] = ESP.getFreeHeap();
    document["min_heap"] = ESP.getMinFreeHeap();
    document["heap_largest_block"] =
        static_cast<uint32_t>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    document["clock_synced"] = clock_->synced();
    document["device_epoch"] = clock_->nowEpoch();
    document["ntp_sync_age_s"] = clock_->syncAgeSeconds();
    document["storage_state"] = storageStateName(queue_->state());
    document["wal_depth"] = queue_->walDepth();
    document["wal_bytes"] = queue_->walBytes();
    document["offline_backlog_bytes"] = queue_->offlineBytes();
    document["rejected_log_bytes"] = queue_->legacyJsonBytes();
    document["quarantine_bytes"] = queue_->quarantineBytes();
    document["filesystem_free_bytes"] = queue_->freeBytes();
    document["readings_generated"] = stats.generated;
    document["readings_durable"] = stats.durable;
    document["live_acked"] = stats.liveAcknowledged;
    document["bulk_files_acked"] = stats.bulkAcknowledged;
    document["delivery_retries"] = stats.retries;
    document["local_write_failures"] = stats.localWriteFailures;
    document["permanent_losses"] = stats.permanentLosses;
    document["last_http_code"] = stats.lastHttpCode;
    document["last_success_age_s"] = stats.lastSuccessMillis == 0
        ? -1 : static_cast<long>((millis() - stats.lastSuccessMillis) / 1000);

    String body;
    serializeJson(document, body);
    HTTPClient http;
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.setTimeout(kHttpTimeoutMs);
    if (!http.begin(url_)) return;
    http.addHeader("Content-Type", "application/json");
    const int code = http.POST(body);
    if (code == 200) processCommands(http.getString());
    http.end();
}

void HeartbeatService::processCommands(const String& response) {
    if (response.isEmpty() || millis() - lastCommandMillis_ < 30000UL) return;
    JsonDocument document;
    if (deserializeJson(document, response)) return;
    JsonArray commands = document["commands"];
    if (commands.isNull()) return;
    lastCommandMillis_ = millis();
    uint8_t processed = 0;
    for (JsonObject command : commands) {
        if (++processed > 10) break;
        const String action = command["action"] | "";
        if (action == "set_interval") {
            settings_->setInterval(command["value"] | 0);
        } else if (action == "set_voltage") {
            settings_->setVoltage(command["value"] | 0.0f);
        } else if (action == "set_ct_slope") {
            const int channel = command["channel"] | 0;
            settings_->setChannelSlope(channel - 1, command["value"] | -1.0f);
        } else if (action == "update_firmware") {
            controls_->otaRequested = true;
        } else if (action == "reboot") {
            controls_->rebootRequested = true;
        }
        // Destructive legacy commands (clear_rejected/factory_reset) are
        // intentionally unsupported. Remote control can never erase telemetry.
    }
}

}  // namespace fw
