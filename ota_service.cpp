#include "ota_service.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <esp_ota_ops.h>

#include "config.h"

namespace fw {

void OtaService::begin(const String& firmwareBaseUrl, const String& deviceId,
                       DurableQueue& queue, ControlFlags& controls) {
    baseUrl_ = firmwareBaseUrl;
    deviceId_ = deviceId;
    queue_ = &queue;
    controls_ = &controls;
    bootMillis_ = millis();
    const esp_partition_t* running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    pendingValidation_ = running && esp_ota_get_state_partition(running, &state) == ESP_OK &&
                         state == ESP_OTA_IMG_PENDING_VERIFY;
}

bool OtaService::newer(const String& candidate, const String& current) const {
    int a[3] = {0}, b[3] = {0};
    sscanf(candidate.c_str(), "%d.%d.%d", &a[0], &a[1], &a[2]);
    sscanf(current.c_str(), "%d.%d.%d", &b[0], &b[1], &b[2]);
    for (uint8_t i = 0; i < 3; ++i) {
        if (a[i] != b[i]) return a[i] > b[i];
    }
    return false;
}

void OtaService::validateRunningImage(bool systemHealthy) {
    if (!pendingValidation_ || !systemHealthy || millis() - bootMillis_ < 5 * 60 * 1000UL) return;
    if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
        pendingValidation_ = false;
        Serial.println("[OTA] Trial image stable for five minutes; marked valid");
    }
}

void OtaService::loop(bool connected, bool systemHealthy) {
    validateRunningImage(systemHealthy);
    if (!connected || updating_ || pendingValidation_) return;
    const uint32_t now = millis();
    const bool forced = controls_ && controls_->otaRequested;
    const bool scheduled = (lastCheckMillis_ == 0 && now >= kOtaFirstCheckMs) ||
                           (lastCheckMillis_ != 0 && now - lastCheckMillis_ >= kOtaCheckIntervalMs);
    if (!forced && !scheduled) return;
    if (controls_) controls_->otaRequested = false;
    lastCheckMillis_ = now;
    checkAndInstall();
}

bool OtaService::checkAndInstall() {
    HTTPClient check;
    check.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    check.setTimeout(10000);
    const String url = baseUrl_ + "check?device_id=" + deviceId_ +
                       "&current_version=" + String(kFirmwareVersion);
    if (!check.begin(url)) return false;
    const int code = check.GET();
    if (code != 200) {
        check.end();
        return false;
    }
    const String response = check.getString();
    check.end();

    JsonDocument document;
    if (deserializeJson(document, response) || !(document["update_available"] | false)) return false;
    const String version = document["version"] | "";
    const String downloadUrl = document["url"] | "";
    const String md5 = document["md5"] | "";
    const size_t declaredSize = document["size"] | 0;
    if (!newer(version, kFirmwareVersion) || downloadUrl.isEmpty() || md5.length() != 32) {
        Serial.println("[OTA] Refused update without a newer version, URL, and MD5");
        return false;
    }

    updating_ = true;
    HTTPClient download;
    download.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    download.setTimeout(60000);
    if (!download.begin(downloadUrl)) {
        updating_ = false;
        return false;
    }
    const int downloadCode = download.GET();
    const int contentLength = download.getSize();
    if (downloadCode != 200 || contentLength <= 0 ||
        (declaredSize != 0 && declaredSize != static_cast<size_t>(contentLength)) ||
        static_cast<size_t>(contentLength) > ESP.getFreeSketchSpace()) {
        download.end();
        updating_ = false;
        return false;
    }
    if (!Update.begin(contentLength)) {
        download.end();
        updating_ = false;
        return false;
    }
    if (!Update.setMD5(md5.c_str())) {
        Update.abort();
        download.end();
        updating_ = false;
        return false;
    }
    const size_t written = Update.writeStream(*download.getStreamPtr());
    const bool complete = written == static_cast<size_t>(contentLength) && Update.end() &&
                          Update.isFinished();
    download.end();
    updating_ = false;
    if (!complete) {
        Update.abort();
        return false;
    }

    // WAL records are already durable. Checkpointing here reduces replay volume;
    // a failed checkpoint is not permission to erase or postpone the update image.
    queue_->checkpoint();
    Serial.printf("[OTA] Installed %s; rebooting into a five-minute trial\n", version.c_str());
    delay(250);
    ESP.restart();
    return true;
}

}  // namespace fw
