#pragma once

#include <Arduino.h>

#include "app_types.h"
#include "clock_service.h"
#include "durable_queue.h"
#include "settings_service.h"
#include "wifi_service.h"

namespace fw {

class HeartbeatService {
public:
    void begin(const String& url, SettingsService& settings, WifiService& wifi,
               ClockService& clock, DurableQueue& queue, ControlFlags& controls);
    void loop(bool connected);

private:
    void send();
    void processCommands(const String& response);
    const char* storageStateName(StorageState state) const;

    String url_;
    String bootReason_;
    SettingsService* settings_ = nullptr;
    WifiService* wifi_ = nullptr;
    ClockService* clock_ = nullptr;
    DurableQueue* queue_ = nullptr;
    ControlFlags* controls_ = nullptr;
    uint32_t lastHeartbeatMillis_ = 0;
    uint32_t lastCommandMillis_ = 0;
};

}  // namespace fw
