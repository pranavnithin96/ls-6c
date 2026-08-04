#pragma once

#include <Arduino.h>
#include "app_types.h"

namespace fw {

class SettingsService {
public:
    bool begin();
    const DeviceConfig& config() const { return config_; }
    bool isProvisioned() const { return provisioned_; }
    uint32_t bootId() const { return bootId_; }

    bool saveProvisioning(const String& ssid, const String& password,
                          const String& deviceId, const String& location,
                          const String& serverUrl, float voltage,
                          const uint16_t ratings[kChannelCount],
                          int intervalSeconds, const String& timezone);
    bool setInterval(int seconds);
    bool setVoltage(float volts);
    bool setChannelSlope(uint8_t channel, float ampsPerCount);

private:
    DeviceConfig config_;
    bool provisioned_ = false;
    uint32_t bootId_ = 0;
};

}  // namespace fw
