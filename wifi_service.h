#pragma once

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>

#include "settings_service.h"

namespace fw {

class WifiService {
public:
    void begin(SettingsService& settings);
    void loop();
    bool connected() const;
    bool provisioning() const { return provisioning_; }
    int rssi() const;
    int minimumRssi() const { return minimumRssi_; }
    int averageRssi() const;
    uint32_t reconnects() const { return reconnects_; }

private:
    void startStation();
    void startProvisioning();
    void handleRoot();
    void handleSave();
    void scanNetworks();
    String setupPin() const;

    SettingsService* settings_ = nullptr;
    WebServer server_{80};
    DNSServer dns_;
    bool provisioning_ = false;
    bool wasConnected_ = false;
    uint32_t lastReconnectMillis_ = 0;
    uint32_t reconnectDelayMs_ = kRetryMinMs;
    uint32_t reconnects_ = 0;
    int64_t rssiSum_ = 0;
    uint32_t rssiSamples_ = 0;
    uint32_t lastRssiSampleMillis_ = 0;
    int minimumRssi_ = 0;
    String scanOptions_;
};

}  // namespace fw
