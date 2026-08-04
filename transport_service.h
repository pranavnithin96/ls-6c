#pragma once

#include <Arduino.h>
#include <FS.h>
#include <HTTPClient.h>
#include <WiFiClient.h>

#include "clock_service.h"
#include "durable_queue.h"
#include "settings_service.h"

namespace fw {

class TransportService {
public:
    void begin(SettingsService& settings, ClockService& clock, DurableQueue& queue);
    void loop(bool connected);
    const String& heartbeatUrl() const { return heartbeatUrl_; }
    const String& firmwareBaseUrl() const { return firmwareBaseUrl_; }

private:
    int postLive(const ElectricalReading& reading);
    int postFile(const String& path, const char* format, bool anchorValid);
    int postPlainHttpFile(File& file, size_t length, const String& headers);
    bool sendAll(int socket, const uint8_t* data, size_t length, uint32_t startedMillis);
    size_t serializeReading(const ElectricalReading& reading, char* output, size_t capacity) const;
    String baseUrl(const String& liveUrl) const;

    SettingsService* settings_ = nullptr;
    ClockService* clock_ = nullptr;
    DurableQueue* queue_ = nullptr;
    String liveUrl_;
    String bulkUrl_;
    String heartbeatUrl_;
    String firmwareBaseUrl_;
    WiFiClient liveClient_;
    HTTPClient liveHttp_;
    uint32_t retryAtMillis_ = 0;
    uint32_t retryDelayMs_ = kRetryMinMs;
    uint32_t lastBulkAttemptMillis_ = 0;
};

}  // namespace fw
