#pragma once

#include <Arduino.h>

namespace fw {

class ClockService {
public:
    void begin();
    void loop(bool networkConnected);
    bool synced() const;
    uint32_t epochAt(uint32_t sampleMillis) const;
    uint32_t nowEpoch() const;
    long syncAgeSeconds() const;
    String formatUtc(uint32_t epoch) const;

private:
    uint32_t lastKickMillis_ = 0;
    uint8_t quickRetries_ = 0;
};

}  // namespace fw
