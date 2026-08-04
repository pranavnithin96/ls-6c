#pragma once

#include <Arduino.h>
#include "config.h"

namespace fw {

struct DeviceConfig {
    String wifiSsid;
    String wifiPassword;
    String deviceId;
    String location;
    String serverUrl;
    String timezone;
    uint16_t voltageDecivolts = kDefaultVoltageDecivolts;
    uint8_t intervalSeconds = kDefaultIntervalSeconds;
    uint16_t ctRating[kChannelCount] = {50, 50, 50, 50, 50, 50};
    float ctSlope[kChannelCount] = {0};
};

struct ElectricalReading {
    uint32_t bootId = 0;
    uint32_t sequence = 0;
    uint32_t epoch = 0;
    uint32_t sampleMillis = 0;
    uint16_t voltageDecivolts = kDefaultVoltageDecivolts;
    uint16_t sampleDurationMs = 0;
    uint8_t intervalSeconds = kDefaultIntervalSeconds;
    uint8_t flags = 0;
    int16_t ampsCenti[kChannelCount] = {0};
};

inline bool sameReading(const ElectricalReading& a, const ElectricalReading& b) {
    return a.bootId == b.bootId && a.sequence == b.sequence;
}

struct DeliveryStats {
    uint32_t generated = 0;
    uint32_t durable = 0;
    uint32_t liveAcknowledged = 0;
    uint32_t bulkAcknowledged = 0;
    uint32_t retries = 0;
    uint32_t localWriteFailures = 0;
    uint32_t permanentLosses = 0;
    int lastHttpCode = 0;
    uint32_t lastSuccessMillis = 0;
};

struct ControlFlags {
    volatile bool otaRequested = false;
    volatile bool rebootRequested = false;
};

}  // namespace fw
