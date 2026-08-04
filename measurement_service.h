#pragma once

#include <Arduino.h>
#include "app_types.h"

namespace fw {

class MeasurementService {
public:
    void begin(const DeviceConfig& config);
    ElectricalReading sample(uint32_t bootId, uint32_t sequence,
                             uint32_t epoch, uint8_t flags) const;

private:
    const DeviceConfig* config_ = nullptr;
    float countToAmps(uint8_t channel, float count) const;
};

}  // namespace fw
