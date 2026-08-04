#include "measurement_service.h"

#include <math.h>

namespace fw {

void MeasurementService::begin(const DeviceConfig& config) {
    config_ = &config;
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
}

float MeasurementService::countToAmps(uint8_t channel, float count) const {
    const float calibrated = config_->ctSlope[channel];
    if (calibrated > 0.0f) return calibrated * count;
    switch (config_->ctRating[channel]) {
        case 50: return 0.0421f * count;
        case 100: return 0.0327f * count;
        default: return 0.0123f * count + 0.13f;
    }
}

ElectricalReading MeasurementService::sample(uint32_t bootId, uint32_t sequence,
                                               uint32_t epoch, uint8_t flags) const {
    ElectricalReading reading{};
    reading.bootId = bootId;
    reading.sequence = sequence;
    reading.epoch = epoch;
    reading.sampleMillis = millis();
    reading.voltageDecivolts = config_->voltageDecivolts;
    reading.intervalSeconds = config_->intervalSeconds;
    reading.flags = flags;

    uint32_t sums[kChannelCount] = {0};
    const uint32_t startedMicros = micros();
    for (uint16_t sample = 0; sample < kSamplesPerWindow; ++sample) {
        for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
            sums[channel] += analogRead(kCtPins[channel]);
        }
        const uint32_t target = startedMicros +
            static_cast<uint32_t>(sample + 1) * kSamplePeriodUs;
        while (static_cast<int32_t>(micros() - target) < 0) {}
    }

    reading.sampleDurationMs = static_cast<uint16_t>((micros() - startedMicros) / 1000);
    for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
        const float average = static_cast<float>(sums[channel]) / kSamplesPerWindow;
        float amps = countToAmps(channel, average);
        if (!isfinite(amps) || amps < 0.0f) amps = 0.0f;
        amps = constrain(amps, 0.0f, 327.67f);
        reading.ampsCenti[channel] = static_cast<int16_t>(lroundf(amps * 100.0f));
    }
    return reading;
}

}  // namespace fw
