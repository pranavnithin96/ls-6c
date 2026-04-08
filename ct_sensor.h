#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// ============================================================================
// CT Sensor — Fixed calibration math, thread-safe, proper timing
// ============================================================================

struct CalPoint { float mv; float amps; };

struct CTReading {
    float amps;
    float watts;
    float pf;
    float voltage;
    int avg_mv;
    int samples;
};

struct AllCTReadings {
    CTReading ct[NUM_CT_CHANNELS];
    float total_watts;
    unsigned long timestamp_ms;      // millis() at sample START
    unsigned long sample_duration_ms;
};

// Multi-point calibration kept as future option — NOT used by default path.
// Default path uses manufacturer's empirical formula: amps = 0.0123 * count + 0.13
static portMUX_TYPE _calMux = portMUX_INITIALIZER_UNLOCKED;
static CalPoint _calPoints[NUM_CT_CHANNELS][CAL_POINTS];
static bool _multiCalLoaded = false;

// Forward declaration
void feedWatchdog();

void initCTSensors() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Multi-cal load is kept for future use — default path doesn't touch _calPoints
    Preferences calPrefs;
    calPrefs.begin("ctcal", true);
    _multiCalLoaded = calPrefs.getBool("mcal", false);
    if (_multiCalLoaded) {
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            for (int p = 0; p < CAL_POINTS; p++) {
                char km[12], ka[12];
                snprintf(km, sizeof(km), "c%dp%dmv", ch, p);
                snprintf(ka, sizeof(ka), "c%dp%da", ch, p);
                _calPoints[ch][p].mv = calPrefs.getFloat(km, 0.0f);
                _calPoints[ch][p].amps = calPrefs.getFloat(ka, 0.0f);
            }
        }
    }
    calPrefs.end();

    Serial.printf("[CT] %d channels | %d samples | %dms window\n",
        NUM_CT_CHANNELS, ADC_SAMPLES_PER_CH, (ADC_SAMPLES_PER_CH * SAMPLE_INTERVAL_US) / 1000);
    Serial.println("[CT] Manufacturer formula: amps = 0.0123 * count + 0.13");
}

// Multi-point calibration recording — kept as future option, not used by default readAllCT path.
// Records raw mV (no zero subtraction). To re-enable in the data path, restore mvToAmps()
// and call it from readAllCT instead of the manufacturer formula.
void setCalPoint(int ch, int pt, float knownAmps) {
    if (ch < 0 || ch >= NUM_CT_CHANNELS || pt < 0 || pt >= CAL_POINTS) {
        Serial.println("[CT] Invalid channel/point");
        return;
    }

    uint32_t sum = 0;
    unsigned long t0 = micros();
    for (int i = 0; i < 500; i++) {
        sum += analogReadMilliVolts(CT_PINS[ch]);
        while (micros() < t0 + (unsigned long)((i+1) * 1000)) {}
        if (i % 100 == 0) feedWatchdog();
    }
    float mv = (float)sum / 500.0f;

    portENTER_CRITICAL(&_calMux);
    _calPoints[ch][pt] = {mv, knownAmps};
    _multiCalLoaded = true;
    portEXIT_CRITICAL(&_calMux);

    Preferences calPrefs;
    calPrefs.begin("ctcal", false);
    calPrefs.putBool("mcal", true);
    char km[12], ka[12];
    snprintf(km, sizeof(km), "c%dp%dmv", ch, pt);
    snprintf(ka, sizeof(ka), "c%dp%da", ch, pt);
    calPrefs.putFloat(km, mv);
    calPrefs.putFloat(ka, knownAmps);
    calPrefs.end();
    Serial.printf("[CT] CH%d pt%d: %.1fmV = %.3fA (recorded, not active)\n", ch+1, pt, mv, knownAmps);
}

// Read all CT channels — manufacturer's exact formula on raw ADC counts.
// amps = 0.0123 * count + 0.13  (empirical fit calibrated against this hardware)
// No auto-zero subtraction, no noise threshold — the +0.13 baseline is the
// correct floor for this diode-rectifier topology.
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();  // Timestamp at START of sampling
    all.total_watts = 0;

    uint32_t sumCounts[NUM_CT_CHANNELS] = {0};
    unsigned long t0 = micros();

    // Interleaved sampling: all 6 channels per 1ms time step.
    // Raw analogRead() counts — NOT analogReadMilliVolts(). The manufacturer's
    // 0.0123 coefficient was empirically fit against raw counts; switching to mV
    // breaks the fit because the count→mV relationship is not constant.
    for (int s = 0; s < ADC_SAMPLES_PER_CH; s++) {
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            sumCounts[ch] += analogRead(CT_PINS[ch]);
        }
        unsigned long target = t0 + (unsigned long)((s + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}  // Busy-wait for precise 1ms cadence
    }

    all.sample_duration_ms = (micros() - t0) / 1000;

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples = ADC_SAMPLES_PER_CH;

        float avgCount = (float)sumCounts[ch] / ADC_SAMPLES_PER_CH;

        // Manufacturer's exact formula — character for character
        float amps = (0.0123f * avgCount) + 0.13f;

        // Guard against NaN/Inf only
        if (isnan(amps) || isinf(amps)) amps = 0.0f;

        float power = grid_voltage * amps * DEFAULT_PF;
        r.avg_mv = (int)avgCount;  // Field name kept for compat — now stores avg count
        r.amps = amps;
        r.watts = power;
        r.pf = DEFAULT_PF;

        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    return all;
}

bool isCTCalibrated() { return true; }  // Manufacturer formula needs no calibration
bool isMultiCalLoaded() { return _multiCalLoaded; }
