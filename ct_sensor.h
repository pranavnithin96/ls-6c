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

// Calibration state — protected by spinlock for cross-core access
static portMUX_TYPE _calMux = portMUX_INITIALIZER_UNLOCKED;
static float _zeroMv[NUM_CT_CHANNELS] = {0};
static bool _calibrated = false;
static CalPoint _calPoints[NUM_CT_CHANNELS][CAL_POINTS];
static bool _multiCalLoaded = false;

// Forward declaration
void feedWatchdog();

static float mvToAmps(int ch, float mv) {
    if (!_multiCalLoaded) return mv * DEFAULT_AMPS_PER_MV;

    CalPoint* p = _calPoints[ch];

    // Validate points are usable
    if (p[0].mv == 0 && p[0].amps == 0 && p[1].mv == 0) {
        return mv * DEFAULT_AMPS_PER_MV;
    }

    if (mv <= p[0].mv) {
        float denom = p[1].mv - p[0].mv;
        if (fabsf(denom) < 0.001f) return p[0].amps;
        return p[0].amps + (p[1].amps - p[0].amps) / denom * (mv - p[0].mv);
    }
    for (int i = 0; i < CAL_POINTS - 1; i++) {
        if (mv <= p[i+1].mv) {
            float denom = p[i+1].mv - p[i].mv;
            if (fabsf(denom) < 0.001f) return p[i].amps;
            float t = (mv - p[i].mv) / denom;
            return p[i].amps + t * (p[i+1].amps - p[i].amps);
        }
    }
    int l = CAL_POINTS - 1;
    float denom = p[l].mv - p[l-1].mv;
    if (fabsf(denom) < 0.001f) return p[l].amps;
    return p[l].amps + (p[l].amps - p[l-1].amps) / denom * (mv - p[l].mv);
}

void initCTSensors() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    Preferences calPrefs;
    calPrefs.begin("ctcal", true);
    _calibrated = calPrefs.getBool("done", false);
    if (_calibrated) {
        for (int i = 0; i < NUM_CT_CHANNELS; i++) {
            char key[8]; snprintf(key, sizeof(key), "z%d", i);
            _zeroMv[i] = calPrefs.getFloat(key, 0.0f);
        }
    }
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
    Serial.printf("[CT] Cal: %s | MultiCal: %s | Gain: %.5f A/mV\n",
        _calibrated ? "yes" : "auto-zero pending", _multiCalLoaded ? "yes" : "no", DEFAULT_AMPS_PER_MV);
}

void calibrateCTZero() {
    Serial.println("[CT] Zero calibration...");
    float newZero[NUM_CT_CHANNELS];

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        uint32_t sum = 0;
        unsigned long t0 = micros();
        for (int i = 0; i < 300; i++) {
            sum += analogReadMilliVolts(CT_PINS[ch]);
            while (micros() < t0 + (unsigned long)((i+1) * 1000)) {}
            if (i % 50 == 0) feedWatchdog();  // Keep WDT happy during long cal
        }
        newZero[ch] = (float)sum / 300.0f;

        // Sanity check: zero offset should be 0-2500 mV
        if (newZero[ch] < 0 || newZero[ch] > 2500) {
            Serial.printf("[CT] CH%d: BAD zero %.1f mV — keeping old value\n", ch + 1, newZero[ch]);
            newZero[ch] = _zeroMv[ch];
        } else {
            Serial.printf("[CT] CH%d: %.1f mV\n", ch + 1, newZero[ch]);
        }
    }

    // Atomic update with lock
    portENTER_CRITICAL(&_calMux);
    for (int i = 0; i < NUM_CT_CHANNELS; i++) _zeroMv[i] = newZero[i];
    _calibrated = true;
    portEXIT_CRITICAL(&_calMux);

    // Persist to NVS
    Preferences calPrefs;
    calPrefs.begin("ctcal", false);
    calPrefs.putBool("done", true);
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        char key[8]; snprintf(key, sizeof(key), "z%d", i);
        calPrefs.putFloat(key, newZero[i]);
    }
    calPrefs.end();
    Serial.println("[CT] Zero calibration saved to NVS");
}

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
    float mv = (float)sum / 500.0f - _zeroMv[ch];
    if (mv < 0) mv = 0;

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
    Serial.printf("[CT] CH%d pt%d: %.1fmV = %.3fA\n", ch+1, pt, mv, knownAmps);
}

// Read all CT channels — timestamps captured at sample START
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();  // Timestamp at START of sampling
    all.total_watts = 0;

    // Auto-zero on first boot (runs once, ~1.8s)
    if (!_calibrated) {
        calibrateCTZero();
    }

    uint32_t sumMv[NUM_CT_CHANNELS] = {0};
    unsigned long t0 = micros();

    // Interleaved sampling: all 6 channels per time step
    for (int s = 0; s < ADC_SAMPLES_PER_CH; s++) {
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            sumMv[ch] += analogReadMilliVolts(CT_PINS[ch]);
        }
        unsigned long target = t0 + (unsigned long)((s + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}  // Busy-wait for precise timing
    }

    all.sample_duration_ms = (micros() - t0) / 1000;

    // Take a snapshot of calibration data under lock
    float zeroSnap[NUM_CT_CHANNELS];
    portENTER_CRITICAL(&_calMux);
    for (int i = 0; i < NUM_CT_CHANNELS; i++) zeroSnap[i] = _zeroMv[i];
    portEXIT_CRITICAL(&_calMux);

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples = ADC_SAMPLES_PER_CH;

        float avgMv = (float)sumMv[ch] / ADC_SAMPLES_PER_CH;
        float corrected = avgMv - zeroSnap[ch];
        if (corrected < 0) corrected = 0;

        float amps = mvToAmps(ch, corrected);
        // Guard against NaN/Inf
        if (isnan(amps) || isinf(amps)) amps = 0.0f;

        float power = grid_voltage * amps * DEFAULT_PF;
        r.avg_mv = (int)corrected;

        if (power < NOISE_THRESHOLD_W) {
            r.amps = 0; r.watts = 0; r.pf = 0;
        } else {
            r.amps = amps; r.watts = power; r.pf = DEFAULT_PF;
        }

        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    return all;
}

bool isCTCalibrated() { return _calibrated; }
bool isMultiCalLoaded() { return _multiCalLoaded; }
