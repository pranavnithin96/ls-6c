#pragma once
#include <Arduino.h>
#include <Preferences.h>

// CT pin mapping (LS-6C-IOT_V1.0 schematic — all ADC1)
#define NUM_CT_CHANNELS 6

// Sampling: interleaved across all 6 channels for ~800ms
// At each time step, read all 6 channels (~700μs), then wait for next ms
// ~800 samples per channel across 40 full 50Hz cycles
#define ADC_SAMPLES_PER_CH 800
#define SAMPLE_INTERVAL_US 1000  // 1ms between sample sets

// Calibration: derived from manufacturer's formula
// Original: amps = 0.0123 * rawADC + 0.13  (raw 12-bit counts)
// analogReadMilliVolts: 1 count ~ 0.806mV at 11dB atten
// amps_per_mV = 0.0123 / 0.806 = 0.01526
#define CALIBRATION_AMPS_PER_MV 0.01526f

// Power factor (assumed — no phase measurement possible with rectified DC)
#define DEFAULT_PF 0.9f

// Noise threshold in watts (matches piimage_3)
#define NOISE_THRESHOLD_W 5.0f

static const uint8_t CT_PINS[NUM_CT_CHANNELS] = {
    36, 39, 34, 35, 32, 33
};

struct CTReading {
    float amps;          // RMS current
    float watts;         // real power estimate (V * A * PF)
    float pf;            // power factor (assumed)
    float voltage;
    int avg_mv;          // raw average millivolts
    int variation;       // max - min (signal spread)
    int samples;
    bool valid;
};

struct AllCTReadings {
    CTReading ct[NUM_CT_CHANNELS];
    float total_watts;
    unsigned long timestamp_ms;
    unsigned long sample_duration_ms;
};

// Per-channel zero calibration (millivolts at no load)
static float _zeroMv[NUM_CT_CHANNELS] = {0};
static bool _calibrated = false;

void initCTSensors() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    Preferences calPrefs;
    calPrefs.begin("ctcal", true);
    _calibrated = calPrefs.getBool("done", false);
    if (_calibrated) {
        for (int i = 0; i < NUM_CT_CHANNELS; i++) {
            char key[8];
            snprintf(key, sizeof(key), "z%d", i);
            _zeroMv[i] = calPrefs.getFloat(key, 0.0f);
        }
        Serial.println("[CT] Loaded saved zero calibration");
    }
    calPrefs.end();

    Serial.printf("[CT] 6-channel sensors initialized\n");
    Serial.printf("[CT] Interleaved sampling: %d samples/ch over %dms\n",
        ADC_SAMPLES_PER_CH, (ADC_SAMPLES_PER_CH * SAMPLE_INTERVAL_US) / 1000);
    Serial.printf("[CT] Scale: %.5f A/mV, PF: %.1f, noise: %.0fW\n",
        CALIBRATION_AMPS_PER_MV, DEFAULT_PF, NOISE_THRESHOLD_W);

    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        Serial.printf("[CT] CH%d zero: %.1f mV\n", i + 1, _zeroMv[i]);
    }

    if (!_calibrated) {
        Serial.println("[CT] No calibration found — will auto-zero on first read");
    }
}

void calibrateCTZero() {
    Serial.println("[CT] === Zero Calibration ===");

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        uint32_t sum = 0;
        int count = 300;

        unsigned long t0 = micros();
        for (int i = 0; i < count; i++) {
            sum += analogReadMilliVolts(CT_PINS[ch]);
            unsigned long target = t0 + (unsigned long)((i + 1) * 1000);
            while (micros() < target) {}
        }

        _zeroMv[ch] = (float)sum / count;
        Serial.printf("[CT] CH%d zero: %.1f mV\n", ch + 1, _zeroMv[ch]);
    }

    Preferences calPrefs;
    calPrefs.begin("ctcal", false);
    calPrefs.putBool("done", true);
    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        char key[8];
        snprintf(key, sizeof(key), "z%d", i);
        calPrefs.putFloat(key, _zeroMv[i]);
    }
    calPrefs.end();

    _calibrated = true;
    Serial.println("[CT] Calibration saved to NVS");
    Serial.println("[CT] ========================");
}

// Read all 6 channels simultaneously — interleaved sampling for full coverage
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();
    all.total_watts = 0;

    if (!_calibrated) {
        calibrateCTZero();
    }

    // Per-channel accumulators
    double sumValues[NUM_CT_CHANNELS] = {0};
    double sumSquares[NUM_CT_CHANNELS] = {0};
    int16_t minVal[NUM_CT_CHANNELS];
    int16_t maxVal[NUM_CT_CHANNELS];
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        minVal[ch] = 32767;
        maxVal[ch] = -32768;
    }

    unsigned long t0 = micros();

    // Interleaved: at each time step, read all 6 channels (~700μs)
    // Then wait until the next 1ms boundary
    for (int s = 0; s < ADC_SAMPLES_PER_CH; s++) {
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            int16_t mv = (int16_t)analogReadMilliVolts(CT_PINS[ch]);
            double v = (double)mv;
            sumValues[ch] += v;
            sumSquares[ch] += v * v;
            if (mv < minVal[ch]) minVal[ch] = mv;
            if (mv > maxVal[ch]) maxVal[ch] = mv;
        }
        // Wait for next sample point
        unsigned long target = t0 + (unsigned long)((s + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}
    }

    all.sample_duration_ms = (micros() - t0) / 1000;

    // Process each channel
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples = ADC_SAMPLES_PER_CH;

        double avgRaw = sumValues[ch] / ADC_SAMPLES_PER_CH;
        double meanSquare = sumSquares[ch] / ADC_SAMPLES_PER_CH;
        double variance = meanSquare - (avgRaw * avgRaw);
        if (variance < 0) variance = 0;
        double rmsMv = sqrt(variance);

        // DC average above zero baseline
        float dcMv = (float)avgRaw - _zeroMv[ch];
        if (dcMv < 0) dcMv = 0;

        // Use whichever gives larger reading
        float effectiveMv = max(dcMv, (float)rmsMv);

        float amps = effectiveMv * CALIBRATION_AMPS_PER_MV;
        float power = grid_voltage * amps * DEFAULT_PF;

        r.variation = maxVal[ch] - minVal[ch];
        r.avg_mv = (int)dcMv;

        if (power < NOISE_THRESHOLD_W) {
            r.amps = 0.0f;
            r.watts = 0.0f;
            r.pf = 0.0f;
        } else {
            r.amps = amps;
            r.watts = power;
            r.pf = DEFAULT_PF;
        }

        r.valid = true;
        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    return all;
}

bool isCTCalibrated() { return _calibrated; }
