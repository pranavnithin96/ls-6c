#pragma once
#include <Arduino.h>
#include <Preferences.h>

// CT pin mapping (LS-6C-IOT_V1.0 schematic — all ADC1)
#define NUM_CT_CHANNELS 6

// Sampling: 200 samples over 200ms = 10 complete 50Hz cycles
#define ADC_SAMPLES 200
#define SAMPLE_INTERVAL_US 1000  // 1ms between samples

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

    // Load saved calibration from NVS (separate namespace — survives factory reset)
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
    Serial.printf("[CT] ADC: analogReadMilliVolts, %d samples / %dms per channel\n",
        ADC_SAMPLES, (ADC_SAMPLES * SAMPLE_INTERVAL_US) / 1000);
    Serial.printf("[CT] Scale: %.5f A/mV, PF: %.1f, noise: %.0fW\n",
        CALIBRATION_AMPS_PER_MV, DEFAULT_PF, NOISE_THRESHOLD_W);

    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        Serial.printf("[CT] CH%d zero: %.1f mV\n", i + 1, _zeroMv[i]);
    }

    if (!_calibrated) {
        Serial.println("[CT] No calibration found — will auto-zero on first read");
    }
}

// Auto-zero: measure baseline per channel with no assumed load
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

    // Save to NVS
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

// Read a single CT channel using RMS calculation (variance method from piimage_3)
// RMS = sqrt(mean(x^2) - mean(x)^2) — correct for AC signals with DC offset
static CTReading readCTChannel(uint8_t pin, int channel, float grid_voltage) {
    CTReading r = {};
    r.voltage = grid_voltage;
    r.samples = ADC_SAMPLES;

    // Collect all samples
    int16_t rawSamples[ADC_SAMPLES];
    unsigned long t0 = micros();
    for (int i = 0; i < ADC_SAMPLES; i++) {
        rawSamples[i] = (int16_t)analogReadMilliVolts(pin);
        unsigned long target = t0 + (unsigned long)((i + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}
    }

    // RMS via variance method (same as piimage_3)
    // This correctly extracts the AC component from a signal with DC offset
    double sumValues = 0;
    double sumSquares = 0;
    int16_t minVal = 32767, maxVal = -32768;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        double v = (double)rawSamples[i];
        sumValues += v;
        sumSquares += v * v;
        if (rawSamples[i] < minVal) minVal = rawSamples[i];
        if (rawSamples[i] > maxVal) maxVal = rawSamples[i];
    }

    double avgRaw = sumValues / ADC_SAMPLES;
    double meanSquare = sumSquares / ADC_SAMPLES;
    double variance = meanSquare - (avgRaw * avgRaw);
    if (variance < 0) variance = 0;
    double rmsMv = sqrt(variance);

    // Also compute the DC average above zero baseline (manufacturer's method)
    float dcMv = (float)avgRaw - _zeroMv[channel];
    if (dcMv < 0) dcMv = 0;

    // For this rectified-DC CT board, the DC average is the primary signal.
    // The RMS of variation captures any AC ripple on top of it.
    // Use whichever gives the larger reading (DC average for steady loads,
    // RMS variance for pulsating/switching loads).
    float effectiveMv = max(dcMv, (float)rmsMv);

    float amps = effectiveMv * CALIBRATION_AMPS_PER_MV;
    float power = grid_voltage * amps * DEFAULT_PF;

    r.variation = maxVal - minVal;
    r.avg_mv = (int)dcMv;

    // Noise floor (matches piimage_3: 5W threshold)
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
    return r;
}

// Read all 6 channels
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();
    all.total_watts = 0;

    if (!_calibrated) {
        calibrateCTZero();
    }

    unsigned long t0 = millis();

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        all.ct[ch] = readCTChannel(CT_PINS[ch], ch, grid_voltage);
        all.total_watts += all.ct[ch].watts;
    }

    all.sample_duration_ms = millis() - t0;
    return all;
}

bool isCTCalibrated() { return _calibrated; }
