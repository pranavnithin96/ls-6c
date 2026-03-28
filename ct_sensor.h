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

// Below this millivolt delta (after zero subtraction) is noise
#define NOISE_FLOOR_MV 5.0f

static const uint8_t CT_PINS[NUM_CT_CHANNELS] = {
    36, 39, 34, 35, 32, 33
};

struct CTReading {
    float amps;
    float watts;       // apparent power = V * A
    float voltage;
    int avg_mv;        // millivolts above zero baseline
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
    Serial.printf("[CT] Scale: %.5f A/mV, noise floor: %.0f mV\n",
        CALIBRATION_AMPS_PER_MV, NOISE_FLOOR_MV);

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

// Read a single CT channel
static float readCTMillivolts(uint8_t pin, int channel) {
    uint32_t sum = 0;

    unsigned long t0 = micros();
    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogReadMilliVolts(pin);
        unsigned long target = t0 + (unsigned long)((i + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}
    }

    float avgMv = (float)sum / ADC_SAMPLES;

    // Subtract per-channel zero offset
    float corrected = avgMv - _zeroMv[channel];
    if (corrected < NOISE_FLOOR_MV) {
        corrected = 0.0f;
    }

    return corrected;
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
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples = ADC_SAMPLES;

        float mv = readCTMillivolts(CT_PINS[ch], ch);
        float amps = mv * CALIBRATION_AMPS_PER_MV;

        r.avg_mv = (int)mv;
        r.amps = amps;
        r.watts = grid_voltage * amps;
        r.valid = true;

        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    all.sample_duration_ms = millis() - t0;
    return all;
}

bool isCTCalibrated() { return _calibrated; }
