#pragma once
#include <Arduino.h>
#include <Preferences.h>

// CT pin mapping (LS-6C-IOT_V1.0 schematic — all ADC1)
#define NUM_CT_CHANNELS 6

// Sampling: interleaved across all 6 channels for ~950ms
#define ADC_SAMPLES_PER_CH 950
#define SAMPLE_INTERVAL_US 1000  // 1ms between sample sets

// Default linear calibration (fallback if no multi-point cal)
#define DEFAULT_AMPS_PER_MV 0.01526f

// Power factor (assumed)
#define DEFAULT_PF 0.9f

// Noise threshold in watts
#define NOISE_THRESHOLD_W 5.0f

// Multi-point calibration: 3 points (low, mid, high) per channel
// Stored as mV -> Amps pairs. Interpolated between points.
#define CAL_POINTS 3

static const uint8_t CT_PINS[NUM_CT_CHANNELS] = {
    36, 39, 34, 35, 32, 33
};

struct CalPoint {
    float mv;    // millivolts reading
    float amps;  // known actual amps
};

struct CTReading {
    float amps;
    float watts;
    float pf;
    float voltage;
    int avg_mv;
    int variation;
    int samples;
    bool valid;
};

struct AllCTReadings {
    CTReading ct[NUM_CT_CHANNELS];
    float total_watts;
    unsigned long timestamp_ms;
    unsigned long sample_duration_ms;
};

// Per-channel zero calibration
static float _zeroMv[NUM_CT_CHANNELS] = {0};
static bool _calibrated = false;

// Per-channel multi-point calibration
static CalPoint _calPoints[NUM_CT_CHANNELS][CAL_POINTS];
static bool _multiCalLoaded = false;

// Convert mV to amps using multi-point interpolation
static float mvToAmps(int channel, float mv) {
    if (!_multiCalLoaded) {
        return mv * DEFAULT_AMPS_PER_MV;
    }

    CalPoint* pts = _calPoints[channel];

    // Below first point: linear extrapolation from first two points
    if (mv <= pts[0].mv) {
        if (pts[1].mv == pts[0].mv) return pts[0].amps;
        float slope = (pts[1].amps - pts[0].amps) / (pts[1].mv - pts[0].mv);
        return pts[0].amps + slope * (mv - pts[0].mv);
    }

    // Interpolate between points
    for (int i = 0; i < CAL_POINTS - 1; i++) {
        if (mv <= pts[i + 1].mv) {
            float range = pts[i + 1].mv - pts[i].mv;
            if (range == 0) return pts[i].amps;
            float t = (mv - pts[i].mv) / range;
            return pts[i].amps + t * (pts[i + 1].amps - pts[i].amps);
        }
    }

    // Above last point: linear extrapolation from last two points
    int last = CAL_POINTS - 1;
    float slope = (pts[last].amps - pts[last - 1].amps) / (pts[last].mv - pts[last - 1].mv);
    return pts[last].amps + slope * (mv - pts[last].mv);
}

void initCTSensors() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Load zero calibration
    Preferences calPrefs;
    calPrefs.begin("ctcal", true);
    _calibrated = calPrefs.getBool("done", false);
    if (_calibrated) {
        for (int i = 0; i < NUM_CT_CHANNELS; i++) {
            char key[8];
            snprintf(key, sizeof(key), "z%d", i);
            _zeroMv[i] = calPrefs.getFloat(key, 0.0f);
        }
        Serial.println("[CT] Loaded zero calibration");
    }

    // Load multi-point calibration
    _multiCalLoaded = calPrefs.getBool("mcal", false);
    if (_multiCalLoaded) {
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            for (int p = 0; p < CAL_POINTS; p++) {
                char keyMv[12], keyA[12];
                snprintf(keyMv, sizeof(keyMv), "c%dp%dmv", ch, p);
                snprintf(keyA, sizeof(keyA), "c%dp%da", ch, p);
                _calPoints[ch][p].mv = calPrefs.getFloat(keyMv, 0.0f);
                _calPoints[ch][p].amps = calPrefs.getFloat(keyA, 0.0f);
            }
        }
        Serial.println("[CT] Loaded multi-point calibration");
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            Serial.printf("[CT] CH%d cal: ", ch + 1);
            for (int p = 0; p < CAL_POINTS; p++) {
                Serial.printf("%.0fmV=%.2fA ", _calPoints[ch][p].mv, _calPoints[ch][p].amps);
            }
            Serial.println();
        }
    } else {
        Serial.printf("[CT] Using default linear: %.5f A/mV\n", DEFAULT_AMPS_PER_MV);
    }
    calPrefs.end();

    Serial.printf("[CT] 6-channel sensors initialized\n");
    Serial.printf("[CT] Interleaved: %d samples/ch over %dms, PF: %.1f, noise: %.0fW\n",
        ADC_SAMPLES_PER_CH, (ADC_SAMPLES_PER_CH * SAMPLE_INTERVAL_US) / 1000,
        DEFAULT_PF, NOISE_THRESHOLD_W);

    for (int i = 0; i < NUM_CT_CHANNELS; i++) {
        Serial.printf("[CT] CH%d zero: %.1f mV\n", i + 1, _zeroMv[i]);
    }

    if (!_calibrated) {
        Serial.println("[CT] No calibration — will auto-zero on first read");
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
    Serial.println("[CT] Calibration saved");
    Serial.println("[CT] ========================");
}

// Set a multi-point calibration point for a channel
// Usage via serial: "calpoint <ch> <point> <known_amps>"
// The device reads the current mV and maps it to the known amps value
void setCalPoint(int channel, int point, float knownAmps) {
    if (channel < 0 || channel >= NUM_CT_CHANNELS || point < 0 || point >= CAL_POINTS) {
        Serial.println("[CT] Invalid channel or point");
        return;
    }

    // Read current mV for this channel (500 samples)
    uint32_t sum = 0;
    unsigned long t0 = micros();
    for (int i = 0; i < 500; i++) {
        sum += analogReadMilliVolts(CT_PINS[channel]);
        unsigned long target = t0 + (unsigned long)((i + 1) * 1000);
        while (micros() < target) {}
    }
    float avgMv = (float)sum / 500.0f;
    float corrected = avgMv - _zeroMv[channel];
    if (corrected < 0) corrected = 0;

    _calPoints[channel][point].mv = corrected;
    _calPoints[channel][point].amps = knownAmps;

    Serial.printf("[CT] CH%d point %d: %.1f mV = %.3f A\n",
        channel + 1, point, corrected, knownAmps);

    // Save to NVS
    Preferences calPrefs;
    calPrefs.begin("ctcal", false);
    calPrefs.putBool("mcal", true);
    char keyMv[12], keyA[12];
    snprintf(keyMv, sizeof(keyMv), "c%dp%dmv", channel, point);
    snprintf(keyA, sizeof(keyA), "c%dp%da", channel, point);
    calPrefs.putFloat(keyMv, corrected);
    calPrefs.putFloat(keyA, knownAmps);
    calPrefs.end();

    _multiCalLoaded = true;
    Serial.println("[CT] Multi-point cal saved to NVS");
}

// Read all 6 channels — interleaved sampling
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();
    all.total_watts = 0;

    if (!_calibrated) {
        calibrateCTZero();
    }

    double sumValues[NUM_CT_CHANNELS] = {0};
    double sumSquares[NUM_CT_CHANNELS] = {0};
    int16_t minVal[NUM_CT_CHANNELS];
    int16_t maxVal[NUM_CT_CHANNELS];
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        minVal[ch] = 32767;
        maxVal[ch] = -32768;
    }

    unsigned long t0 = micros();

    for (int s = 0; s < ADC_SAMPLES_PER_CH; s++) {
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            int16_t mv = (int16_t)analogReadMilliVolts(CT_PINS[ch]);
            double v = (double)mv;
            sumValues[ch] += v;
            sumSquares[ch] += v * v;
            if (mv < minVal[ch]) minVal[ch] = mv;
            if (mv > maxVal[ch]) maxVal[ch] = mv;
        }
        unsigned long target = t0 + (unsigned long)((s + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}
    }

    all.sample_duration_ms = (micros() - t0) / 1000;

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples = ADC_SAMPLES_PER_CH;

        double avgRaw = sumValues[ch] / ADC_SAMPLES_PER_CH;
        double meanSquare = sumSquares[ch] / ADC_SAMPLES_PER_CH;
        double variance = meanSquare - (avgRaw * avgRaw);
        if (variance < 0) variance = 0;
        double rmsMv = sqrt(variance);

        float dcMv = (float)avgRaw - _zeroMv[ch];
        if (dcMv < 0) dcMv = 0;

        float effectiveMv = max(dcMv, (float)rmsMv);

        // Use multi-point calibration if available, otherwise linear
        float amps = mvToAmps(ch, effectiveMv);
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
bool isMultiCalLoaded() { return _multiCalLoaded; }
