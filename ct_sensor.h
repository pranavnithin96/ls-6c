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
    // --- Waveform features (FEATURE_WAVEFORM_STATS, live payload only) ---
    // Derived per 500ms window from the same samples readAllCT already takes.
    // Populated unconditionally (cost is tens of us); emitted only when the
    // flag is on. peak/min via the manufacturer formula on max/min count;
    // env_peak_ratio is max/mean of counts (NOT electrical crest factor);
    // ripple is the within-second std in amps; env5 is the 5Hz envelope.
    float peak_amps;
    float min_amps;
    float env_peak_ratio;
    float ripple_amps;
    float env5[5];
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

// --- Waveform-stats feature flag (cached in RAM; NEVER read NVS in the 1Hz path) ---
static bool _wfStatsEnabled = (FEATURE_WAVEFORM_STATS != 0);
bool waveformStatsEnabled() { return _wfStatsEnabled; }
void setWaveformStats(bool on) {
    _wfStatsEnabled = on;
    Preferences p; p.begin("lscfg", false);
    p.putBool("wfstats", on); p.end();
    Serial.printf("[CT] Waveform stats %s\n", on ? "ENABLED" : "disabled");
}
static void loadWaveformStatsPref() {
    Preferences p; p.begin("lscfg", true);
    _wfStatsEnabled = p.getBool("wfstats", (FEATURE_WAVEFORM_STATS != 0));
    p.end();
}

void initCTSensors() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    loadWaveformStatsPref();

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
    Serial.println("[CT] Rating-aware cal: 50A=0.0090/ct, 100A=0.0327/ct, else legacy 0.0123+0.13");
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

// ---------------------------------------------------------------------------
// Rating-aware CT calibration (2.9.0). One formula never fit both sensor types:
// the legacy 0.0123*count+0.13 sat BETWEEN them, so 50A CTs overread ~40% and
// 100A CTs underread ~2.6x on the same firmware. Each channel's rating is set by
// the installer at setup (getCtRating), so we select the empirically-fit slope
// per rating. Fit against a multimeter clamp on PRODUCTION sensors 2026-07-21;
// these CTs read 0 count at 0A, so no +0.13 floor. Fix-forward: readings from
// firmware >= 2.9.0 use these — firmware_version is the calibration stamp.
//   100A : 0.0327 A/count  — solid (resistive furnace, verified 0-35A)
//   50A  : 0.0090 A/count  — PROVISIONAL (verified only to 3.5A, on a motor)
//   150A / unset : legacy manufacturer formula, unchanged (not yet calibrated)
// ---------------------------------------------------------------------------
#define CT_SLOPE_50A     0.0090f
#define CT_SLOPE_100A    0.0327f
#define CT_SLOPE_LEGACY  0.0123f
#define CT_OFFSET_LEGACY 0.13f

static inline float ctSlope(int rating) {
    switch (rating) {
        case 100: return CT_SLOPE_100A;
        case 50:  return CT_SLOPE_50A;
        default:  return CT_SLOPE_LEGACY;
    }
}
// count -> amps for a channel of the given CT rating.
static inline float ctCountToAmps(int rating, float count) {
    switch (rating) {
        case 100: return CT_SLOPE_100A * count;                          // through origin
        case 50:  return CT_SLOPE_50A  * count;                          // through origin
        default:  return CT_SLOPE_LEGACY * count + CT_OFFSET_LEGACY;     // legacy fallback (150A/unset)
    }
}

// Read all CT channels — per-rating calibration on raw ADC counts (see above).
// Legacy fallback keeps the +0.13 floor; the 50A/100A fits are through-origin.
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();  // Timestamp at START of sampling
    all.total_watts = 0;

    uint32_t sumCounts[NUM_CT_CHANNELS] = {0};
    // Waveform accumulators — O(1) integer ops per sample, no extra analogRead().
    // sumSq needs u64: 500 * 4095^2 = 8.4e9 overflows u32.
    uint16_t maxCount[NUM_CT_CHANNELS];
    uint16_t minCount[NUM_CT_CHANNELS];
    uint64_t sumSq[NUM_CT_CHANNELS] = {0};
    uint32_t subSum[NUM_CT_CHANNELS][5] = {{0}};
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) { maxCount[ch] = 0; minCount[ch] = 4095; }

    unsigned long t0 = micros();

    // Interleaved sampling: all 6 channels per 1ms time step.
    // Raw analogRead() counts — NOT analogReadMilliVolts(). The manufacturer's
    // 0.0123 coefficient was empirically fit against raw counts; switching to mV
    // breaks the fit because the count→mV relationship is not constant.
    for (int s = 0; s < ADC_SAMPLES_PER_CH; s++) {
        int sw = (s * 5) / ADC_SAMPLES_PER_CH;        // even 100ms sub-window 0..4
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            uint16_t v = (uint16_t)analogRead(CT_PINS[ch]);   // single read, reused
            sumCounts[ch] += v;
            if (v > maxCount[ch]) maxCount[ch] = v;
            if (v < minCount[ch]) minCount[ch] = v;
            sumSq[ch] += (uint32_t)v * v;
            subSum[ch][sw] += v;
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

        // Rating-aware calibration (2.9.0) — formula chosen by the channel's CT type
        int rating = getCtRating(ch);
        float amps = ctCountToAmps(rating, avgCount);

        // Guard against NaN/Inf only
        if (isnan(amps) || isinf(amps)) amps = 0.0f;

        float power = grid_voltage * amps * DEFAULT_PF;
        r.avg_mv = (int)avgCount;  // Field name kept for compat — now stores avg count
        r.amps = amps;
        r.watts = power;
        r.pf = DEFAULT_PF;

        // --- Waveform features (same manufacturer formula on each statistic) ---
        // Computed unconditionally (tens of us total); emitted only when the flag
        // is on (see queueReading). Saturates with the ADC at ~50.5A: on a pinned
        // channel max≈avg → ratio→1, ripple→0 (expected, documented).
        r.peak_amps = ctCountToAmps(rating, maxCount[ch]);
        r.min_amps  = ctCountToAmps(rating, minCount[ch]);

        // Within-second std in counts. Compute E[X^2]-E[X]^2 in DOUBLE: the terms
        // are close at ~1e6-1e7 magnitudes and float32 loses the difference to
        // catastrophic cancellation (you'd read zero ripple under load).
        double meanC  = (double)sumCounts[ch] / ADC_SAMPLES_PER_CH;
        double meanSq = (double)sumSq[ch] / ADC_SAMPLES_PER_CH;
        double var    = meanSq - meanC * meanC;
        if (var < 0.0) var = 0.0;                  // guard tiny negative from rounding
        r.ripple_amps = ctSlope(rating) * (float)sqrt(var);  // spread scaled by channel slope, no offset

        // env_peak_ratio: within-second max/mean of the rectified envelope.
        // NOT electrical crest factor (no waveform access; +0.13 offset means
        // count-ratio != amp-ratio). Computed on counts; clamped [1,20].
        float ratio = (float)maxCount[ch] / (avgCount > 1.0f ? avgCount : 1.0f);
        if (ratio < 1.0f)  ratio = 1.0f;
        if (ratio > 20.0f) ratio = 20.0f;
        r.env_peak_ratio = ratio;

        // env5: five 100ms sub-window means -> 5Hz envelope, in amps.
        const int subN = ADC_SAMPLES_PER_CH / 5;
        for (int k = 0; k < 5; k++) {
            float subAvg = (float)subSum[ch][k] / subN;
            r.env5[k] = ctCountToAmps(rating, subAvg);
        }

        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    return all;
}

bool isCTCalibrated() { return true; }  // Manufacturer formula needs no calibration
bool isMultiCalLoaded() { return _multiCalLoaded; }
