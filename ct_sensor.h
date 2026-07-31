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

// Forward declarations
void feedWatchdog();
static void loadChannelSlopes();   // defined below; called from initCTSensors
float getChannelSlope(int ch);

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
    loadChannelSlopes();

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
    Serial.println("[CT] Cal: per-channel slope if set, else 50A=0.0421 100A=0.0327 legacy 0.0123+0.13");
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
// Rating-default CT calibration (2.9.0, defaults corrected 2.13.0). One formula
// never fit every sensor: the legacy 0.0123*count+0.13 sat between real builds.
// Each channel's rating is set by the installer (getCtRating); the table below
// maps rating -> empirically-fit slope. All fits are against a multimeter clamp
// on PRODUCTION sensors; these CTs read 0 count at 0A, so no +0.13 floor.
// firmware_version remains the calibration-era stamp for the server.
//   100A : 0.0327 A/count — furnace, verified 0-35A (2026-07-21)
//   50A  : 0.0421 A/count — furnace, 10 clamp points ±1% (2026-07-23, below)
//   150A / unset : legacy manufacturer formula (never calibrated)
// ---------------------------------------------------------------------------
// 50A default corrected 2026-07-23: 0.0421 measured+verified on a standard 50A
// CT (furnace, 10 clamp readings ±1%, zero-anchored, re-verified across a power
// cycle). The old 0.0090 came from a known fewer-turns odd unit and would
// misread standard 50A sensors ~4.7x.
#define CT_SLOPE_50A     0.0421f
#define CT_SLOPE_100A    0.0327f
#define CT_SLOPE_LEGACY  0.0123f
#define CT_OFFSET_LEGACY 0.13f

// ---------------------------------------------------------------------------
// Per-channel measured slope (2.13.0). Field calibration proved same-rating CTs
// are NOT interchangeable — two "50A" sensors measured 5x apart (23.8 vs 115
// counts/A, both live-clamped). The rating table above is only a DEFAULT to get
// a unit through install; the per-SENSOR slope, measured in place with a clamp
// meter, is the trustworthy number. 0 = uncalibrated -> rating fallback.
// Persisted in Preferences("ctcal") keys "s0".."s5"; survives reboot and OTA.
// Set via serial "setslope <ch> <slope>" or heartbeat command "set_ct_slope".
// ---------------------------------------------------------------------------
static float _chSlope[NUM_CT_CHANNELS] = {0};

static void loadChannelSlopes() {
    Preferences p; p.begin("ctcal", true);
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        char k[4]; snprintf(k, sizeof(k), "s%d", ch);
        _chSlope[ch] = p.getFloat(k, 0.0f);
        if (_chSlope[ch] > 0.0f)
            Serial.printf("[CT] CH%d calibrated slope: %.5f A/count\n", ch + 1, _chSlope[ch]);
    }
    p.end();
}

// Set (or clear with 0) a channel's measured slope — live AND persisted, no
// reboot needed (mirrors setSendInterval). Sanity range covers every plausible
// CT build seen so far: 0.001 (1000 counts/A) .. 0.5 (2 counts/A).
bool setChannelSlope(int ch, float aPerCount) {
    if (ch < 0 || ch >= NUM_CT_CHANNELS) return false;
    if (aPerCount != 0.0f && (aPerCount < 0.001f || aPerCount > 0.5f)) return false;
    _chSlope[ch] = aPerCount;
    Preferences p; p.begin("ctcal", false);
    char k[4]; snprintf(k, sizeof(k), "s%d", ch);
    p.putFloat(k, aPerCount);
    p.end();
    Serial.printf("[CT] CH%d slope %s: %.5f A/count\n", ch + 1,
                  aPerCount == 0.0f ? "cleared -> rating fallback" : "set", aPerCount);
    return true;
}
float getChannelSlope(int ch) { return (ch >= 0 && ch < NUM_CT_CHANNELS) ? _chSlope[ch] : 0.0f; }

static inline float ctSlope(int ch, int rating) {
    if (_chSlope[ch] > 0.0f) return _chSlope[ch];    // measured per-sensor slope wins
    switch (rating) {
        case 100: return CT_SLOPE_100A;
        case 50:  return CT_SLOPE_50A;
        default:  return CT_SLOPE_LEGACY;
    }
}
// count -> amps for a channel: measured slope if calibrated, else rating default.
static inline float ctCountToAmps(int ch, int rating, float count) {
    if (_chSlope[ch] > 0.0f) return _chSlope[ch] * count;               // through origin
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
        float amps = ctCountToAmps(ch, rating, avgCount);

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
        r.peak_amps = ctCountToAmps(ch, rating, maxCount[ch]);
        r.min_amps  = ctCountToAmps(ch, rating, minCount[ch]);

        // Within-second std in counts. Compute E[X^2]-E[X]^2 in DOUBLE: the terms
        // are close at ~1e6-1e7 magnitudes and float32 loses the difference to
        // catastrophic cancellation (you'd read zero ripple under load).
        double meanC  = (double)sumCounts[ch] / ADC_SAMPLES_PER_CH;
        double meanSq = (double)sumSq[ch] / ADC_SAMPLES_PER_CH;
        double var    = meanSq - meanC * meanC;
        if (var < 0.0) var = 0.0;                  // guard tiny negative from rounding
        r.ripple_amps = ctSlope(ch, rating) * (float)sqrt(var);  // spread scaled by channel slope, no offset

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
            r.env5[k] = ctCountToAmps(ch, rating, subAvg);
        }

        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    return all;
}

bool isCTCalibrated() { return true; }  // Manufacturer formula needs no calibration
bool isMultiCalLoaded() { return _multiCalLoaded; }
