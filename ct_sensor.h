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
    // flag is on. ALL of these are computed on the ENVELOPE (20ms sub-window
    // means, one mains cycle each) as of 2.13.0 — never on raw ADC samples,
    // which measured the 50Hz carrier rather than the load. peak/min are the
    // envelope's max/min load; ripple is the envelope std (genuine load
    // variation, 0 for a perfectly steady load); env_peak_ratio is envelope
    // max/mean; env5 is the same envelope aggregated to 5 x 100ms.
    float peak_amps;
    float min_amps;
    float env_peak_ratio;
    float ripple_amps;
    float env5[5];
    // Per-channel current fundamental from hump timing (2.17.0). On the
    // lathes/VMCs the CTs sit on the VFD OUTPUT (confirmed meton_01/sb3/
    // pcs11, 08-03..05), so this is the spindle's electrical frequency — a
    // live RPM proxy. Mains-side channels just read ~50. 0 = unmeasurable
    // this window. Wider 10-200Hz plausibility band than mains_hz: VFD
    // output observed 26-92Hz in the field, headroom above for fast spindles.
    float hz;
};

struct AllCTReadings {
    CTReading ct[NUM_CT_CHANNELS];
    float total_watts;
    unsigned long timestamp_ms;      // millis() at sample START
    unsigned long sample_duration_ms;
    // Mains frequency measured from the rectified waveform's hump timing on
    // the strongest channel (2.14.0). One hump per mains cycle (half-wave
    // rectified front end), sub-sample interpolated at the threshold
    // crossings. 0.0 = not measurable this window (no loaded channel, or
    // fewer than 5 clean humps). Clamped to the plausible 45-65Hz band.
    float mains_hz;
};

// --- Runtime sampling window (2.14.0) --------------------------------------
// 500ms default (identical behavior to 2.13.x). set via setSampleWindowMs()
// from the set_window_ms heartbeat command; persisted in lscfg/winms.
static int _winSamples = ADC_SAMPLES_PER_CH;

int getSampleWindowMs() { return _winSamples; }   // 1 sample == 1 ms at 1kHz

bool setSampleWindowMs(int ms) {
    if (ms < MIN_ADC_SAMPLES || ms > MAX_ADC_SAMPLES || ms % 100 != 0)
        return false;                 // %100 keeps 20ms sub-windows AND env5 exact
    _winSamples = ms;
    Preferences p; p.begin("lscfg", false);
    p.putInt("winms", ms);
    p.end();
    Serial.printf("[CT] sample window set: %dms\n", ms);
    return true;
}

static void loadSampleWindowPref() {
    Preferences p; p.begin("lscfg", true);
    int ms = p.getInt("winms", ADC_SAMPLES_PER_CH);
    p.end();
    if (ms >= MIN_ADC_SAMPLES && ms <= MAX_ADC_SAMPLES && ms % 100 == 0)
        _winSamples = ms;
}

// Multi-point calibration kept as future option — NOT used by default path.
// Default path uses manufacturer's empirical formula: amps = 0.0123 * count + 0.13
static portMUX_TYPE _calMux = portMUX_INITIALIZER_UNLOCKED;
static CalPoint _calPoints[NUM_CT_CHANNELS][CAL_POINTS];
static bool _multiCalLoaded = false;

// Forward declarations
void feedWatchdog();
static void loadChannelSlopes();   // defined below; called from initCTSensors
static void loadDcCal();           // defined below; called from initCTSensors
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
    loadDcCal();
    loadSampleWindowPref();

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

// ---------------------------------------------------------------------------
// DC power special case (2.17.4.1). Some installs tap a line whose true power
// is read off an external kWh meter rather than derived from amps*volts*pf —
// e.g. a DC bus feeding a metered load. For those channels the trustworthy
// relationship is count -> kW, fitted directly against the meter, with voltage
// sag and sensor scaling already folded into the meter's own reading.
//
// When a channel's DC slope is set (nonzero), readAllCT reports:
//     kW    = dcSlope * avgCount + dcOffset      (clamped at 0 below no-load)
//     watts = kW * 1000, pf = 1.0, amps = watts / grid_voltage (bookkeeping)
// All other channels keep the stock AC path untouched.
//
// Persisted in Preferences("ctcal") keys "d0".."d5" (slope, kW/count) and
// "o0".."o5" (offset, kW); survives reboot and OTA like the AC slopes.
// Calibration workflow over serial, points held in RAM until adopted:
//     dcpoint <ch> <kW>   sample a fresh window NOW, pair it with the meter
//     dcfit  <ch>         least-squares fit + R^2 over recorded points
//     dcadopt <ch>        persist the fit as the channel's live cal
//     dcset <ch> <s> <o>  manual set (0 0 clears -> stock AC path)
//     dcshow / dcclear
// ---------------------------------------------------------------------------
#define DC_MAX_POINTS 12
struct DcPoint { float count; float kw; };
static float _dcKwSlope[NUM_CT_CHANNELS]  = {0};   // kW per count; 0 = DC mode off
static float _dcKwOffset[NUM_CT_CHANNELS] = {0};   // kW
static DcPoint _dcPts[NUM_CT_CHANNELS][DC_MAX_POINTS];
static int  _nDcPts[NUM_CT_CHANNELS] = {0};

bool dcModeEnabled(int ch) {
    return ch >= 0 && ch < NUM_CT_CHANNELS && _dcKwSlope[ch] != 0.0f;
}

static void loadDcCal() {
    Preferences p; p.begin("ctcal", true);
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        char kd[4], ko[4];
        snprintf(kd, sizeof(kd), "d%d", ch);
        snprintf(ko, sizeof(ko), "o%d", ch);
        _dcKwSlope[ch]  = p.getFloat(kd, 0.0f);
        _dcKwOffset[ch] = p.getFloat(ko, 0.0f);
        if (_dcKwSlope[ch] != 0.0f)
            Serial.printf("[DC] CH%d cal: kW = %.6f*count %+.3f\n",
                          ch + 1, _dcKwSlope[ch], _dcKwOffset[ch]);
    }
    p.end();
}

// Set (or clear with 0 0) a channel's DC kW calibration — live AND persisted.
// Slope range covers taps from watts-scale bench rigs to MW-scale buses;
// offset bounded so a fat-fingered value can't fabricate a phantom megawatt.
bool setDcCal(int ch, float kwPerCount, float offsetKw) {
    if (ch < 0 || ch >= NUM_CT_CHANNELS) return false;
    bool clearing = (kwPerCount == 0.0f && offsetKw == 0.0f);
    if (!clearing) {
        if (kwPerCount <= 0.0f || kwPerCount > 10.0f) return false;
        if (fabsf(offsetKw) > 1000.0f) return false;
    }
    _dcKwSlope[ch]  = kwPerCount;
    _dcKwOffset[ch] = offsetKw;
    Preferences p; p.begin("ctcal", false);
    char kd[4], ko[4];
    snprintf(kd, sizeof(kd), "d%d", ch);
    snprintf(ko, sizeof(ko), "o%d", ch);
    p.putFloat(kd, kwPerCount);
    p.putFloat(ko, offsetKw);
    p.end();
    if (clearing) Serial.printf("[DC] CH%d cal cleared -> stock AC path\n", ch + 1);
    else Serial.printf("[DC] CH%d cal set: kW = %.6f*count %+.3f\n", ch + 1, kwPerCount, offsetKw);
    return true;
}
float getDcSlope(int ch)  { return (ch >= 0 && ch < NUM_CT_CHANNELS) ? _dcKwSlope[ch]  : 0.0f; }
float getDcOffset(int ch) { return (ch >= 0 && ch < NUM_CT_CHANNELS) ? _dcKwOffset[ch] : 0.0f; }

// Average RAW counts over one full window, same cadence as readAllCT — raw
// analogRead, never analogReadMilliVolts (the fit must live in count space).
static float dcSampleAvgCount(int ch) {
    uint32_t sum = 0;
    unsigned long t0 = micros();
    for (int i = 0; i < ADC_SAMPLES_PER_CH; i++) {
        sum += (uint16_t)analogRead(CT_PINS[ch]);
        while (micros() < t0 + (unsigned long)((i + 1) * SAMPLE_INTERVAL_US)) {}
        if (i % 100 == 0) feedWatchdog();
    }
    return (float)sum / ADC_SAMPLES_PER_CH;
}

static bool dcFitPoints(int ch, float& A, float& B, float& r2) {
    int n = _nDcPts[ch];
    if (n < 2) return false;
    double Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
    for (int i = 0; i < n; i++) {
        double x = _dcPts[ch][i].count, y = _dcPts[ch][i].kw;
        Sx += x; Sy += y; Sxx += x * x; Sxy += x * y;
    }
    double denom = n * Sxx - Sx * Sx;
    if (fabs(denom) < 1e-9) return false;              // all points at one count
    A = (float)((n * Sxy - Sx * Sy) / denom);
    B = (float)((Sy - A * Sx) / n);
    double meanY = Sy / n, ssTot = 0, ssRes = 0;
    for (int i = 0; i < n; i++) {
        double y = _dcPts[ch][i].kw, yh = A * _dcPts[ch][i].count + B;
        ssTot += (y - meanY) * (y - meanY);
        ssRes += (y - yh) * (y - yh);
    }
    r2 = (ssTot > 1e-12) ? (float)(1.0 - ssRes / ssTot) : 1.0f;
    return true;
}

void dcPrintFit(int ch) {
    float A, B, r2;
    Serial.printf("[DC] CH%d fit (%d points):\n", ch + 1, _nDcPts[ch]);
    if (!dcFitPoints(ch, A, B, r2)) {
        Serial.println("  need >=2 points at different loads");
        return;
    }
    Serial.printf("  kW = %.6f * count %+.3f   (R^2 = %.5f)\n", A, B, r2);
    if (B > 0.0f)
        Serial.printf("  ** +%.2f kW at zero count — record a low/zero-load dcpoint to pin the offset **\n", B);
    Serial.println("  count |  true kW |  fit kW  (err)");
    for (int i = 0; i < _nDcPts[ch]; i++) {
        float c = _dcPts[ch][i].count, t = _dcPts[ch][i].kw, f = A * c + B;
        Serial.printf("  %5.0f | %8.2f | %8.2f (%+.2f)\n", c, t, f, f - t);
    }
    Serial.printf("  'dcadopt %d' to make this live\n", ch + 1);
}

// Record a synchronized (count, kW) pair: samples a fresh window RIGHT NOW, so
// call it while reading the meter — the pairing is only as good as that moment.
void dcRecordPoint(int ch, float kw) {
    if (ch < 0 || ch >= NUM_CT_CHANNELS || kw < 0.0f) {
        Serial.println("[DC] invalid channel/kW");
        return;
    }
    if (_nDcPts[ch] >= DC_MAX_POINTS) {
        Serial.printf("[DC] CH%d point buffer full — 'dcclear %d' first\n", ch + 1, ch + 1);
        return;
    }
    float c = dcSampleAvgCount(ch);
    _dcPts[ch][_nDcPts[ch]++] = {c, kw};
    Serial.printf("[DC] CH%d point %d: count=%.1f  meter=%.2f kW\n", ch + 1, _nDcPts[ch], c, kw);
    if (_nDcPts[ch] >= 2) dcPrintFit(ch);
}

bool dcAdoptFit(int ch) {
    float A, B, r2;
    if (ch < 0 || ch >= NUM_CT_CHANNELS || !dcFitPoints(ch, A, B, r2)) {
        Serial.println("[DC] no fit to adopt — need >=2 dcpoints");
        return false;
    }
    if (!setDcCal(ch, A, B)) {
        Serial.printf("[DC] fit rejected by sanity range (slope %.6f, offset %.3f)\n", A, B);
        return false;
    }
    return true;
}

void dcClearPoints(int ch) {
    if (ch < 0 || ch >= NUM_CT_CHANNELS) return;
    _nDcPts[ch] = 0;
    Serial.printf("[DC] CH%d points cleared (persisted cal untouched)\n", ch + 1);
}

// ---------------------------------------------------------------------------
// Factory-default DC cal for known DC-tap installs (2.17.4.1). Applied at boot
// ONLY when the channel has no stored cal, so a real dcadopt / set_dc_cal
// always wins and is never clobbered. This is how a fresh flash knows "this
// board needs the DC path": the table below is keyed by provisioned device id.
//
// pcs_21 CH1 provisional: two-point fit from the 2026-08-31 bench sync
// (count 1487 <-> 440 kW, count 1805 <-> ~500 kW). Good to ~±7 kW inside the
// 400-500 kW band; OVERREADS at low load (+159 kW at zero count) — replace
// with a real multi-point dcfit/dcadopt (incl. a low-load point) or a server
// set_dc_cal at the first opportunity.
// ---------------------------------------------------------------------------
void applyDcDefaultsFor(const String& deviceId) {
    if (deviceId == "pcs_21" && _dcKwSlope[0] == 0.0f) {
        if (setDcCal(0, 0.18868f, 159.43f))
            Serial.println("[DC] CH1 PROVISIONAL cal applied (pcs_21 factory default)"
                           " — refine with dcpoint/dcfit/dcadopt");
    }
}

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

// ---------------------------------------------------------------------------
// Envelope resolution for the waveform features. 25 sub-windows over the 500ms
// sampling window = 20ms each = exactly ONE 50Hz mains cycle per sub-window, so
// each sub-window mean cancels the AC carrier and reports load. Must divide
// ADC_SAMPLES_PER_CH evenly, and must be a multiple of 5 so env5 (the 5 x 100ms
// payload envelope) can be aggregated from it exactly.
// NOTE: sized for 50Hz mains (India). Grid frequency drift leaves a small
// residual because a 20ms window then spans slightly more or less than one full
// cycle. Simulated ripple/mean floor on a perfectly steady load:
//     49.5Hz 0.5%   50.0Hz 0.0%   50.5Hz 0.6%   49/51Hz 1.0%   48/52Hz 2.0%
// Genuine load variation reads 8-32% of mean, so the floor is ~1/15th of the
// smallest real signal — but any downstream "is this load steady" threshold
// should sit above ~2% rather than at zero.
// On a 60Hz supply a 20ms window spans 1.2 cycles and the floor is much larger.
// If the fleet ever runs on 60Hz, use ENV_SUBWIN 10: 50 samples = 50ms = exactly
// 3 mains cycles at 60Hz, and it satisfies both asserts below. (30 does NOT —
// 500/30 is not an integer, so 16.67ms sub-windows are not reachable at 1kHz.)
// ---------------------------------------------------------------------------
// ENV_SUBWIN became runtime in 2.14.0 (n_subwin below) because the window
// length is runtime-set; the 20ms sub-window itself is the fixed invariant.
static_assert(ADC_SAMPLES_PER_CH % (ENV_SUBWIN_SAMPLES * 5) == 0,
              "default window must divide into 20ms sub-windows and 5 env5 slots");
static_assert(MAX_ADC_SAMPLES % (ENV_SUBWIN_SAMPLES * 5) == 0,
              "max window must divide into 20ms sub-windows and 5 env5 slots");

// Black box hook — defined in waveform_blackbox.h (same translation unit).
void wbFeed(int ch, uint16_t v);

// Read all CT channels — per-rating calibration on raw ADC counts (see above).
// Legacy fallback keeps the +0.13 floor; the 50A/100A fits are through-origin.
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();  // Timestamp at START of sampling
    all.total_watts = 0;

    const int nSamples = _winSamples;                       // runtime window
    const int n_subwin = nSamples / ENV_SUBWIN_SAMPLES;     // 20ms sub-windows

    uint32_t sumCounts[NUM_CT_CHANNELS] = {0};
    uint16_t maxCount[NUM_CT_CHANNELS] = {0};   // for channel selection only,
                                                // NOT features (2.13.0 lesson)
    // Envelope accumulator — n_subwin sub-windows, 20ms each = exactly one
    // 50Hz mains cycle, so each sub-window mean averages the AC carrier out and
    // leaves the LOAD level. O(1) integer ops per sample, no extra analogRead().
    // Worst case per sub-window: 20 * 4095 = 81,900 — fits u32 with room.
    //
    // static, not stack: at 6 x 45 x u32 this is 1080B, and the Arduino loopTask
    // stack is only 8KB (networkTask's 16KB is a different task). readAllCT is
    // called ONLY from loop() — both call sites are in Line_Sight.ino — so one
    // shared instance is safe. If it ever gains a second calling task, this must
    // move back onto the stack or take a lock.
    static uint32_t subSum[NUM_CT_CHANNELS][MAX_ENV_SUBWIN];
    memset(subSum, 0, sizeof(subSum));

    // --- Mains-frequency state (2.14.0) ---
    // Hump timing on the STRONGEST channel of the previous window. The
    // rectified front end produces one hump per mains cycle; counting
    // threshold crossings with sub-sample interpolation gives grid frequency
    // to ~0.01Hz over a 500ms window. Threshold = half the previous window's
    // peak (loads change slowly relative to 1s), with hysteresis so ADC noise
    // near the threshold can't double-count a hump.
    static int _strongCh = 0;
    static uint16_t _prevWinMax[NUM_CT_CHANNELS] = {0};
    const int fq = _strongCh;
    // 2.17.0: hump timing runs on EVERY loaded channel (per-channel hz — the
    // spindle-frequency feature), not just the strongest. Same detector, same
    // thresholds-from-last-window trick, state promoted to arrays. Cost is a
    // few integer compares per channel per sample — noise next to the 6
    // analogRead()s that dominate the 1ms budget.
    uint16_t thrHiC[NUM_CT_CHANNELS], thrLoC[NUM_CT_CHANNELS];
    bool usableC[NUM_CT_CHANNELS], aboveC[NUM_CT_CHANNELS];
    int nCrossC[NUM_CT_CHANNELS];
    float firstXC[NUM_CT_CHANNELS], lastXC[NUM_CT_CHANNELS];
    float maxIvC[NUM_CT_CHANNELS], minIvC[NUM_CT_CHANNELS];
    uint16_t prevVC[NUM_CT_CHANNELS];
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        thrHiC[ch] = _prevWinMax[ch] / 2;
        thrLoC[ch] = (uint16_t)(_prevWinMax[ch] * 3 / 8);   // 12.5% hysteresis
        usableC[ch] = thrHiC[ch] >= 100;   // needs a real load to have humps
        aboveC[ch] = false; nCrossC[ch] = 0;
        firstXC[ch] = 0.0f; lastXC[ch] = 0.0f;
        maxIvC[ch] = 0.0f; minIvC[ch] = 1e9f;
        prevVC[ch] = 0;
    }
    const bool fqUsable = usableC[fq];

    unsigned long t0 = micros();

    // Interleaved sampling: all 6 channels per 1ms time step.
    // Raw analogRead() counts — NOT analogReadMilliVolts(). The manufacturer's
    // 0.0123 coefficient was empirically fit against raw counts; switching to mV
    // breaks the fit because the count→mV relationship is not constant.
    for (int s = 0; s < nSamples; s++) {
        int sw = s / ENV_SUBWIN_SAMPLES;    // fixed 20ms sub-window index
        for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
            uint16_t v = (uint16_t)analogRead(CT_PINS[ch]);   // single read, reused
            sumCounts[ch] += v;
            subSum[ch][sw] += v;
            if (v > maxCount[ch]) maxCount[ch] = v;
            if (ch == fq) wbFeed(ch, v);    // black-box ring (one u16 store)
            if (usableC[ch]) {
                if (!aboveC[ch] && v >= thrHiC[ch]) {
                    // rising crossing; interpolate between s-1 and s
                    float frac = (v > prevVC[ch] && s > 0)
                               ? (float)(thrHiC[ch] - prevVC[ch]) / (float)(v - prevVC[ch])
                               : 0.0f;
                    if (frac < 0.0f) frac = 0.0f;
                    if (frac > 1.0f) frac = 1.0f;
                    float x = (float)(s - 1) + frac;
                    if (x < 0.0f) x = 0.0f;   // s==0: no s-1 to interpolate from
                    if (nCrossC[ch] == 0) firstXC[ch] = x;
                    else {
                        float iv = x - lastXC[ch];
                        if (iv > maxIvC[ch]) maxIvC[ch] = iv;
                        if (iv < minIvC[ch]) minIvC[ch] = iv;
                    }
                    lastXC[ch] = x;
                    nCrossC[ch]++;
                    aboveC[ch] = true;
                } else if (aboveC[ch] && v < thrLoC[ch]) {
                    aboveC[ch] = false;
                }
                prevVC[ch] = v;
            }
        }
        unsigned long target = t0 + (unsigned long)((s + 1) * SAMPLE_INTERVAL_US);
        while (micros() < target) {}  // Busy-wait for precise 1ms cadence
    }

    all.sample_duration_ms = (micros() - t0) / 1000;

    // Mains frequency: humps are 1/cycle, samples are 1ms apart. Require >=5
    // clean humps and a plausible answer; otherwise report 0 (= unknown).
    //
    // Interval-consistency guard (2.14.2): one MISSED interior hump (a shallow
    // hump under the threshold during a load dip) drops the count by one while
    // the span stays put, reading exactly 50 x 23/24 = 47.92Hz — the pilot's
    // outlier cluster, ~0.8% of readings, all at 47.91-47.95. A miss doubles
    // one interval (~40ms), a double-count halves one (~10ms); both are far
    // outside real grid drift (<1% interval variation), so reject the window
    // when any interval strays 50% from the mean. Better no reading than a
    // wrong one — consumers treat 0 as "not measured".
    all.mains_hz = 0.0f;
    if (fqUsable && nCrossC[fq] >= 5 && lastXC[fq] > firstXC[fq]) {
        float meanIv = (lastXC[fq] - firstXC[fq]) / (float)(nCrossC[fq] - 1);
        bool consistent = (maxIvC[fq] <= 1.5f * meanIv) && (minIvC[fq] >= 0.5f * meanIv);
        float hz = 1000.0f / meanIv;
        if (consistent && hz >= 45.0f && hz <= 65.0f) all.mains_hz = hz;
    }
    // Choose next window's frequency/black-box channel: strongest this window.
    // 2.15.1: only ADOPT a new channel when it carries a real load. When every
    // channel idles near zero (machine parked), the argmax jitters across ADC
    // noise on EMPTY channels and each switch resets the black-box ring —
    // measured on meton_01: 445 ring resets in 5h of streaming, truncating
    // exactly the between-part parks the stream exists to record. Below the
    // floor, keep the incumbent: the ring then records the park intact.
    { int best = 0;
      for (int ch = 1; ch < NUM_CT_CHANNELS; ch++)
          if (maxCount[ch] > maxCount[best]) best = ch;
      if (maxCount[best] >= 100 && best != _strongCh) _strongCh = best;
      for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) _prevWinMax[ch] = maxCount[ch];
    }

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples = nSamples;

        float avgCount = (float)sumCounts[ch] / nSamples;

        // Rating-aware calibration (2.9.0) — formula chosen by the channel's CT type
        int rating = getCtRating(ch);
        float amps = ctCountToAmps(ch, rating, avgCount);

        // Guard against NaN/Inf only
        if (isnan(amps) || isinf(amps)) amps = 0.0f;

        float power;
        if (_dcKwSlope[ch] != 0.0f) {
            // DC special case (2.17.4.1): count -> kW fitted directly against
            // the external meter. Clamp below zero — the fitted offset can dip
            // a hair negative at true no-load, and negative power is noise here.
            float kw = _dcKwSlope[ch] * avgCount + _dcKwOffset[ch];
            if (kw < 0.0f || isnan(kw) || isinf(kw)) kw = 0.0f;
            power = kw * 1000.0f;
            // amps back-derived for payload consistency; meter kW is the truth.
            amps = (grid_voltage > 1.0f) ? power / grid_voltage : 0.0f;
            r.pf = 1.0f;
        } else {
            power = grid_voltage * amps * DEFAULT_PF;
            r.pf = DEFAULT_PF;
        }
        r.avg_mv = (int)avgCount;  // Field name kept for compat — now stores avg count
        r.amps = amps;
        r.watts = power;

        // --- Waveform features, computed on the ENVELOPE (2.13.0) ---
        // These were previously computed on RAW ADC samples, which measured the
        // 50Hz carrier instead of the load and made three of the four fields
        // useless. Verified against production data (mark_pdc, 2026-07-31, 408
        // readings): peak_amps sat pinned at 36.9A (= 4095 counts, the ADC rail)
        // in 408/408 readings, and ripple/amps was a near-constant 1.39-1.63
        // across an 11x load range — i.e. a scaled copy of current, carrying no
        // information about how steady the load was. The cause is physical: the
        // CT signal is an AC sine biased mid-scale, so its peaks clip the ADC and
        // its sample-std is inherently proportional to amplitude.
        //
        // Averaging each 20ms sub-window (one full mains cycle) removes the
        // carrier, so max/min/std over those means describe the LOAD. env5 was
        // always correct precisely because it was already a sub-window mean.
        const int subN = ENV_SUBWIN_SAMPLES;   // 20 samples = 20ms, always
        float env[MAX_ENV_SUBWIN];
        float envSum = 0.0f, envMax = 0.0f, envMin = 3.4e38f;
        for (int k = 0; k < n_subwin; k++) {
            env[k] = ctCountToAmps(ch, rating, (float)subSum[ch][k] / subN);
            if (isnan(env[k]) || isinf(env[k])) env[k] = 0.0f;
            envSum += env[k];
            if (env[k] > envMax) envMax = env[k];
            if (env[k] < envMin) envMin = env[k];
        }
        float envMean = envSum / n_subwin;

        r.peak_amps = envMax;      // highest 20ms load in the window
        r.min_amps  = envMin;      // lowest 20ms load in the window

        // Load-variation std over the envelope, in amps. Two-pass about the
        // mean: these are small floats (not the ~1e7 sums the raw-count path
        // used), so there is no catastrophic-cancellation risk here.
        float acc = 0.0f;
        for (int k = 0; k < n_subwin; k++) { float d = env[k] - envMean; acc += d * d; }
        r.ripple_amps = sqrtf(acc / n_subwin);

        // env_peak_ratio: envelope max/mean — burstiness of the LOAD.
        // 1.0 = perfectly steady. Clamped [1,20]. The 0.05A divisor guard keeps
        // a dead/unplugged channel from dividing by ~0 and reporting a bogus 20.
        float ratio = envMax / (envMean > 0.05f ? envMean : 0.05f);
        if (ratio < 1.0f)  ratio = 1.0f;
        if (ratio > 20.0f) ratio = 20.0f;
        r.env_peak_ratio = ratio;

        // env5: five equal slots across the window -> in amps. At the default
        // 500ms window each slot is 100ms (unchanged contract); at a widened
        // window slots stretch proportionally (window/5) — consumers should
        // read `samples` (window ms) rather than assume 100ms slots.
        const int per5 = n_subwin / 5;
        for (int k = 0; k < 5; k++) {
            float s5 = 0.0f;
            for (int j = 0; j < per5; j++) s5 += env[k * per5 + j];
            r.env5[k] = s5 / per5;
        }

        all.ct[ch] = r;
        all.total_watts += r.watts;
    }

    // Per-channel fundamental (2.17.0) — MUST run after the construction loop
    // above: that loop whole-struct-assigns all.ct[ch], which would clobber
    // anything written earlier (review HIGH-1). Same >=5-hump + interval-
    // consistency contract as mains_hz, wider 10-200Hz band (VFD-output
    // channels run the spindle's electrical frequency, not the grid's). The
    // amps floor keeps a floating input's 50Hz pickup from emitting a
    // plausible-looking hz with no real load behind it (review LOW-10).
    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        all.ct[ch].hz = 0.0f;
        if (usableC[ch] && all.ct[ch].amps >= 0.3f &&
            nCrossC[ch] >= 5 && lastXC[ch] > firstXC[ch]) {
            float meanIv = (lastXC[ch] - firstXC[ch]) / (float)(nCrossC[ch] - 1);
            bool consistent = (maxIvC[ch] <= 1.5f * meanIv) && (minIvC[ch] >= 0.5f * meanIv);
            float chz = 1000.0f / meanIv;
            if (consistent && chz >= 10.0f && chz <= 200.0f) all.ct[ch].hz = chz;
        }
    }

    return all;
}

bool isCTCalibrated() { return true; }  // Manufacturer formula needs no calibration
bool isMultiCalLoaded() { return _multiCalLoaded; }
