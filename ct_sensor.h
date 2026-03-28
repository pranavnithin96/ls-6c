#pragma once
#include <Arduino.h>

// CT pin mapping (LS-6C-IOT_V1.0 schematic)
#define NUM_CT_CHANNELS 6

// ADC config
#define ADC_SAMPLES 100
#define ADC_SAMPLE_DELAY_MS 1

// Manufacturer calibration: amps = (0.0123 * avgADC) + 0.13
#define CALIBRATION_SCALE 0.0123f
#define CALIBRATION_OFFSET 0.13f

// No noise filtering — send raw readings
#define NOISE_THRESHOLD_A 0.0f

static const uint8_t CT_PINS[NUM_CT_CHANNELS] = {
    36,  // CT1
    39,  // CT2
    34,  // CT3
    35,  // CT4
    32,  // CT5
    33   // CT6
};

struct CTReading {
    float rms_power_w;
    float rms_current_a;
    float peak_current_a;
    float crest_factor;
    float frequency_hz;
    float voltage;
    float power_factor;
    int adc_variation;
    int samples_captured;
    bool valid;
};

struct AllCTReadings {
    CTReading ct[NUM_CT_CHANNELS];
    float total_power_w;
    unsigned long timestamp_ms;
    unsigned long sample_duration_ms;
};

static int _ctErrors = 0;

void initCTSensors() {
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    Serial.printf("[CT] 6-channel sensors initialized\n");
    Serial.printf("[CT] Calibration: manufacturer (DC average, %d samples)\n", ADC_SAMPLES);
    Serial.printf("[CT] Formula: amps = (%.4f * ADC) + %.2f\n", CALIBRATION_SCALE, CALIBRATION_OFFSET);
}

// Read a single CT channel using manufacturer's averaging method
static float readCTAmps(uint8_t pin) {
    uint32_t sum = 0;

    for (int i = 0; i < ADC_SAMPLES; i++) {
        sum += analogRead(pin);
        delay(ADC_SAMPLE_DELAY_MS);
    }

    int avgADC = sum / ADC_SAMPLES;
    float amps = (CALIBRATION_SCALE * avgADC) + CALIBRATION_OFFSET;

    return amps;
}

// Read all 6 channels
AllCTReadings readAllCT(float grid_voltage) {
    AllCTReadings all = {};
    all.timestamp_ms = millis();
    all.total_power_w = 0;

    unsigned long t0 = millis();

    for (int ch = 0; ch < NUM_CT_CHANNELS; ch++) {
        CTReading r = {};
        r.voltage = grid_voltage;
        r.samples_captured = ADC_SAMPLES;

        float amps = readCTAmps(CT_PINS[ch]);

        r.rms_current_a = amps;
        r.peak_current_a = amps;
        r.crest_factor = (amps > 0) ? 1.0f : 0.0f;
        r.frequency_hz = (amps > 0) ? 50.0f : 0.0f;
        r.power_factor = (amps > 0) ? 0.9f : 0.0f;
        r.rms_power_w = grid_voltage * amps * r.power_factor;
        r.valid = true;

        all.ct[ch] = r;
        all.total_power_w += r.rms_power_w;
    }

    all.sample_duration_ms = millis() - t0;
    return all;
}

int getCTErrors() { return _ctErrors; }
