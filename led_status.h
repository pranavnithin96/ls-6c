#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================================
// LED State Machine — Thread-safe with spinlock
// ============================================================================

enum LEDState {
    LED_BOOTING,            // Fast 100ms blink
    LED_WIFI_CONNECTING,    // 500ms blink
    LED_WIFI_AP_MODE,       // Double blink + pause
    LED_RUNNING,            // Solid on
    LED_SENDING,            // Brief off-pulse every 2s
    LED_ERROR,              // Triple fast blink + pause (auto-clears after 30s)
    LED_OTA_UPDATING,       // Very fast 50ms blink
    LED_WIFI_DISCONNECTED   // Slow 1s blink
};

static portMUX_TYPE _ledMux = portMUX_INITIALIZER_UNLOCKED;
static volatile LEDState _ledState = LED_BOOTING;
static unsigned long _ledLastToggle = 0;
static bool _ledOn = false;
static int _ledBlinkCount = 0;
static unsigned long _ledErrorTimeout = 0;

void initLED() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    _ledOn = false;
    _ledLastToggle = millis();
    _ledBlinkCount = 0;
}

void setLEDState(LEDState state) {
    portENTER_CRITICAL(&_ledMux);
    if (_ledState != state) {
        _ledState = state;
        _ledBlinkCount = 0;
        _ledLastToggle = millis();
        if (state == LED_ERROR) {
            _ledErrorTimeout = millis() + 30000;
        }
        if (state == LED_RUNNING) {
            _ledOn = true;
            digitalWrite(LED_PIN, HIGH);
        }
    }
    portEXIT_CRITICAL(&_ledMux);
}

LEDState getLEDState() {
    portENTER_CRITICAL(&_ledMux);
    LEDState s = _ledState;
    portEXIT_CRITICAL(&_ledMux);
    return s;
}

void updateLED() {
    unsigned long now = millis();

    portENTER_CRITICAL(&_ledMux);
    LEDState state = _ledState;
    portEXIT_CRITICAL(&_ledMux);

    // Auto-clear error state after timeout
    if (state == LED_ERROR && _ledErrorTimeout > 0 && now > _ledErrorTimeout) {
        setLEDState(LED_RUNNING);
        return;
    }

    unsigned long elapsed = now - _ledLastToggle;

    switch (state) {
        case LED_BOOTING:
            if (elapsed >= 100) {
                _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledLastToggle = now;
            }
            break;
        case LED_WIFI_CONNECTING:
            if (elapsed >= 500) {
                _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledLastToggle = now;
            }
            break;
        case LED_WIFI_AP_MODE:
            if (_ledBlinkCount < 4) {
                if (elapsed >= 150) { _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledBlinkCount++; _ledLastToggle = now; }
            } else {
                if (elapsed >= 800) { _ledBlinkCount = 0; _ledLastToggle = now; }
            }
            break;
        case LED_RUNNING:
            if (!_ledOn) { _ledOn = true; digitalWrite(LED_PIN, HIGH); }
            break;
        case LED_SENDING:
            if (_ledOn && elapsed >= 2000) { _ledOn = false; digitalWrite(LED_PIN, LOW); _ledLastToggle = now; }
            else if (!_ledOn && elapsed >= 50) { _ledOn = true; digitalWrite(LED_PIN, HIGH); _ledLastToggle = now; }
            break;
        case LED_ERROR:
            if (_ledBlinkCount < 6) {
                if (elapsed >= 100) { _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledBlinkCount++; _ledLastToggle = now; }
            } else {
                if (elapsed >= 1000) { _ledBlinkCount = 0; _ledLastToggle = now; }
            }
            break;
        case LED_OTA_UPDATING:
            if (elapsed >= 50) { _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledLastToggle = now; }
            break;
        case LED_WIFI_DISCONNECTED:
            if (elapsed >= 1000) { _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledLastToggle = now; }
            break;
    }
}
