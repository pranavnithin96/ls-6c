#pragma once
#include <Arduino.h>
#include "config.h"

// ============================================================================
// LED State Machine — Thread-safe with spinlock
// ============================================================================

enum LEDState {
    LED_BOOTING,            // Fast 100ms blink
    LED_WIFI_CONNECTING,    // Fast 100ms blink (same as booting)
    LED_WIFI_AP_MODE,       // Double blink, no pause
    LED_RUNNING,            // Solid on
    LED_ERROR,              // Triple fast blink
    LED_OTA_UPDATING,       // Very fast 50ms blink
    LED_WIFI_DISCONNECTED   // LED off
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
    bool turnOn = false;
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
            turnOn = true;
        }
    }
    portEXIT_CRITICAL(&_ledMux);
    if (turnOn) digitalWrite(LED_PIN, HIGH);  // GPIO outside spinlock
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
        case LED_WIFI_CONNECTING:
            // Fast 100ms blink
            if (elapsed >= 100) {
                _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledLastToggle = now;
            }
            break;
        case LED_WIFI_AP_MODE:
            // Double blink, no pause — continuous on/off/on/off
            if (elapsed >= 200) {
                _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledBlinkCount++; _ledLastToggle = now;
            }
            break;
        case LED_RUNNING:
            // Solid on
            if (!_ledOn) { _ledOn = true; digitalWrite(LED_PIN, HIGH); }
            break;
        case LED_ERROR:
            // Triple fast blink (6 toggles = 3 on/off cycles), then repeat
            if (_ledBlinkCount < 6) {
                if (elapsed >= 100) { _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledBlinkCount++; _ledLastToggle = now; }
            } else {
                if (elapsed >= 500) { _ledBlinkCount = 0; _ledLastToggle = now; }
            }
            break;
        case LED_OTA_UPDATING:
            if (elapsed >= 50) { _ledOn = !_ledOn; digitalWrite(LED_PIN, _ledOn); _ledLastToggle = now; }
            break;
        case LED_WIFI_DISCONNECTED:
            // LED off
            if (_ledOn) { _ledOn = false; digitalWrite(LED_PIN, LOW); }
            break;
    }
}
