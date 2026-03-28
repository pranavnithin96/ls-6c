#pragma once
#include <Arduino.h>

#define SIGNAL_LED_PIN 23

enum LEDState {
    LED_BOOTING,           // Fast blink (100ms)
    LED_WIFI_CONNECTING,   // Slow blink (500ms)
    LED_WIFI_AP_MODE,      // Double blink, pause
    LED_RUNNING,           // Solid on
    LED_SENDING,           // Brief off-pulse every 2s
    LED_ERROR,             // Triple fast blink, pause
    LED_OTA_UPDATING,      // Very fast blink (50ms)
    LED_WIFI_DISCONNECTED  // Long blink (1s on/off)
};

static LEDState _ledState = LED_BOOTING;
static unsigned long _ledLastToggle = 0;
static bool _ledOn = false;
static int _ledBlinkCount = 0;

void initLED() {
    pinMode(SIGNAL_LED_PIN, OUTPUT);
    digitalWrite(SIGNAL_LED_PIN, LOW);
    _ledState = LED_BOOTING;
    _ledLastToggle = millis();
}

void setLEDState(LEDState state) {
    _ledState = state;
    _ledBlinkCount = 0;
    _ledLastToggle = millis();
    if (state == LED_RUNNING) {
        digitalWrite(SIGNAL_LED_PIN, HIGH);
        _ledOn = true;
    }
}

LEDState getLEDState() {
    return _ledState;
}

void updateLED() {
    unsigned long now = millis();
    unsigned long elapsed = now - _ledLastToggle;

    switch (_ledState) {
        case LED_BOOTING:
            if (elapsed >= 100) {
                _ledOn = !_ledOn;
                digitalWrite(SIGNAL_LED_PIN, _ledOn ? HIGH : LOW);
                _ledLastToggle = now;
            }
            break;

        case LED_WIFI_CONNECTING:
            if (elapsed >= 500) {
                _ledOn = !_ledOn;
                digitalWrite(SIGNAL_LED_PIN, _ledOn ? HIGH : LOW);
                _ledLastToggle = now;
            }
            break;

        case LED_WIFI_AP_MODE:
            // Double blink: on-off-on-off-pause
            if (_ledBlinkCount < 4) {
                if (elapsed >= 150) {
                    _ledOn = !_ledOn;
                    digitalWrite(SIGNAL_LED_PIN, _ledOn ? HIGH : LOW);
                    _ledLastToggle = now;
                    _ledBlinkCount++;
                }
            } else {
                if (elapsed >= 800) {
                    _ledBlinkCount = 0;
                    _ledLastToggle = now;
                }
            }
            break;

        case LED_RUNNING:
            // Solid on — no toggling needed
            if (!_ledOn) {
                digitalWrite(SIGNAL_LED_PIN, HIGH);
                _ledOn = true;
            }
            break;

        case LED_SENDING:
            // Brief off-pulse every 2 seconds
            if (_ledOn && elapsed >= 2000) {
                digitalWrite(SIGNAL_LED_PIN, LOW);
                _ledOn = false;
                _ledLastToggle = now;
            } else if (!_ledOn && elapsed >= 50) {
                digitalWrite(SIGNAL_LED_PIN, HIGH);
                _ledOn = true;
                _ledLastToggle = now;
            }
            break;

        case LED_ERROR:
            // Triple fast blink then pause
            if (_ledBlinkCount < 6) {
                if (elapsed >= 100) {
                    _ledOn = !_ledOn;
                    digitalWrite(SIGNAL_LED_PIN, _ledOn ? HIGH : LOW);
                    _ledLastToggle = now;
                    _ledBlinkCount++;
                }
            } else {
                if (elapsed >= 1000) {
                    _ledBlinkCount = 0;
                    _ledLastToggle = now;
                }
            }
            break;

        case LED_OTA_UPDATING:
            if (elapsed >= 50) {
                _ledOn = !_ledOn;
                digitalWrite(SIGNAL_LED_PIN, _ledOn ? HIGH : LOW);
                _ledLastToggle = now;
            }
            break;

        case LED_WIFI_DISCONNECTED:
            if (elapsed >= 1000) {
                _ledOn = !_ledOn;
                digitalWrite(SIGNAL_LED_PIN, _ledOn ? HIGH : LOW);
                _ledLastToggle = now;
            }
            break;
    }
}
