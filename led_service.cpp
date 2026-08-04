#include "led_service.h"

#include "config.h"

namespace fw {

void LedService::begin() {
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, LOW);
    changedAt_ = millis();
}

void LedService::set(LedState state) {
    if (state_ == state) return;
    state_ = state;
    changedAt_ = millis();
    on_ = false;
}

void LedService::loop() {
    const uint32_t elapsed = millis() - changedAt_;
    bool desired = false;
    switch (state_) {
        case LedState::Running: desired = true; break;
        case LedState::Booting:
        case LedState::Connecting: desired = ((elapsed / 150) % 2) == 0; break;
        case LedState::Provisioning: desired = (elapsed % 1200) < 150 ||
                                                ((elapsed + 300) % 1200) < 150; break;
        case LedState::StorageFault: desired = (elapsed % 1200) < 500; break;
        case LedState::Updating: desired = ((elapsed / 50) % 2) == 0; break;
    }
    if (desired != on_) {
        on_ = desired;
        digitalWrite(kLedPin, on_ ? HIGH : LOW);
    }
}

}  // namespace fw
