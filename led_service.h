#pragma once

#include <Arduino.h>

namespace fw {

enum class LedState : uint8_t { Booting, Provisioning, Connecting, Running, StorageFault, Updating };

class LedService {
public:
    void begin();
    void set(LedState state);
    void loop();

private:
    LedState state_ = LedState::Booting;
    uint32_t changedAt_ = 0;
    bool on_ = false;
};

}  // namespace fw
