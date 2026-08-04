#pragma once

#include <Arduino.h>

#include "app_types.h"
#include "durable_queue.h"

namespace fw {

class OtaService {
public:
    void begin(const String& firmwareBaseUrl, const String& deviceId,
               DurableQueue& queue, ControlFlags& controls);
    void loop(bool connected, bool systemHealthy);
    bool updating() const { return updating_; }

private:
    bool checkAndInstall();
    bool newer(const String& candidate, const String& current) const;
    void validateRunningImage(bool systemHealthy);

    String baseUrl_;
    String deviceId_;
    DurableQueue* queue_ = nullptr;
    ControlFlags* controls_ = nullptr;
    volatile bool updating_ = false;
    bool pendingValidation_ = false;
    uint32_t bootMillis_ = 0;
    uint32_t lastCheckMillis_ = 0;
};

}  // namespace fw
