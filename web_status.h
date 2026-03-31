#pragma once
#include <Arduino.h>
#include "ct_sensor.h"

// ============================================================================
// Web Status — DISABLED in station mode to save sockets
// Only updateLastReadings() is kept for internal state sharing
// ============================================================================

static AllCTReadings _lastReadings = {};
static SemaphoreHandle_t _readingsMutex = NULL;

void initReadingsMutex() {
    if (_readingsMutex == NULL) {
        _readingsMutex = xSemaphoreCreateMutex();
    }
}

void updateLastReadings(const AllCTReadings& readings) {
    if (_readingsMutex && xSemaphoreTake(_readingsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        _lastReadings = readings;
        xSemaphoreGive(_readingsMutex);
    }
}

AllCTReadings getLastReadings() {
    AllCTReadings copy = {};
    if (_readingsMutex && xSemaphoreTake(_readingsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        copy = _lastReadings;
        xSemaphoreGive(_readingsMutex);
    }
    return copy;
}

// Stubs — web server removed from station mode to prevent socket exhaustion
void initStatusServer() {
    Serial.println("[WEB] Status server disabled in station mode (saves sockets)");
}

void handleStatusServer() {
    // No-op: web server only runs in AP mode (via wifi_manager.h)
}
