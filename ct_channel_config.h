#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// Applied configuration is read by Core 1 at acquisition boundaries. Remote
// commands run on Core 0 and only stage a pending value; Core 1 atomically
// promotes and persists it between frames so a frame can never mix masks.
static portMUX_TYPE _ctConfigMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint8_t  _ctActiveMask = 0;
static volatile uint32_t _ctConfigRevision = 0;
static volatile bool     _ctConfigPresent = false;
static volatile bool     _ctConfigPending = false;
static volatile uint8_t  _ctPendingMask = 0;
static volatile uint32_t _ctPendingRevision = 0;

void initCTChannelConfig() {
    Preferences p;
    p.begin("lscfg", true);
    bool present = p.getBool("ctcfg", false);
    uint8_t mask = p.getUChar("ctmask", 0) & 0x3f;
    uint32_t revision = p.getUInt("ctrev", 0);
    p.end();

    portENTER_CRITICAL(&_ctConfigMux);
    _ctConfigPresent = present;
    _ctActiveMask = present ? mask : 0;
    _ctConfigRevision = present ? revision : 0;
    portEXIT_CRITICAL(&_ctConfigMux);

    if (present)
        Serial.printf("[CTCFG] applied rev=%u mask=0x%02X\n", (unsigned)revision, mask);
    else
        Serial.println("[CTCFG] configuration required - no CT channels active");
}

uint8_t getActiveCTMask() {
    portENTER_CRITICAL(&_ctConfigMux);
    uint8_t mask = _ctActiveMask;
    portEXIT_CRITICAL(&_ctConfigMux);
    return mask;
}

uint32_t getCTConfigRevision() {
    portENTER_CRITICAL(&_ctConfigMux);
    uint32_t revision = _ctConfigRevision;
    portEXIT_CRITICAL(&_ctConfigMux);
    return revision;
}

bool isCTChannelEnabled(int ch) {
    return ch >= 0 && ch < NUM_CT_CHANNELS && (getActiveCTMask() & (1u << ch));
}

bool isCTConfigRequired() {
    portENTER_CRITICAL(&_ctConfigMux);
    bool required = !_ctConfigPresent;
    portEXIT_CRITICAL(&_ctConfigMux);
    return required;
}

bool isCTConfigPending() {
    portENTER_CRITICAL(&_ctConfigMux);
    bool pending = _ctConfigPending;
    portEXIT_CRITICAL(&_ctConfigMux);
    return pending;
}

// revision must move forward. Re-delivery of the already-applied pair is an
// idempotent success so command acknowledgement remains safe.
bool stageCTChannelConfig(uint8_t mask, uint32_t revision) {
    if (mask & 0xc0u || revision == 0) return false;

    portENTER_CRITICAL(&_ctConfigMux);
    uint32_t currentRevision = _ctConfigRevision;
    uint8_t currentMask = _ctActiveMask;
    bool pending = _ctConfigPending;
    uint32_t pendingRevision = _ctPendingRevision;
    uint8_t pendingMask = _ctPendingMask;
    bool accepted = false;
    if (revision == currentRevision && mask == currentMask) {
        accepted = true;
    } else if (pending && revision == pendingRevision && mask == pendingMask) {
        accepted = true;
    } else if (revision > currentRevision && (!pending || revision >= pendingRevision)) {
        _ctPendingMask = mask;
        _ctPendingRevision = revision;
        _ctConfigPending = true;
        accepted = true;
    }
    portEXIT_CRITICAL(&_ctConfigMux);

    if (accepted)
        Serial.printf("[CTCFG] staged rev=%u mask=0x%02X\n", (unsigned)revision, mask);
    return accepted;
}

// Core 1 only, immediately before starting a new acquisition frame.
uint8_t applyPendingCTConfigAtFrameBoundary(uint32_t* revisionOut) {
    bool changed = false;
    uint8_t mask;
    uint32_t revision;

    portENTER_CRITICAL(&_ctConfigMux);
    if (_ctConfigPending) {
        _ctActiveMask = _ctPendingMask;
        _ctConfigRevision = _ctPendingRevision;
        _ctConfigPresent = true;
        _ctConfigPending = false;
        changed = true;
    }
    mask = _ctActiveMask;
    revision = _ctConfigRevision;
    portEXIT_CRITICAL(&_ctConfigMux);

    if (changed) {
        Preferences p;
        p.begin("lscfg", false);
        p.putUChar("ctmask", mask);
        p.putUInt("ctrev", revision);
        p.putBool("ctcfg", true);
        p.end();
        Serial.printf("[CTCFG] applied rev=%u mask=0x%02X at frame boundary\n",
                      (unsigned)revision, mask);
    }
    if (revisionOut) *revisionOut = revision;
    return mask;
}
