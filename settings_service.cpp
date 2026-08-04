#include "settings_service.h"

#include <Preferences.h>

namespace fw {
namespace {

uint16_t validRating(int value) {
    return (value == 50 || value == 100 || value == 150)
               ? static_cast<uint16_t>(value)
               : 50;
}

String validUrl(const String& value) {
    if (value.startsWith("http://") || value.startsWith("https://")) return value;
    return String(kDefaultServerUrl);
}

}  // namespace

bool SettingsService::begin() {
    Preferences prefs;
    if (!prefs.begin("lscfg", true)) return false;
    config_.wifiSsid = prefs.getString("ssid", "");
    config_.wifiPassword = prefs.getString("pass", "");
    config_.deviceId = prefs.getString("devid", "");
    config_.location = prefs.getString("loc", "");
    config_.serverUrl = validUrl(prefs.getString("url", kDefaultServerUrl));
    config_.timezone = prefs.getString("tz", "UTC");
    const float voltage = prefs.getFloat("volt", kDefaultVoltageDecivolts / 10.0f);
    config_.voltageDecivolts = static_cast<uint16_t>(
        constrain(voltage, 100.0f, 250.0f) * 10.0f + 0.5f);
    config_.intervalSeconds = static_cast<uint8_t>(
        constrain(prefs.getInt("interval", kDefaultIntervalSeconds), 1, 60));
    for (uint8_t ch = 0; ch < kChannelCount; ++ch) {
        char key[4];
        snprintf(key, sizeof(key), "ct%u", ch + 1);
        config_.ctRating[ch] = validRating(prefs.getInt(key, 50));
    }
    prefs.end();

    Preferences calibration;
    if (calibration.begin("ctcal", true)) {
        for (uint8_t ch = 0; ch < kChannelCount; ++ch) {
            char key[4];
            snprintf(key, sizeof(key), "s%u", ch);
            const float slope = calibration.getFloat(key, 0.0f);
            config_.ctSlope[ch] =
                (slope >= 0.001f && slope <= 0.5f) ? slope : 0.0f;
        }
        calibration.end();
    }

    provisioned_ = !config_.wifiSsid.isEmpty() && !config_.deviceId.isEmpty();

    Preferences state;
    if (!state.begin("lsv3", false)) return false;
    bootId_ = state.getUInt("boot_id", 0) + 1;
    if (bootId_ == 0) bootId_ = 1;
    const bool bootStored = state.putUInt("boot_id", bootId_) == sizeof(uint32_t);
    state.end();
    return bootStored;
}

bool SettingsService::saveProvisioning(
        const String& ssid, const String& password, const String& deviceId,
        const String& location, const String& serverUrl, float voltage,
        const uint16_t ratings[kChannelCount], int intervalSeconds,
        const String& timezone) {
    if (ssid.isEmpty() || ssid.length() > 32 || password.length() > 63 ||
        deviceId.isEmpty() || deviceId.length() > 32 || location.length() > 64 ||
        voltage < 100.0f || voltage > 250.0f || intervalSeconds < 1 ||
        intervalSeconds > 60) {
        return false;
    }

    Preferences prefs;
    if (!prefs.begin("lscfg", false)) return false;
    bool ok = true;
    ok &= prefs.putString("ssid", ssid) == ssid.length();
    ok &= prefs.putString("pass", password) == password.length();
    ok &= prefs.putString("devid", deviceId) == deviceId.length();
    ok &= prefs.putString("loc", location) == location.length();
    const String checkedUrl = validUrl(serverUrl);
    ok &= prefs.putString("url", checkedUrl) == checkedUrl.length();
    ok &= prefs.putFloat("volt", voltage) == sizeof(float);
    ok &= prefs.putInt("interval", intervalSeconds) == sizeof(int32_t);
    ok &= prefs.putString("tz", timezone.isEmpty() ? "UTC" : timezone) > 0;
    for (uint8_t ch = 0; ch < kChannelCount; ++ch) {
        char key[4];
        snprintf(key, sizeof(key), "ct%u", ch + 1);
        ok &= prefs.putInt(key, validRating(ratings[ch])) == sizeof(int32_t);
    }
    prefs.end();
    return ok;
}

bool SettingsService::setInterval(int seconds) {
    if (seconds < 1 || seconds > 60) return false;
    Preferences prefs;
    if (!prefs.begin("lscfg", false)) return false;
    const bool ok = prefs.putInt("interval", seconds) == sizeof(int32_t);
    prefs.end();
    if (ok) config_.intervalSeconds = static_cast<uint8_t>(seconds);
    return ok;
}

bool SettingsService::setVoltage(float volts) {
    if (volts < 100.0f || volts > 250.0f) return false;
    Preferences prefs;
    if (!prefs.begin("lscfg", false)) return false;
    const bool ok = prefs.putFloat("volt", volts) == sizeof(float);
    prefs.end();
    if (ok) config_.voltageDecivolts = static_cast<uint16_t>(volts * 10.0f + 0.5f);
    return ok;
}

bool SettingsService::setChannelSlope(uint8_t channel, float ampsPerCount) {
    if (channel >= kChannelCount ||
        (ampsPerCount != 0.0f && (ampsPerCount < 0.001f || ampsPerCount > 0.5f))) {
        return false;
    }
    Preferences prefs;
    if (!prefs.begin("ctcal", false)) return false;
    char key[4];
    snprintf(key, sizeof(key), "s%u", channel);
    const bool ok = prefs.putFloat(key, ampsPerCount) == sizeof(float);
    prefs.end();
    if (ok) config_.ctSlope[channel] = ampsPerCount;
    return ok;
}

}  // namespace fw
