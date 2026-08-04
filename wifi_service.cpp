#include "wifi_service.h"

#include <WiFi.h>

namespace fw {

void WifiService::begin(SettingsService& settings) {
    settings_ = &settings;
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    if (!settings.isProvisioned()) startProvisioning();
    else startStation();
}

void WifiService::startStation() {
    provisioning_ = false;
    WiFi.mode(WIFI_STA);
    const DeviceConfig& config = settings_->config();
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    lastReconnectMillis_ = millis();
    Serial.printf("[WIFI] Connecting to %s\n", config.wifiSsid.c_str());
}

String WifiService::setupPin() const {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    char value[5];
    snprintf(value, sizeof(value), "%02X%02X", mac[4], mac[5]);
    return String(value);
}

void WifiService::scanNetworks() {
    scanOptions_ = "";
    const int count = WiFi.scanNetworks();
    for (int i = 0; i < count && i < 20; ++i) {
        String ssid = WiFi.SSID(i);
        ssid.replace("&", "&amp;");
        ssid.replace("<", "&lt;");
        ssid.replace(">", "&gt;");
        ssid.replace("\"", "&quot;");
        scanOptions_ += "<option value=\"" + ssid + "\">" + ssid + " (" +
                        String(WiFi.RSSI(i)) + " dBm)</option>";
    }
    WiFi.scanDelete();
}

void WifiService::startProvisioning() {
    provisioning_ = true;
    WiFi.mode(WIFI_AP_STA);
    const String pin = setupPin();
    const String ssid = "LineSights-" + pin;
    WiFi.softAP(ssid.c_str());
    scanNetworks();
    dns_.start(53, "*", WiFi.softAPIP());
    server_.on("/", HTTP_GET, [this]() { handleRoot(); });
    server_.on("/scan", HTTP_GET, [this]() {
        scanNetworks();
        server_.sendHeader("Location", "/");
        server_.send(302, "text/plain", "Redirecting");
    });
    server_.on("/save", HTTP_POST, [this]() { handleSave(); });
    server_.onNotFound([this]() {
        server_.sendHeader("Location", "/");
        server_.send(302, "text/plain", "Redirecting");
    });
    server_.begin();
    Serial.printf("[WIFI] Provisioning AP %s, PIN %s, http://192.168.4.1\n",
                  ssid.c_str(), pin.c_str());
}

void WifiService::handleRoot() {
    String html =
        "<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<style>body{font:16px sans-serif;max-width:520px;margin:30px auto;padding:16px}"
        "input,select,button{width:100%;padding:10px;margin:5px 0 14px;box-sizing:border-box}</style>"
        "<h2>LineSights setup</h2><p>Enter the PIN printed on serial.</p>"
        "<form action=/save method=post><label>PIN</label><input name=pin required maxlength=4>"
        "<label>Wi-Fi</label><select name=ssid required><option value=''>Choose network</option>";
    html += scanOptions_;
    html +=
        "</select><label>Password</label><input name=pass type=password maxlength=63>"
        "<label>Device ID</label><input name=devid required maxlength=32>"
        "<label>Location</label><input name=loc required maxlength=64>"
        "<label>Server URL</label><input name=url value='" + String(kDefaultServerUrl) + "'>"
        "<label>Grid voltage</label><input name=volt type=number min=100 max=250 value=230>";
    for (uint8_t ch = 1; ch <= kChannelCount; ++ch) {
        html += "<label>CT" + String(ch) + " rating</label><select name=ct" + String(ch) +
                "><option>50</option><option>100</option><option>150</option></select>";
    }
    html +=
        "<label>Interval (seconds)</label><input name=int type=number min=1 max=60 value=1>"
        "<label>Timezone label</label><input name=tz value=UTC>"
        "<button type=submit>Save and restart</button></form>";
    server_.send(200, "text/html", html);
}

void WifiService::handleSave() {
    if (server_.arg("pin") != setupPin()) {
        server_.send(403, "text/plain", "Invalid setup PIN");
        return;
    }
    uint16_t ratings[kChannelCount];
    for (uint8_t ch = 0; ch < kChannelCount; ++ch) {
        ratings[ch] = static_cast<uint16_t>(server_.arg("ct" + String(ch + 1)).toInt());
    }
    const bool saved = settings_->saveProvisioning(
        server_.arg("ssid"), server_.arg("pass"), server_.arg("devid"),
        server_.arg("loc"), server_.arg("url"), server_.arg("volt").toFloat(),
        ratings, server_.arg("int").toInt(), server_.arg("tz"));
    if (!saved) {
        server_.send(400, "text/plain", "Invalid configuration or NVS write failure");
        return;
    }
    server_.send(200, "text/html", "<h2>Saved</h2><p>Restarting…</p>");
    delay(750);
    ESP.restart();
}

void WifiService::loop() {
    if (provisioning_) {
        dns_.processNextRequest();
        server_.handleClient();
        return;
    }

    const bool nowConnected = connected();
    if (nowConnected) {
        reconnectDelayMs_ = kRetryMinMs;
        if (!wasConnected_) Serial.printf("[WIFI] Connected: %s\n", WiFi.localIP().toString().c_str());
        if (millis() - lastRssiSampleMillis_ >= 1000UL) {
            lastRssiSampleMillis_ = millis();
            const int value = WiFi.RSSI();
            rssiSum_ += value;
            ++rssiSamples_;
            if (minimumRssi_ == 0 || value < minimumRssi_) minimumRssi_ = value;
        }
    } else if (wasConnected_) {
        ++reconnects_;
        Serial.println("[WIFI] Link lost; durable capture continues");
    }
    wasConnected_ = nowConnected;

    if (!nowConnected && millis() - lastReconnectMillis_ >= reconnectDelayMs_) {
        WiFi.reconnect();
        lastReconnectMillis_ = millis();
        const uint32_t jitterBound = max(static_cast<uint32_t>(2), reconnectDelayMs_ / 4);
        reconnectDelayMs_ = min(reconnectDelayMs_ * 2 +
                                static_cast<uint32_t>(random(0, jitterBound)),
                                static_cast<uint32_t>(kRetryMaxMs));
    }
}

bool WifiService::connected() const { return WiFi.status() == WL_CONNECTED; }
int WifiService::rssi() const { return connected() ? WiFi.RSSI() : 0; }
int WifiService::averageRssi() const {
    return rssiSamples_ == 0 ? 0 : static_cast<int>(rssiSum_ / rssiSamples_);
}

}  // namespace fw
