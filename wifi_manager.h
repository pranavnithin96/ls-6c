#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <time.h>
#include "config.h"

// ============================================================================
// WiFi Manager — AP portal with PIN auth, non-blocking NTP, clean mDNS
// ============================================================================

static Preferences _prefs;
static WebServer _apServer(80);
static DNSServer _dnsServer;
static bool _apMode = false;
static unsigned long _lastReconnectAttempt = 0;
static int _reconnectBackoff = 1000;
static bool _mdnsStarted = false;
static String _apPIN = "";  // 4-digit PIN for AP portal security

// Config values
static String _deviceId, _locationName, _cfgServerUrl, _cfgTimezone, _wifiSSID, _wifiPassword;
static float _gridVoltage;
static int _ctRatings[6];
static int _sendInterval;

// RSSI tracking
#define RSSI_HISTORY_SIZE 60
static int _rssiHistory[RSSI_HISTORY_SIZE] = {0};
static int _rssiIdx = 0;
static int _rssiCount = 0;
static int _rssiMin = -30;
static int _rssiAvg = 0;

static String _scanResultsHTML = "";

void _scanNetworks() {
    int n = WiFi.scanNetworks();
    _scanResultsHTML = "";
    for (int i = 0; i < n && i < 15; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;
        int rssi = WiFi.RSSI(i);
        const char* lock = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN) ? " *" : "";
        // HTML-escape SSID to prevent injection
        ssid.replace("&", "&amp;");
        ssid.replace("<", "&lt;");
        ssid.replace(">", "&gt;");
        ssid.replace("\"", "&quot;");
        _scanResultsHTML += "<option value=\"" + ssid + "\">" + ssid + " (" + String(rssi) + "dBm" + lock + ")</option>";
    }
    WiFi.scanDelete();
}

// Generate PIN from MAC address (last 4 hex digits)
String generatePIN() {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char pin[5];
    snprintf(pin, sizeof(pin), "%02X%02X", mac[4], mac[5]);
    return String(pin);
}

void _handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LineSights Setup</title>
<style>body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:20px;background:#f5f5f5}
h2{color:#1565C0}input,select{width:100%;padding:10px;margin:5px 0 15px;border:1px solid #ddd;border-radius:6px;box-sizing:border-box}
button{width:100%;padding:12px;background:#1565C0;color:#fff;border:none;border-radius:6px;font-size:16px;cursor:pointer}
button:hover{background:#0D47A1}label{font-weight:bold;font-size:14px}
.refresh{background:#4CAF50;margin-bottom:15px;padding:8px;font-size:13px}
.pin{background:#fff3cd;padding:10px;border-radius:6px;margin-bottom:15px;border:1px solid #ffc107;font-size:13px}</style></head>
<body><h2>LineSights Power Monitor</h2>
<div class="pin">Enter the Setup PIN shown on the device serial console to save configuration.</div>
<button class="refresh" onclick="location.href='/scan'">Refresh WiFi Networks</button>
<form action="/save" method="POST">
<label>Setup PIN</label><input name="pin" type="text" maxlength="4" required placeholder="4-digit PIN from serial">
<label>WiFi Network</label><select name="ssid" required>
<option value="">-- Select Network --</option>
)rawliteral";
    html += _scanResultsHTML;
    html += R"rawliteral(
</select>
<label>WiFi Password</label><input name="pass" type="password" required minlength="8">
<label>Device ID</label><input name="devid" placeholder="powermon_001" required maxlength="32">
<label>Location</label><input name="loc" placeholder="Factory Floor" required maxlength="64">
<label>Server URL</label><input name="url" value="http://linesights.com/api/data">
<label>Grid Voltage (100-250V)</label><input name="volt" type="number" value="230" min="100" max="250">
<label>CT1 Rating (A)</label><select name="ct1"><option>50</option><option>100</option><option>150</option></select>
<label>CT2 Rating (A)</label><select name="ct2"><option>50</option><option>100</option><option>150</option></select>
<label>CT3 Rating (A)</label><select name="ct3"><option>50</option><option>100</option><option>150</option></select>
<label>CT4 Rating (A)</label><select name="ct4"><option>50</option><option>100</option><option>150</option></select>
<label>CT5 Rating (A)</label><select name="ct5"><option>50</option><option>100</option><option>150</option></select>
<label>CT6 Rating (A)</label><select name="ct6"><option>50</option><option>100</option><option>150</option></select>
<label>Send Interval (1-60s)</label><input name="int" type="number" value="1" min="1" max="60">
<label>Timezone</label><input name="tz" value="UTC">
<button type="submit">Save & Connect</button></form></body></html>
)rawliteral";
    _apServer.send(200, "text/html", html);
}

void _handleScan() {
    _scanNetworks();
    _apServer.sendHeader("Location", "/");
    _apServer.send(302, "text/plain", "Redirecting...");
}

void _handleSave() {
    // PIN authentication
    String pin = _apServer.arg("pin");
    if (pin != _apPIN) {
        _apServer.send(403, "text/html",
            "<html><body><h2>Invalid PIN</h2><p>Check the serial console for the correct PIN.</p>"
            "<p><a href='/'>Try again</a></p></body></html>");
        Serial.printf("[WiFi] Invalid PIN attempt: '%s' (expected '%s')\n", pin.c_str(), _apPIN.c_str());
        return;
    }

    // Validate inputs
    String ssid = _apServer.arg("ssid");
    String pass = _apServer.arg("pass");
    if (ssid.length() == 0 || ssid.length() > 32) {
        _apServer.send(400, "text/plain", "Invalid SSID");
        return;
    }
    if (pass.length() < 8 || pass.length() > 63) {
        _apServer.send(400, "text/plain", "Password must be 8-63 characters");
        return;
    }

    float volt = _apServer.arg("volt").toFloat();
    if (volt < 100 || volt > 250) volt = DEFAULT_GRID_VOLTAGE;

    int interval = _apServer.arg("int").toInt();
    if (interval < 1 || interval > 60) interval = DEFAULT_SEND_INTERVAL;

    String url = _apServer.arg("url");
    if (!url.startsWith("http://") && !url.startsWith("https://")) {
        url = "http://linesights.com/api/data";
    }

    _prefs.begin("lscfg", false);
    _prefs.putString("ssid", ssid);
    _prefs.putString("pass", pass);
    _prefs.putString("devid", _apServer.arg("devid"));
    _prefs.putString("loc", _apServer.arg("loc"));
    _prefs.putString("url", url);
    _prefs.putFloat("volt", volt);
    for (int i = 0; i < 6; i++) {
        char name[4]; snprintf(name, sizeof(name), "ct%d", i + 1);
        int rating = _apServer.arg(name).toInt();
        if (rating != 50 && rating != 100 && rating != 150) rating = 50;
        _prefs.putInt(name, rating);
    }
    _prefs.putInt("interval", interval);
    _prefs.putString("tz", _apServer.arg("tz"));
    _prefs.end();

    _apServer.send(200, "text/html",
        "<html><body style='font-family:sans-serif;text-align:center;padding:60px'>"
        "<h2>Configuration Saved!</h2><p>Device will restart and connect to your WiFi.</p></body></html>");
    delay(2000);
    ESP.restart();
}

bool _loadConfig() {
    _prefs.begin("lscfg", true);
    _wifiSSID = _prefs.getString("ssid", "");
    _wifiPassword = _prefs.getString("pass", "");
    _deviceId = _prefs.getString("devid", "");
    _locationName = _prefs.getString("loc", "");
    _cfgServerUrl = _prefs.getString("url", "http://linesights.com/api/data");
    _gridVoltage = _prefs.getFloat("volt", DEFAULT_GRID_VOLTAGE);
    for (int i = 0; i < 6; i++) {
        char key[4]; snprintf(key, sizeof(key), "ct%d", i + 1);
        _ctRatings[i] = _prefs.getInt(key, 30);
    }
    _sendInterval = _prefs.getInt("interval", DEFAULT_SEND_INTERVAL);
    _cfgTimezone = _prefs.getString("tz", "UTC");
    _prefs.end();
    return _wifiSSID.length() > 0;
}

void _startAPMode() {
    _apMode = true;
    _apPIN = generatePIN();

    WiFi.mode(WIFI_AP_STA);
    String apSSID = String(AP_SSID_PREFIX) + _apPIN;
    WiFi.softAP(apSSID.c_str());
    _scanNetworks();

    _dnsServer.start(53, "*", WiFi.softAPIP());

    Serial.printf("\n[WiFi] AP mode: %s at %s\n", apSSID.c_str(), WiFi.softAPIP().toString().c_str());
    Serial.printf("[WiFi] Setup PIN: %s\n", _apPIN.c_str());
    Serial.println("[WiFi] Open http://192.168.4.1 to configure");

    _apServer.on("/", _handleRoot);
    _apServer.on("/scan", _handleScan);
    _apServer.on("/save", HTTP_POST, _handleSave);
    _apServer.on("/generate_204", [](){ _apServer.sendHeader("Location", "/"); _apServer.send(302); });
    _apServer.on("/fwlink", [](){ _apServer.sendHeader("Location", "/"); _apServer.send(302); });
    _apServer.on("/hotspot-detect.html", [](){ _apServer.sendHeader("Location", "/"); _apServer.send(302); });
    _apServer.on("/library/test/success.html", [](){ _apServer.sendHeader("Location", "/"); _apServer.send(302); });
    _apServer.on("/connecttest.txt", [](){ _apServer.sendHeader("Location", "/"); _apServer.send(302); });
    _apServer.onNotFound([](){ _apServer.sendHeader("Location", "/"); _apServer.send(302); });
    _apServer.begin();
}

void _stopMDNS() {
    if (_mdnsStarted) {
        MDNS.end();
        _mdnsStarted = false;
    }
}

void _startMDNS() {
    if (_mdnsStarted) return;
    _stopMDNS();  // Ensure clean state
    delay(50);
    if (MDNS.begin(MDNS_HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        _mdnsStarted = true;
    }
}

void initWiFiManager() {
    if (!_loadConfig()) {
        Serial.println("[WiFi] No config — starting AP setup mode");
        _startAPMode();
        return;
    }

    Serial.printf("[WiFi] Connecting to '%s'...\n", _wifiSSID.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(_wifiSSID.c_str(), _wifiPassword.c_str());

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < (WIFI_CONNECT_TIMEOUT_MS / 500)) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected! IP: %s RSSI: %d dBm\n",
            WiFi.localIP().toString().c_str(), WiFi.RSSI());
        _startMDNS();
    } else {
        Serial.println("\n[WiFi] Connection failed — will retry in background");
    }
}

void updateRSSI() {
    int rssi = WiFi.RSSI();
    _rssiHistory[_rssiIdx] = rssi;
    _rssiIdx = (_rssiIdx + 1) % RSSI_HISTORY_SIZE;
    if (_rssiCount < RSSI_HISTORY_SIZE) _rssiCount++;

    int sum = 0;
    _rssiMin = 0;
    for (int i = 0; i < _rssiCount; i++) {
        sum += _rssiHistory[i];
        if (_rssiHistory[i] < _rssiMin) _rssiMin = _rssiHistory[i];
    }
    _rssiAvg = (_rssiCount > 0) ? sum / _rssiCount : rssi;
}

int getRSSIMin() { return _rssiMin; }
int getRSSIAvg() { return _rssiAvg; }

bool isWiFiConnected() {
    if (_apMode) {
        _dnsServer.processNextRequest();
        _apServer.handleClient();
        return false;
    }

    if (WiFi.status() != WL_CONNECTED) {
        unsigned long now = millis();
        if (now - _lastReconnectAttempt > (unsigned long)_reconnectBackoff) {
            Serial.println("[WiFi] Reconnecting...");
            WiFi.reconnect();
            _lastReconnectAttempt = now;
            // Exponential backoff with jitter to prevent thundering herd
            int jitter = random(0, _reconnectBackoff / 4);
            _reconnectBackoff = min(_reconnectBackoff * 2 + jitter, 5000);
        }
        _stopMDNS();
        return false;
    }

    _reconnectBackoff = 1000;
    if (!_mdnsStarted) _startMDNS();
    updateRSSI();
    return true;
}

bool isAPMode() { return _apMode; }
String getDeviceId() { return _deviceId; }
String getLocationName() { return _locationName; }
String getServerUrl() { return _cfgServerUrl; }
float getGridVoltage() { return _gridVoltage; }
int getCtRating(int channel = 0) { return _ctRatings[channel < 6 ? channel : 0]; }
int getSendInterval() { return _sendInterval; }
String getTimezone() { return _cfgTimezone; }

// Non-blocking NTP: configure servers, then poll with zero timeout
void syncNTP() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "UTC0", 1);
    tzset();

    // Only wait 100ms — don't block Core 1 sampling
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 100)) {
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &timeinfo);
        Serial.printf("[NTP] Synced: %s\n", buf);
    } else {
        Serial.println("[NTP] Pending — will sync in background");
    }
}

String getUTCTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 0)) return "1970-01-01T00:00:00.000Z";
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S.000Z", &timeinfo);
    return String(buf);
}
