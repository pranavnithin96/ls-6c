#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>

static Preferences _prefs;
static WebServer _apServer(80);
static bool _apMode = false;
static unsigned long _lastReconnectAttempt = 0;
static int _reconnectBackoff = 1000;

// Default config
#define DEFAULT_SERVER_URL "http://77.42.75.92/api/data"
#define DEFAULT_GRID_VOLTAGE 230.0f
#define DEFAULT_CT_RATING 30
#define DEFAULT_SEND_INTERVAL 1
#define DEFAULT_TIMEZONE "Asia/Kolkata"
#define AP_SSID "LineSights-Setup"

static String _deviceId, _locationName, _cfgServerUrl, _cfgTimezone, _wifiSSID, _wifiPassword;
static float _gridVoltage;
static int _ctRatings[6];
static int _sendInterval;

// Config page HTML
static const char CONFIG_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>LineSights Setup</title>
<style>body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:20px;background:#f5f5f5}
h2{color:#1565C0}input,select{width:100%;padding:10px;margin:5px 0 15px;border:1px solid #ddd;border-radius:6px;box-sizing:border-box}
button{width:100%;padding:12px;background:#1565C0;color:#fff;border:none;border-radius:6px;font-size:16px;cursor:pointer}
button:hover{background:#0D47A1}label{font-weight:bold;font-size:14px}</style></head>
<body><h2>LineSights Power Monitor</h2><form action="/save" method="POST">
<label>WiFi Network</label><input name="ssid" required>
<label>WiFi Password</label><input name="pass" type="password" required>
<label>Device ID</label><input name="devid" placeholder="powermon_001" required>
<label>Location</label><input name="loc" placeholder="Factory Floor" required>
<label>Server URL</label><input name="url" value="http://77.42.75.92/api/data">
<label>Grid Voltage</label><input name="volt" type="number" value="230">
<label>CT1 Rating (A)</label><select name="ct1"><option>50</option><option>100</option><option>150</option></select>
<label>CT2 Rating (A)</label><select name="ct2"><option>50</option><option>100</option><option>150</option></select>
<label>CT3 Rating (A)</label><select name="ct3"><option>50</option><option>100</option><option>150</option></select>
<label>CT4 Rating (A)</label><select name="ct4"><option>50</option><option>100</option><option>150</option></select>
<label>CT5 Rating (A)</label><select name="ct5"><option>50</option><option>100</option><option>150</option></select>
<label>CT6 Rating (A)</label><select name="ct6"><option>50</option><option>100</option><option>150</option></select>
<label>Send Interval (s)</label><input name="int" type="number" value="1" min="1" max="60">
<label>Timezone</label><input name="tz" value="Asia/Kolkata">
<button type="submit">Save & Connect</button></form></body></html>
)rawliteral";

static const char SAVED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Saved</title><style>body{font-family:sans-serif;text-align:center;padding:60px}</style></head>
<body><h2>Configuration Saved!</h2><p>Device will restart and connect to your WiFi network.</p></body></html>
)rawliteral";

void _handleRoot() { _apServer.send(200, "text/html", CONFIG_HTML); }

void _handleSave() {
    // Validate inputs (matches piimage_3 validation)
    float volt = _apServer.arg("volt").toFloat();
    if (volt < 100 || volt > 250) volt = 230.0f;

    int interval = _apServer.arg("int").toInt();
    if (interval < 1 || interval > 60) interval = 1;

    String url = _apServer.arg("url");
    if (!url.startsWith("http://") && !url.startsWith("https://")) {
        url = DEFAULT_SERVER_URL;
    }

    _prefs.begin("lscfg", false);
    _prefs.putString("ssid", _apServer.arg("ssid"));
    _prefs.putString("pass", _apServer.arg("pass"));
    _prefs.putString("devid", _apServer.arg("devid"));
    _prefs.putString("loc", _apServer.arg("loc"));
    _prefs.putString("url", url);
    _prefs.putFloat("volt", volt);
    for (int i = 0; i < 6; i++) {
        char name[4];
        snprintf(name, sizeof(name), "ct%d", i + 1);
        int rating = _apServer.arg(name).toInt();
        if (rating != 50 && rating != 100 && rating != 150) rating = 50;
        _prefs.putInt(name, rating);
    }
    _prefs.putInt("interval", interval);
    _prefs.putString("tz", _apServer.arg("tz"));
    _prefs.end();

    _apServer.send(200, "text/html", SAVED_HTML);
    delay(2000);
    ESP.restart();
}

bool _loadConfig() {
    _prefs.begin("lscfg", true);
    _wifiSSID = _prefs.getString("ssid", "");
    _wifiPassword = _prefs.getString("pass", "");
    _deviceId = _prefs.getString("devid", "");
    _locationName = _prefs.getString("loc", "");
    _cfgServerUrl = _prefs.getString("url", DEFAULT_SERVER_URL);
    _gridVoltage = _prefs.getFloat("volt", DEFAULT_GRID_VOLTAGE);
    for (int i = 0; i < 6; i++) {
        char key[4];
        snprintf(key, sizeof(key), "ct%d", i + 1);
        _ctRatings[i] = _prefs.getInt(key, DEFAULT_CT_RATING);
    }
    _sendInterval = _prefs.getInt("interval", DEFAULT_SEND_INTERVAL);
    _cfgTimezone = _prefs.getString("tz", DEFAULT_TIMEZONE);
    _prefs.end();
    return _wifiSSID.length() > 0;
}

void _startAPMode() {
    _apMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID);
    Serial.printf("[WiFi] AP mode started: %s at %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
    _apServer.on("/", _handleRoot);
    _apServer.on("/save", HTTP_POST, _handleSave);
    _apServer.begin();
}

void initWiFiManager() {
    if (!_loadConfig()) {
        Serial.println("[WiFi] No config found, starting AP mode for setup");
        _startAPMode();
        return;
    }

    Serial.printf("[WiFi] Connecting to %s...\n", _wifiSSID.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(_wifiSSID.c_str(), _wifiPassword.c_str());

    // Wait up to 15 seconds
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected! IP: %s, RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("\n[WiFi] Connection failed, will retry in background");
    }
}

bool isWiFiConnected() {
    if (_apMode) {
        _apServer.handleClient();
        return false;
    }

    if (WiFi.status() != WL_CONNECTED) {
        unsigned long now = millis();
        if (now - _lastReconnectAttempt > (unsigned long)_reconnectBackoff) {
            Serial.println("[WiFi] Reconnecting...");
            WiFi.reconnect();
            _lastReconnectAttempt = now;
            _reconnectBackoff = min(_reconnectBackoff * 2, 30000);
        }
        return false;
    }

    _reconnectBackoff = 1000;
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

void syncNTP() {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    // Set timezone
    setenv("TZ", _cfgTimezone.c_str(), 1);
    tzset();

    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10000)) {
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);
        Serial.printf("[NTP] Time synced: %s\n", buf);
    } else {
        Serial.println("[NTP] Time sync failed, will retry later");
        // logError called from heartbeat.h if included
    }
}

String getUTCTimestamp() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "1970-01-01T00:00:00.000Z";
    time_t now = mktime(&timeinfo);
    struct tm utc;
    gmtime_r(&now, &utc);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &utc);
    return String(buf) + ".000Z";
}
