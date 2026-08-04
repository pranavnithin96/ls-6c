#include "transport_service.h"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <errno.h>
#include <lwip/sockets.h>

namespace fw {

String TransportService::baseUrl(const String& liveUrl) const {
    const int apiData = liveUrl.indexOf("/api/data");
    if (apiData >= 0) return liveUrl.substring(0, apiData);
    const int scheme = liveUrl.indexOf("://");
    const int slash = liveUrl.indexOf('/', scheme >= 0 ? scheme + 3 : 0);
    return slash >= 0 ? liveUrl.substring(0, slash) : liveUrl;
}

void TransportService::begin(SettingsService& settings, ClockService& clock,
                             DurableQueue& queue) {
    settings_ = &settings;
    clock_ = &clock;
    queue_ = &queue;
    liveUrl_ = settings.config().serverUrl;
    const String root = baseUrl(liveUrl_);
    bulkUrl_ = root + "/api/data/bulk";
    heartbeatUrl_ = root + "/api/firmware/heartbeat";
    firmwareBaseUrl_ = root + "/api/firmware/";
    liveHttp_.setReuse(true);
    liveHttp_.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    liveHttp_.setTimeout(kHttpTimeoutMs);
}

size_t TransportService::serializeReading(const ElectricalReading& reading,
                                          char* output, size_t capacity) const {
    JsonDocument document;
    const DeviceConfig& config = settings_->config();
    document["device_id"] = config.deviceId;
    document["timestamp"] = clock_->formatUtc(reading.epoch);
    document["location"] = config.location;
    document["timezone"] = config.timezone;
    document["firmware_version"] = kFirmwareVersion;
    document["boot_id"] = reading.bootId;
    document["sequence"] = reading.sequence;
    if ((reading.flags & kFlagClockUnsynced) != 0 || reading.epoch == 0) {
        document["clock_unsynced"] = true;
    }
    JsonObject cts = document["readings"]["cts"].to<JsonObject>();
    const float voltage = reading.voltageDecivolts / 10.0f;
    for (uint8_t ch = 0; ch < kChannelCount; ++ch) {
        char key[8];
        snprintf(key, sizeof(key), "ct_%u", ch + 1);
        JsonObject ct = cts[key].to<JsonObject>();
        const float amps = reading.ampsCenti[ch] / 100.0f;
        ct["real_power_w"] = serialized(String(voltage * amps *
                                             (kDefaultPowerFactorMilli / 1000.0f), 1));
        ct["amps"] = serialized(String(amps, 2));
        ct["pf"] = serialized(String(kDefaultPowerFactorMilli / 1000.0f, 3));
    }
    document["readings"]["voltage_rms"] = serialized(String(voltage, 1));
    return serializeJson(document, output, capacity);
}

int TransportService::postLive(const ElectricalReading& reading) {
    char body[1024];
    const size_t length = serializeReading(reading, body, sizeof(body));
    if (length == 0 || length >= sizeof(body)) return -1000;
    for (uint8_t attempt = 0; attempt < 2; ++attempt) {
        if (!liveHttp_.begin(liveClient_, liveUrl_)) return -1001;
        liveHttp_.addHeader("Content-Type", "application/json");
        liveHttp_.addHeader("X-Reading-Id", String(reading.bootId) + ":" + String(reading.sequence));
        const int code = liveHttp_.POST(reinterpret_cast<uint8_t*>(body), length);
        liveHttp_.end();
        if (code > 0 || attempt == 1) return code;
        liveClient_.stop();
    }
    return -1002;
}

int TransportService::postFile(const String& path, const char* format, bool anchorValid) {
    File file = LittleFS.open(path.c_str(), "r");
    if (!file) return -1100;
    const size_t length = file.size();
    if (length == 0) {
        file.close();
        return -1101;
    }
    String headers = "Content-Type: ";
    headers += strcmp(format, "rejected-ndjson") == 0
                   ? "application/x-ndjson\r\n" : "application/octet-stream\r\n";
    headers += "X-Device-Id: " + settings_->config().deviceId + "\r\n";
    headers += "X-Format: " + String(format) + "\r\n";
    headers += "X-Interval: " + String(settings_->config().intervalSeconds) + "\r\n";
    headers += "X-Ntp-Epoch: " + String(clock_->nowEpoch()) + "\r\n";
    headers += "X-Ntp-Millis: " + String(millis()) + "\r\n";
    headers += String("X-Ntp-Valid: ") + (clock_->synced() ? "1\r\n" : "0\r\n");
    headers += String("X-Anchor-Valid: ") + (anchorValid ? "1\r\n" : "0\r\n");

    int code;
    if (bulkUrl_.startsWith("http://")) {
        code = postPlainHttpFile(file, length, headers);
    } else {
        // TLS remains available for configured HTTPS endpoints. The core's stream
        // sender is the fallback because the raw socket path below is plain HTTP.
        HTTPClient http;
        http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
        http.setTimeout(60000);
        if (!http.begin(bulkUrl_)) {
            file.close();
            return -1102;
        }
        int cursor = 0;
        while (cursor < static_cast<int>(headers.length())) {
            const int separator = headers.indexOf(": ", cursor);
            const int end = headers.indexOf("\r\n", separator + 2);
            if (separator < 0 || end < 0) break;
            http.addHeader(headers.substring(cursor, separator),
                           headers.substring(separator + 2, end));
            cursor = end + 2;
        }
        code = http.sendRequest("POST", &file, length);
        http.end();
    }
    file.close();
    return code;
}

bool TransportService::sendAll(int socket, const uint8_t* data, size_t length,
                               uint32_t startedMillis) {
    size_t offset = 0;
    uint32_t lastProgressMillis = millis();
    while (offset < length) {
        const uint32_t now = millis();
        if (now - startedMillis >= kBulkTimeoutMs ||
            now - lastProgressMillis >= kBulkStallMs) return false;
        fd_set writable;
        FD_ZERO(&writable);
        FD_SET(socket, &writable);
        timeval wait{0, 200000};
        const int selected = select(socket + 1, nullptr, &writable, nullptr, &wait);
        if (selected < 0) return false;
        if (selected == 0) continue;
        const int sent = send(socket, data + offset, length - offset, MSG_DONTWAIT);
        if (sent > 0) {
            offset += static_cast<size_t>(sent);
            lastProgressMillis = millis();
        } else if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        } else {
            return false;
        }
    }
    return true;
}

int TransportService::postPlainHttpFile(File& file, size_t length, const String& headers) {
    const String remainder = bulkUrl_.substring(7);
    const int slash = remainder.indexOf('/');
    String authority = slash >= 0 ? remainder.substring(0, slash) : remainder;
    const String path = slash >= 0 ? remainder.substring(slash) : "/";
    uint16_t port = 80;
    const int colon = authority.lastIndexOf(':');
    if (colon >= 0) {
        const int parsed = authority.substring(colon + 1).toInt();
        if (parsed < 1 || parsed > 65535) return -1110;
        port = static_cast<uint16_t>(parsed);
        authority = authority.substring(0, colon);
    }

    WiFiClient client;
    if (!client.connect(authority.c_str(), port, kHttpTimeoutMs)) return HTTPC_ERROR_CONNECTION_REFUSED;
    const int socket = client.fd();
    const uint32_t startedMillis = millis();
    String request = "POST " + path + " HTTP/1.1\r\nHost: " + authority +
                     "\r\nConnection: close\r\nContent-Length: " + String(length) +
                     "\r\n" + headers + "\r\n";
    if (!sendAll(socket, reinterpret_cast<const uint8_t*>(request.c_str()),
                 request.length(), startedMillis)) {
        client.stop();
        return HTTPC_ERROR_SEND_HEADER_FAILED;
    }

    uint8_t chunk[1024];
    size_t remaining = length;
    while (remaining > 0) {
        const size_t wanted = min(remaining, sizeof(chunk));
        const size_t read = file.read(chunk, wanted);
        if (read != wanted || !sendAll(socket, chunk, read, startedMillis)) {
            client.stop();
            return HTTPC_ERROR_SEND_PAYLOAD_FAILED;
        }
        remaining -= read;
    }

    char statusLine[64];
    size_t statusLength = 0;
    uint32_t lastResponseProgress = millis();
    while (true) {
        const uint32_t now = millis();
        if (now - startedMillis >= kBulkTimeoutMs ||
            now - lastResponseProgress >= kBulkResponseWaitMs) {
            client.stop();
            return HTTPC_ERROR_READ_TIMEOUT;
        }
        fd_set readable;
        FD_ZERO(&readable);
        FD_SET(socket, &readable);
        timeval wait{0, 200000};
        const int selected = select(socket + 1, &readable, nullptr, nullptr, &wait);
        if (selected < 0) {
            client.stop();
            return HTTPC_ERROR_READ_TIMEOUT;
        }
        if (selected == 0) continue;
        char value;
        const int received = recv(socket, &value, 1, MSG_DONTWAIT);
        if (received <= 0) {
            if (received < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            client.stop();
            return HTTPC_ERROR_READ_TIMEOUT;
        }
        lastResponseProgress = millis();
        if (value == '\n') break;
        if (statusLength + 1 < sizeof(statusLine) && value != '\r') {
            statusLine[statusLength++] = value;
        }
    }
    statusLine[statusLength] = '\0';
    client.stop();
    const char* separator = strchr(statusLine, ' ');
    const int code = separator ? atoi(separator + 1) : 0;
    return code > 0 ? code : HTTPC_ERROR_READ_TIMEOUT;
}

void TransportService::loop(bool connected) {
    if (!connected || queue_ == nullptr) return;
    const uint32_t now = millis();
    if (static_cast<int32_t>(now - retryAtMillis_) >= 0) {
        ElectricalReading reading;
        uint8_t sentThisTick = 0;
        while (sentThisTick < 3 && queue_->peekLive(reading)) {
            const int code = postLive(reading);
            const bool acknowledged = code == 200;
            queue_->recordDeliveryResult(code, acknowledged);
            if (!acknowledged) {
                retryAtMillis_ = millis() + retryDelayMs_;
                retryDelayMs_ = min(retryDelayMs_ * 2, kRetryMaxMs);
                break;
            }
            queue_->acknowledgeLive(reading);
            retryDelayMs_ = kRetryMinMs;
            retryAtMillis_ = 0;
            ++sentThisTick;
        }
    }

    queue_->checkpoint();
    if (now - lastBulkAttemptMillis_ < 60000UL) return;
    lastBulkAttemptMillis_ = now;

    String path;
    bool anchorValid = false;
    if (queue_->nextBulkUpload(path, anchorValid)) {
        const int code = postFile(path, "ls02-blocked", anchorValid);
        queue_->recordDeliveryResult(code, code == 200);
        if (code == 200) queue_->acknowledgeBulkUpload(path);
        return;
    }
    if (queue_->nextLegacyJsonUpload(path)) {
        const int code = postFile(path, "rejected-ndjson", false);
        queue_->recordDeliveryResult(code, code == 200);
        if (code == 200) queue_->acknowledgeLegacyJsonUpload(path);
    }
}

}  // namespace fw
