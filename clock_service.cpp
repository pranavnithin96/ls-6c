#include "clock_service.h"

#include <esp_sntp.h>
#include <sys/time.h>
#include <time.h>

namespace fw {
namespace {
volatile uint32_t gLastSyncMillis = 0;
void onTimeSync(struct timeval*) { gLastSyncMillis = millis(); }
}  // namespace

void ClockService::begin() {
    sntp_set_time_sync_notification_cb(onTimeSync);
    sntp_set_sync_interval(15 * 60 * 1000UL);
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "UTC0", 1);
    tzset();
    lastKickMillis_ = millis();
}

void ClockService::loop(bool networkConnected) {
    if (!networkConnected) return;
    const uint32_t now = millis();
    const uint32_t retryMs = quickRetries_ < 5 ? 30000UL : 15 * 60 * 1000UL;
    if (!synced() && now - lastKickMillis_ >= retryMs) {
        begin();
        lastKickMillis_ = now;
        ++quickRetries_;
    } else if (synced() && syncAgeSeconds() > 2 * 60 * 60 &&
               now - lastKickMillis_ >= 60000UL) {
        begin();
        lastKickMillis_ = now;
    }
}

bool ClockService::synced() const {
    time_t now = time(nullptr);
    return now > 1700000000;
}

uint32_t ClockService::nowEpoch() const {
    return synced() ? static_cast<uint32_t>(time(nullptr)) : 0;
}

uint32_t ClockService::epochAt(uint32_t sampleMillis) const {
    if (!synced()) return 0;
    timeval tv{};
    gettimeofday(&tv, nullptr);
    const int64_t epochMs = static_cast<int64_t>(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
    const uint32_t elapsed = millis() - sampleMillis;
    const int64_t sampleEpochMs = epochMs - elapsed;
    return sampleEpochMs > 0 ? static_cast<uint32_t>(sampleEpochMs / 1000) : 0;
}

long ClockService::syncAgeSeconds() const {
    return gLastSyncMillis == 0 ? -1 : static_cast<long>((millis() - gLastSyncMillis) / 1000);
}

String ClockService::formatUtc(uint32_t epoch) const {
    if (epoch == 0) return "1970-01-01T00:00:00.000Z";
    time_t value = epoch;
    tm utc{};
    gmtime_r(&value, &utc);
    char out[32];
    strftime(out, sizeof(out), "%Y-%m-%dT%H:%M:%S.000Z", &utc);
    return String(out);
}

}  // namespace fw
