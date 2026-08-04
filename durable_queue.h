#pragma once

#include <Arduino.h>
#include "app_types.h"

namespace fw {

enum class StorageState : uint8_t { Uninitialized, Ready, CapacityBlocked, Fault };

class DurableQueue {
public:
    bool begin(const String& deviceId);
    bool recover();
    bool commit(const ElectricalReading& reading);
    bool peekLive(ElectricalReading& reading);
    void acknowledgeLive(const ElectricalReading& reading);
    void recordDeliveryResult(int httpCode, bool success);
    void recordCaptureGap(uint32_t intervals);
    bool checkpoint();

    bool nextBulkUpload(String& path, bool& anchorValid);
    bool acknowledgeBulkUpload(const String& path);
    bool nextLegacyJsonUpload(String& path);
    bool acknowledgeLegacyJsonUpload(const String& path);

    StorageState state() const { return state_; }
    DeliveryStats stats() const;
    uint32_t walDepth() const { return walRecordCount_; }
    uint32_t walBytes() const;
    uint32_t offlineBytes() const;
    uint32_t legacyJsonBytes() const;
    uint32_t quarantineBytes() const;
    uint32_t freeBytes() const;
    bool filesystemReady() const { return fsReady_; }

private:
    struct LiveEntry {
        ElectricalReading reading;
        bool acknowledged = false;
    };

    bool recoverCheckpointTransaction();
    bool recoverOfflineRepair();
    bool quarantineInvalidWalTail();
    bool appendWal(const ElectricalReading& reading);
    bool checkpointLocked();
    bool appendOfflineBlock(const ElectricalReading* readings, size_t count);
    bool ensureOfflineHeader();
    bool rotateActiveOffline(String& destination);
    bool rewriteWalWithoutPrefix(size_t prefixRecords);
    bool restoreOfflinePrefix(size_t verifiedBytes, const char* reason);
    void reloadLiveLocked();
    bool readingAcknowledgedLocked(const ElectricalReading& reading) const;
    String firstBulkPathLocked(bool includeActive) const;
    String allocateLegacyPathLocked() const;
    uint32_t fileSize(const char* path) const;

    String deviceId_;
    SemaphoreHandle_t mutex_ = nullptr;
    LiveEntry live_[kLiveQueueCapacity];
    size_t liveCount_ = 0;
    bool fsReady_ = false;
    volatile uint32_t walRecordCount_ = 0;
    volatile StorageState state_ = StorageState::Uninitialized;
    DeliveryStats stats_;
    String sameSessionUploadPath_;
};

}  // namespace fw
