#include "durable_queue.h"

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <esp32/rom/miniz.h>

#include "config.h"

namespace fw {
namespace {

constexpr char kWalNextFile[] = "/v3.wal.next";
constexpr char kLegacyBufferNdjson[] = "/buffer.ndjson";
constexpr char kLegacyBufferNdjsonTmp[] = "/buffer.ndjson.tmp";
constexpr char kOfflineRepairFile[] = "/offline.repair";
constexpr uint16_t kOfflineRawFlag = 0x8000;

struct __attribute__((packed)) WalRecord {
    uint32_t magic;
    uint32_t bootId;
    uint32_t sequence;
    uint32_t epoch;
    uint32_t sampleMillis;
    uint16_t voltageDecivolts;
    uint16_t sampleDurationMs;
    uint8_t intervalSeconds;
    uint8_t flags;
    int16_t ampsCenti[kChannelCount];
    uint32_t crc32;
};

struct __attribute__((packed)) OfflineHeader {
    char magic[4];
    char deviceId[32];
    uint32_t fileEpoch;
};

struct __attribute__((packed)) OfflineBlockHeader {
    uint32_t startEpoch;
    uint32_t startMillis;
    uint8_t flags;
    uint16_t sizeFlags;
    uint16_t crc16;
};

static_assert(sizeof(OfflineHeader) == 40, "LS02 header layout changed");
static_assert(sizeof(OfflineBlockHeader) == 13, "LS02 block layout changed");
static_assert(sizeof(WalRecord) == 42, "WAL layout changed");

uint32_t crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
        }
    }
    return ~crc;
}

uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

WalRecord encode(const ElectricalReading& reading) {
    WalRecord record{};
    record.magic = kWalMagic;
    record.bootId = reading.bootId;
    record.sequence = reading.sequence;
    record.epoch = reading.epoch;
    record.sampleMillis = reading.sampleMillis;
    record.voltageDecivolts = reading.voltageDecivolts;
    record.sampleDurationMs = reading.sampleDurationMs;
    record.intervalSeconds = reading.intervalSeconds;
    record.flags = reading.flags;
    memcpy(record.ampsCenti, reading.ampsCenti, sizeof(record.ampsCenti));
    record.crc32 = crc32(reinterpret_cast<const uint8_t*>(&record),
                         sizeof(record) - sizeof(record.crc32));
    return record;
}

bool valid(const WalRecord& record) {
    return record.magic == kWalMagic && record.intervalSeconds >= 1 &&
           record.intervalSeconds <= 60 &&
           record.crc32 == crc32(reinterpret_cast<const uint8_t*>(&record),
                                  sizeof(record) - sizeof(record.crc32));
}

ElectricalReading decode(const WalRecord& record) {
    ElectricalReading reading{};
    reading.bootId = record.bootId;
    reading.sequence = record.sequence;
    reading.epoch = record.epoch;
    reading.sampleMillis = record.sampleMillis;
    reading.voltageDecivolts = record.voltageDecivolts;
    reading.sampleDurationMs = record.sampleDurationMs;
    reading.intervalSeconds = record.intervalSeconds;
    reading.flags = record.flags;
    memcpy(reading.ampsCenti, record.ampsCenti, sizeof(reading.ampsCenti));
    return reading;
}

bool sameIdentity(const WalRecord& record, const ElectricalReading& reading) {
    return record.bootId == reading.bootId && record.sequence == reading.sequence;
}

bool copyBytes(File& source, File& destination, size_t count) {
    uint8_t buffer[256];
    while (count > 0) {
        const size_t wanted = min(count, sizeof(buffer));
        const size_t got = source.read(buffer, wanted);
        if (got != wanted || destination.write(buffer, got) != got) return false;
        count -= got;
        taskYIELD();
    }
    return true;
}

}  // namespace

bool DurableQueue::begin(const String& deviceId) {
    deviceId_ = deviceId;
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
        state_ = StorageState::Fault;
        return false;
    }

    // Never pass formatOnFail=true. Existing telemetry is more valuable than a boot.
    fsReady_ = LittleFS.begin(false);
    if (!fsReady_) {
        state_ = StorageState::Fault;
        return false;
    }

    if (!recoverCheckpointTransaction() || !recoverOfflineRepair() ||
        !quarantineInvalidWalTail()) {
        state_ = StorageState::Fault;
        return false;
    }
    walRecordCount_ = fileSize(kWalFile) / sizeof(WalRecord);
    Preferences counters;
    if (counters.begin("lsv3", true)) {
        stats_.permanentLosses = counters.getUInt("losses", 0);
        counters.end();
    }

    // An active LS01/LS02 file from a previous boot has a stale millis anchor.
    if (LittleFS.exists(kOfflineFile)) {
        String moved;
        if (!rotateActiveOffline(moved)) {
            state_ = StorageState::CapacityBlocked;
            sameSessionUploadPath_ = "";
        } else {
            // rotateActiveOffline is also used for files created this session. This
            // one predates boot, so its millis anchor must never be advertised valid.
            sameSessionUploadPath_ = "";
        }
    }

    reloadLiveLocked();
    while (walRecordCount_ > kLiveQueueCapacity) {
        if (!checkpointLocked()) {
            state_ = StorageState::CapacityBlocked;
            return false;
        }
    }
    state_ = StorageState::Ready;
    return true;
}

bool DurableQueue::recover() {
    if (!fsReady_ || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    bool ok = recoverCheckpointTransaction() && recoverOfflineRepair() &&
              quarantineInvalidWalTail();
    if (ok) {
        walRecordCount_ = fileSize(kWalFile) / sizeof(WalRecord);
        reloadLiveLocked();
        while (ok && walRecordCount_ > kLiveQueueCapacity) ok = checkpointLocked();
    }
    state_ = ok ? StorageState::Ready
                : (freeBytes() < kFilesystemReserveBytes
                       ? StorageState::CapacityBlocked : StorageState::Fault);
    xSemaphoreGive(mutex_);
    return ok;
}

bool DurableQueue::recoverOfflineRepair() {
    if (!LittleFS.exists(kOfflineRepairFile)) return true;
    String archive;
    for (uint8_t slot = 0; slot < 32; ++slot) {
        archive = "/offline.repair-orphan." + String(slot);
        if (!LittleFS.exists(archive.c_str())) break;
        archive = "";
    }
    // The pre-repair image remains archived and the WAL owns the in-flight
    // block. Retain this partial copy, then allow a clean repair retry.
    return !archive.isEmpty() && LittleFS.rename(kOfflineRepairFile, archive.c_str());
}

bool DurableQueue::recoverCheckpointTransaction() {
    const bool recovery = LittleFS.exists(kWalRecoveryFile);
    const bool current = LittleFS.exists(kWalFile);
    const bool next = LittleFS.exists(kWalNextFile);

    if (recovery && !current) {
        // The crash happened between the two renames. Replaying the old WAL can
        // duplicate an LS02 block, but cannot lose a reading.
        if (!LittleFS.rename(kWalRecoveryFile, kWalFile)) return false;
    } else if (recovery && current) {
        // A visible current WAL means the transaction reached its commit point.
        // Validate it before retiring the redundant pre-commit image.
        File file = LittleFS.open(kWalFile, "r");
        bool currentValid = static_cast<bool>(file);
        WalRecord record{};
        while (currentValid && file.available()) {
            if (file.read(reinterpret_cast<uint8_t*>(&record), sizeof(record)) != sizeof(record) ||
                !valid(record)) currentValid = false;
        }
        if (file) file.close();
        if (!currentValid) {
            String bad = "/v3.wal.bad-current";
            if (LittleFS.exists(bad)) bad += "." + String(millis());
            if (!LittleFS.rename(kWalFile, bad.c_str()) ||
                !LittleFS.rename(kWalRecoveryFile, kWalFile)) return false;
        } else if (!LittleFS.remove(kWalRecoveryFile)) {
            return false;
        }
    }

    // A .next without a recovery image was never committed. The authoritative
    // WAL is intact; keep the orphan for forensic recovery instead of deleting it.
    if (next && !LittleFS.exists(kWalRecoveryFile)) {
        String orphan = "/v3.wal.orphan-next";
        if (LittleFS.exists(orphan)) orphan += "." + String(millis());
        if (!LittleFS.rename(kWalNextFile, orphan.c_str())) return false;
    }
    return true;
}

bool DurableQueue::quarantineInvalidWalTail() {
    if (!LittleFS.exists(kWalFile)) return true;
    File source = LittleFS.open(kWalFile, "r");
    if (!source) return false;
    size_t validBytes = 0;
    WalRecord record{};
    while (source.available() >= static_cast<int>(sizeof(record))) {
        if (source.read(reinterpret_cast<uint8_t*>(&record), sizeof(record)) != sizeof(record) ||
            !valid(record)) break;
        validBytes += sizeof(record);
    }
    const size_t originalBytes = source.size();
    source.close();
    if (validBytes == originalBytes) return true;

    String archive;
    for (uint8_t slot = 0; slot < 32; ++slot) {
        archive = "/v3.wal.corrupt." + String(slot);
        if (!LittleFS.exists(archive.c_str())) break;
        archive = "";
    }
    if (archive.isEmpty() || !LittleFS.rename(kWalFile, archive.c_str())) return false;
    File archived = LittleFS.open(archive.c_str(), "r");
    File repaired = LittleFS.open(kWalFile, "w");
    if (!archived || !repaired || !copyBytes(archived, repaired, validBytes)) {
        if (archived) archived.close();
        if (repaired) repaired.close();
        return false;
    }
    repaired.flush();
    archived.close();
    repaired.close();
    return fileSize(kWalFile) == validBytes;
}

bool DurableQueue::commit(const ElectricalReading& reading) {
    if (!fsReady_ || mutex_ == nullptr) return false;
    if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    if (LittleFS.exists(kWalRecoveryFile) || LittleFS.exists(kWalNextFile)) {
        if (!recoverCheckpointTransaction() || !quarantineInvalidWalTail()) {
            ++stats_.localWriteFailures;
            state_ = StorageState::Fault;
            xSemaphoreGive(mutex_);
            return false;
        }
        walRecordCount_ = fileSize(kWalFile) / sizeof(WalRecord);
        reloadLiveLocked();
    }
    if (liveCount_ >= kLiveQueueCapacity && !checkpointLocked()) {
        ++stats_.localWriteFailures;
        state_ = StorageState::CapacityBlocked;
        xSemaphoreGive(mutex_);
        return false;
    }
    const bool stored = appendWal(reading);
    if (stored) {
        ++stats_.generated;
        ++stats_.durable;
        if (liveCount_ < kLiveQueueCapacity) {
            live_[liveCount_].reading = reading;
            live_[liveCount_].acknowledged = false;
            ++liveCount_;
        }
        state_ = StorageState::Ready;
    } else {
        ++stats_.localWriteFailures;
        state_ = freeBytes() < kFilesystemReserveBytes
                     ? StorageState::CapacityBlocked : StorageState::Fault;
    }
    xSemaphoreGive(mutex_);
    return stored;
}

bool DurableQueue::appendWal(const ElectricalReading& reading) {
    const WalRecord encoded = encode(reading);
    if (LittleFS.exists(kWalFile)) {
        File tail = LittleFS.open(kWalFile, "r");
        if (!tail) return false;
        const size_t tailSize = tail.size();
        if (tailSize % sizeof(WalRecord) != 0) {
            tail.close();
            if (!quarantineInvalidWalTail()) return false;
            walRecordCount_ = fileSize(kWalFile) / sizeof(WalRecord);
            return appendWal(reading);
        }
        if (tail.size() >= sizeof(WalRecord)) {
            tail.seek(tail.size() - sizeof(WalRecord));
            WalRecord last{};
            if (tail.read(reinterpret_cast<uint8_t*>(&last), sizeof(last)) == sizeof(last) &&
                valid(last) && sameIdentity(last, reading)) {
                walRecordCount_ = tail.size() / sizeof(WalRecord);
                tail.close();
                return true;
            }
            if (!valid(last)) {
                tail.close();
                if (!quarantineInvalidWalTail()) return false;
                walRecordCount_ = fileSize(kWalFile) / sizeof(WalRecord);
                return appendWal(reading);
            }
        }
        tail.close();
    }
    if (freeBytes() < kFilesystemReserveBytes + sizeof(WalRecord)) return false;
    const size_t offset = fileSize(kWalFile);
    File file = LittleFS.open(kWalFile, "a");
    if (!file) return false;
    const bool wrote = file.write(reinterpret_cast<const uint8_t*>(&encoded),
                                  sizeof(encoded)) == sizeof(encoded);
    file.flush();
    file.close();
    if (!wrote) return false;

    File verify = LittleFS.open(kWalFile, "r");
    WalRecord observed{};
    const bool verified = verify && verify.size() == offset + sizeof(encoded) &&
        verify.seek(offset) &&
        verify.read(reinterpret_cast<uint8_t*>(&observed), sizeof(observed)) == sizeof(observed) &&
        valid(observed) && observed.crc32 == encoded.crc32;
    if (verify) verify.close();
    if (verified) ++walRecordCount_;
    return verified;
}

bool DurableQueue::peekLive(ElectricalReading& reading) {
    if (!fsReady_ || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    bool found = false;
    for (size_t i = 0; i < liveCount_; ++i) {
        if (!live_[i].acknowledged) {
            reading = live_[i].reading;
            found = true;
            break;
        }
    }
    xSemaphoreGive(mutex_);
    return found;
}

void DurableQueue::acknowledgeLive(const ElectricalReading& reading) {
    if (!fsReady_ || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) return;
    for (size_t i = 0; i < liveCount_; ++i) {
        if (sameReading(live_[i].reading, reading)) {
            if (!live_[i].acknowledged) ++stats_.liveAcknowledged;
            live_[i].acknowledged = true;
            break;
        }
    }
    xSemaphoreGive(mutex_);
}

void DurableQueue::recordDeliveryResult(int httpCode, bool success) {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) return;
    stats_.lastHttpCode = httpCode;
    if (success) stats_.lastSuccessMillis = millis();
    else ++stats_.retries;
    xSemaphoreGive(mutex_);
}

void DurableQueue::recordCaptureGap(uint32_t intervals) {
    if (intervals == 0 || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) return;
    stats_.permanentLosses += intervals;
    const uint32_t total = stats_.permanentLosses;
    xSemaphoreGive(mutex_);
    Preferences counters;
    if (counters.begin("lsv3", false)) {
        counters.putUInt("losses", total);
        counters.end();
    }
}

bool DurableQueue::checkpoint() {
    if (!fsReady_ || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    const bool ok = checkpointLocked();
    if (!ok) state_ = freeBytes() < kFilesystemReserveBytes
                          ? StorageState::CapacityBlocked : StorageState::Fault;
    xSemaphoreGive(mutex_);
    return ok;
}

bool DurableQueue::checkpointLocked() {
    if (walRecordCount_ < kWalCheckpointRecords) return true;
    File wal = LittleFS.open(kWalFile, "r");
    if (!wal) return false;
    const size_t depth = wal.size() / sizeof(WalRecord);
    if (depth < kWalCheckpointRecords) {
        walRecordCount_ = depth;
        wal.close();
        return true;
    }

    ElectricalReading prefix[kWalCheckpointRecords];
    bool allAcknowledged = true;
    for (size_t i = 0; i < kWalCheckpointRecords; ++i) {
        WalRecord record{};
        if (wal.read(reinterpret_cast<uint8_t*>(&record), sizeof(record)) != sizeof(record) ||
            !valid(record)) {
            wal.close();
            return false;
        }
        prefix[i] = decode(record);
        allAcknowledged &= readingAcknowledgedLocked(prefix[i]);
    }
    wal.close();

    // If even one ACK is uncertain, archive the whole time-contiguous block.
    // This deliberately prefers a server-deduplicated replay over a gap.
    if (!allAcknowledged && !appendOfflineBlock(prefix, kWalCheckpointRecords)) return false;
    if (!rewriteWalWithoutPrefix(kWalCheckpointRecords)) return false;
    reloadLiveLocked();
    return true;
}

bool DurableQueue::readingAcknowledgedLocked(const ElectricalReading& reading) const {
    for (size_t i = 0; i < liveCount_; ++i) {
        if (sameReading(live_[i].reading, reading)) return live_[i].acknowledged;
    }
    return false;
}

bool DurableQueue::ensureOfflineHeader() {
    if (LittleFS.exists(kOfflineFile)) {
        File existing = LittleFS.open(kOfflineFile, "r");
        OfflineHeader observed{};
        const bool compatible = existing && existing.size() >= sizeof(observed) &&
            existing.read(reinterpret_cast<uint8_t*>(&observed), sizeof(observed)) == sizeof(observed) &&
            memcmp(observed.magic, kOfflineMagic, 4) == 0;
        if (existing) existing.close();
        if (compatible) return true;
        String moved;
        if (!rotateActiveOffline(moved)) return false;
        sameSessionUploadPath_ = "";
    }
    if (freeBytes() < kFilesystemReserveBytes + sizeof(OfflineHeader)) return false;
    OfflineHeader header{};
    memcpy(header.magic, kOfflineMagic, 4);
    strncpy(header.deviceId, deviceId_.c_str(), sizeof(header.deviceId) - 1);
    header.fileEpoch = static_cast<uint32_t>(time(nullptr));
    File file = LittleFS.open(kOfflineFile, "w");
    if (!file) return false;
    const bool wrote = file.write(reinterpret_cast<const uint8_t*>(&header),
                                  sizeof(header)) == sizeof(header);
    file.flush();
    file.close();
    if (wrote && fileSize(kOfflineFile) == sizeof(header)) return true;
    restoreOfflinePrefix(0, "header-failed");
    return false;
}

bool DurableQueue::appendOfflineBlock(const ElectricalReading* readings, size_t count) {
    if (count == 0 || count > kWalCheckpointRecords ||
        offlineBytes() >= kOfflineBudgetBytes ||
        freeBytes() < kFilesystemReserveBytes + 256) return false;
    if (!ensureOfflineHeader()) return false;

    uint8_t raw[kWalCheckpointRecords * kChannelCount * sizeof(int16_t)] = {0};
    size_t rawLength = 0;
    for (size_t i = 0; i < count; ++i) {
        memcpy(raw + rawLength, readings[i].ampsCenti, sizeof(readings[i].ampsCenti));
        rawLength += sizeof(readings[i].ampsCenti);
    }
    uint8_t compressed[256];
    const size_t compressedLength = tdefl_compress_mem_to_mem(
        compressed, sizeof(compressed), raw, rawLength, TDEFL_DEFAULT_MAX_PROBES);
    const bool useRaw = compressedLength == 0 || compressedLength == static_cast<size_t>(-1) ||
                        compressedLength >= rawLength;
    const uint8_t* payload = useRaw ? raw : compressed;
    const size_t payloadLength = useRaw ? rawLength : compressedLength;

    OfflineBlockHeader header{};
    header.startEpoch = readings[0].epoch;
    header.startMillis = readings[0].sampleMillis;
    header.flags = readings[0].flags;
    header.sizeFlags = static_cast<uint16_t>(payloadLength) | (useRaw ? kOfflineRawFlag : 0);
    header.crc16 = crc16(payload, payloadLength);

    const size_t before = fileSize(kOfflineFile);
    File file = LittleFS.open(kOfflineFile, "a");
    if (!file) return false;
    const bool wrote = file.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header)) == sizeof(header) &&
                       file.write(payload, payloadLength) == payloadLength;
    file.flush();
    file.close();
    if (!wrote || fileSize(kOfflineFile) != before + sizeof(header) + payloadLength) {
        restoreOfflinePrefix(before, "append-failed");
        return false;
    }

    File verify = LittleFS.open(kOfflineFile, "r");
    OfflineBlockHeader observed{};
    bool verified = verify && verify.seek(before) &&
        verify.read(reinterpret_cast<uint8_t*>(&observed), sizeof(observed)) == sizeof(observed) &&
        observed.crc16 == header.crc16 && observed.sizeFlags == header.sizeFlags;
    uint8_t observedPayload[256];
    if (verified) {
        verified = verify.read(observedPayload, payloadLength) == payloadLength &&
                   crc16(observedPayload, payloadLength) == observed.crc16;
    }
    if (verify) verify.close();
    if (!verified) {
        restoreOfflinePrefix(before, "verify-failed");
        return false;
    }

    if (fileSize(kOfflineFile) >= kOfflineChunkBytes) {
        String moved;
        if (!rotateActiveOffline(moved)) return false;
    }
    return true;
}

bool DurableQueue::restoreOfflinePrefix(size_t verifiedBytes, const char* reason) {
    if (!LittleFS.exists(kOfflineFile) || LittleFS.exists(kOfflineRepairFile)) return false;
    String archive;
    for (uint8_t slot = 0; slot < 32; ++slot) {
        archive = "/offline." + String(reason) + "." + String(slot) + ".dat";
        if (!LittleFS.exists(archive.c_str())) break;
        archive = "";
    }
    if (archive.isEmpty() || !LittleFS.rename(kOfflineFile, archive.c_str())) return false;
    if (verifiedBytes == 0) return true;

    File source = LittleFS.open(archive.c_str(), "r");
    File repair = LittleFS.open(kOfflineRepairFile, "w");
    if (!source || !repair || source.size() < verifiedBytes ||
        !copyBytes(source, repair, verifiedBytes)) {
        if (source) source.close();
        if (repair) repair.close();
        return false;
    }
    repair.flush();
    source.close();
    repair.close();
    if (fileSize(kOfflineRepairFile) != verifiedBytes ||
        !LittleFS.rename(kOfflineRepairFile, kOfflineFile)) return false;
    return fileSize(kOfflineFile) == verifiedBytes;
}

bool DurableQueue::rewriteWalWithoutPrefix(size_t prefixRecords) {
    File source = LittleFS.open(kWalFile, "r");
    if (!source) return false;
    const size_t prefixBytes = prefixRecords * sizeof(WalRecord);
    if (source.size() < prefixBytes || !source.seek(prefixBytes)) {
        source.close();
        return false;
    }
    if (LittleFS.exists(kWalNextFile)) {
        source.close();
        return false;
    }
    File next = LittleFS.open(kWalNextFile, "w");
    if (!next) {
        source.close();
        return false;
    }
    const size_t tailBytes = source.size() - prefixBytes;
    const bool copied = copyBytes(source, next, tailBytes);
    next.flush();
    source.close();
    next.close();
    if (!copied || fileSize(kWalNextFile) != tailBytes) return false;
    if (LittleFS.exists(kWalRecoveryFile) ||
        !LittleFS.rename(kWalFile, kWalRecoveryFile) ||
        !LittleFS.rename(kWalNextFile, kWalFile)) return false;

    if (fileSize(kWalFile) != tailBytes) return false;
    if (!LittleFS.remove(kWalRecoveryFile)) return false;
    walRecordCount_ -= prefixRecords;
    return true;
}

void DurableQueue::reloadLiveLocked() {
    liveCount_ = 0;
    File file = LittleFS.open(kWalFile, "r");
    if (!file) return;
    WalRecord record{};
    while (liveCount_ < kLiveQueueCapacity &&
           file.read(reinterpret_cast<uint8_t*>(&record), sizeof(record)) == sizeof(record)) {
        if (!valid(record)) break;
        live_[liveCount_].reading = decode(record);
        live_[liveCount_].acknowledged = false;
        ++liveCount_;
    }
    file.close();
}

String DurableQueue::allocateLegacyPathLocked() const {
    int highest = LittleFS.exists(kOfflinePrimaryLegacy) ? 0 : -1;
    for (uint8_t slot = 1; slot < 100; ++slot) {
        const String path = "/offline.legacy." + String(slot) + ".dat";
        if (LittleFS.exists(path.c_str())) highest = slot;
    }
    if (highest < 0) return String(kOfflinePrimaryLegacy);
    if (highest >= 99) return String();
    return "/offline.legacy." + String(highest + 1) + ".dat";
}

bool DurableQueue::rotateActiveOffline(String& destination) {
    if (!LittleFS.exists(kOfflineFile)) return true;
    destination = allocateLegacyPathLocked();
    if (destination.isEmpty()) return false;
    if (!LittleFS.rename(kOfflineFile, destination.c_str())) return false;
    sameSessionUploadPath_ = destination;
    return true;
}

String DurableQueue::firstBulkPathLocked(bool includeActive) const {
    if (LittleFS.exists(kOfflinePrimaryLegacy)) return String(kOfflinePrimaryLegacy);
    for (uint8_t slot = 1; slot < 100; ++slot) {
        const String path = "/offline.legacy." + String(slot) + ".dat";
        if (LittleFS.exists(path.c_str())) return path;
    }
    for (uint16_t slot = 0; slot < 1000; ++slot) {
        const String path = "/offline.rejected." + String(slot) + ".dat";
        if (LittleFS.exists(path.c_str())) return path;
    }
    const char* failureKinds[] = {"append-failed", "verify-failed"};
    for (const char* kind : failureKinds) {
        for (uint8_t slot = 0; slot < 32; ++slot) {
            const String path = "/offline." + String(kind) + "." + String(slot) + ".dat";
            if (LittleFS.exists(path.c_str())) return path;
        }
    }
    return includeActive && LittleFS.exists(kOfflineFile) ? String(kOfflineFile) : String();
}

bool DurableQueue::nextBulkUpload(String& path, bool& anchorValid) {
    path = "";
    anchorValid = false;
    if (!fsReady_ || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    path = firstBulkPathLocked(false);
    if (path.isEmpty() && fileSize(kOfflineFile) > sizeof(OfflineHeader)) {
        if (!rotateActiveOffline(path)) {
            xSemaphoreGive(mutex_);
            return false;
        }
    }
    anchorValid = !path.isEmpty() && path == sameSessionUploadPath_;
    xSemaphoreGive(mutex_);
    return !path.isEmpty();
}

bool DurableQueue::acknowledgeBulkUpload(const String& path) {
    if (!fsReady_ || path.isEmpty() || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    const bool removed = LittleFS.exists(path.c_str()) && LittleFS.remove(path.c_str());
    if (removed) {
        ++stats_.bulkAcknowledged;
        if (path == sameSessionUploadPath_) sameSessionUploadPath_ = "";
    }
    xSemaphoreGive(mutex_);
    return removed;
}

bool DurableQueue::nextLegacyJsonUpload(String& path) {
    path = "";
    if (!fsReady_ || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(2000)) != pdTRUE) return false;
    if (LittleFS.exists(kLegacyBufferNdjson)) {
        path = kLegacyBufferNdjson;
    } else if (LittleFS.exists(kLegacyBufferFile)) {
        File source = LittleFS.open(kLegacyBufferFile, "r");
        JsonDocument document;
        const DeserializationError error = deserializeJson(document, source);
        source.close();
        if (!error && document.is<JsonArray>()) {
            File output = LittleFS.open(kLegacyBufferNdjsonTmp, "w");
            bool ok = static_cast<bool>(output);
            if (ok) {
                for (JsonVariant entry : document.as<JsonArray>()) {
                    ok &= serializeJson(entry, output) > 0;
                    ok &= output.write('\n') == 1;
                }
                output.flush();
                output.close();
            }
            if (ok && LittleFS.rename(kLegacyBufferNdjsonTmp, kLegacyBufferNdjson)) {
                path = kLegacyBufferNdjson;
            }
        }
    }

    if (path.isEmpty()) {
        for (uint8_t slot = 1; slot <= 32; ++slot) {
            const String candidate = "/rejected." + String(slot) + ".log";
            if (LittleFS.exists(candidate.c_str())) {
                path = candidate;
                break;
            }
        }
    }
    if (path.isEmpty() && LittleFS.exists(kRejectedFile)) {
        for (uint8_t slot = 1; slot <= 32; ++slot) {
            const String destination = "/rejected." + String(slot) + ".log";
            if (!LittleFS.exists(destination.c_str()) &&
                LittleFS.rename(kRejectedFile, destination.c_str())) {
                path = destination;
                break;
            }
        }
    }
    xSemaphoreGive(mutex_);
    return !path.isEmpty();
}

bool DurableQueue::acknowledgeLegacyJsonUpload(const String& path) {
    if (!fsReady_ || path.isEmpty() || mutex_ == nullptr ||
        xSemaphoreTake(mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    bool removed = LittleFS.exists(path.c_str()) && LittleFS.remove(path.c_str());
    if (removed && path == kLegacyBufferNdjson && LittleFS.exists(kLegacyBufferFile)) {
        removed = LittleFS.remove(kLegacyBufferFile);
    }
    xSemaphoreGive(mutex_);
    return removed;
}

DeliveryStats DurableQueue::stats() const {
    if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) != pdTRUE) return stats_;
    const DeliveryStats copy = stats_;
    xSemaphoreGive(mutex_);
    return copy;
}

uint32_t DurableQueue::walBytes() const { return walRecordCount_ * sizeof(WalRecord); }

uint32_t DurableQueue::offlineBytes() const {
    if (!fsReady_) return 0;
    uint32_t total = 0;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        const String name = file.name();
        if (!file.isDirectory() && (name.endsWith("offline.dat") ||
            (name.indexOf("offline.") >= 0 && name.endsWith(".dat")))) {
            total += file.size();
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    return total;
}

uint32_t DurableQueue::legacyJsonBytes() const {
    if (!fsReady_) return 0;
    uint32_t total = 0;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        const String name = file.name();
        if (!file.isDirectory() && (name.endsWith("buffer.json") ||
            name.endsWith("buffer.ndjson") ||
            (name.indexOf("rejected.") >= 0 && name.endsWith(".log")))) {
            total += file.size();
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    return total;
}

uint32_t DurableQueue::quarantineBytes() const {
    if (!fsReady_) return 0;
    uint32_t total = 0;
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while (file) {
        const String name = file.name();
        if (!file.isDirectory() && (name.indexOf("v3.wal.corrupt.") >= 0 ||
            name.indexOf("v3.wal.bad-current") >= 0 ||
            name.indexOf("v3.wal.orphan-next") >= 0 ||
            name.indexOf("offline.append-failed") >= 0 ||
            name.indexOf("offline.verify-failed") >= 0 ||
            name.indexOf("offline.header-failed") >= 0 ||
            name.indexOf("offline.repair-orphan") >= 0)) {
            total += file.size();
        }
        file.close();
        file = root.openNextFile();
    }
    root.close();
    return total;
}

uint32_t DurableQueue::freeBytes() const {
    if (!fsReady_) return 0;
    const size_t total = LittleFS.totalBytes();
    const size_t used = LittleFS.usedBytes();
    return used < total ? total - used : 0;
}

uint32_t DurableQueue::fileSize(const char* path) const {
    if (!fsReady_ || !LittleFS.exists(path)) return 0;
    File file = LittleFS.open(path, "r");
    if (!file) return 0;
    const uint32_t size = file.size();
    file.close();
    return size;
}

}  // namespace fw
