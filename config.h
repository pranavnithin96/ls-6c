#pragma once

#include <Arduino.h>

namespace fw {

constexpr char kFirmwareVersion[] = "3.0.0-refactor";
constexpr char kDefaultServerUrl[] = "http://46.224.90.187/api/data";

constexpr uint8_t kChannelCount = 6;
constexpr uint8_t kCtPins[kChannelCount] = {36, 39, 34, 35, 32, 33};
constexpr uint8_t kLedPin = 23;
constexpr uint8_t kBootButtonPin = 0;

constexpr uint16_t kSamplesPerWindow = 500;
constexpr uint16_t kSamplePeriodUs = 1000;
constexpr uint16_t kDefaultVoltageDecivolts = 2300;
constexpr uint16_t kDefaultPowerFactorMilli = 900;
constexpr uint8_t kDefaultIntervalSeconds = 1;

constexpr uint8_t kLiveQueueCapacity = 32;
constexpr uint8_t kWalCheckpointRecords = 10;
constexpr uint32_t kOfflineChunkBytes = 64 * 1024UL;
constexpr uint32_t kOfflineBudgetBytes = 550000UL;
constexpr uint32_t kFilesystemReserveBytes = 32768UL;

constexpr uint32_t kNetworkLoopMs = 100;
constexpr uint32_t kWifiConnectTimeoutMs = 15000;
constexpr uint32_t kHttpTimeoutMs = 5000;
constexpr uint32_t kBulkTimeoutMs = 300000;
constexpr uint32_t kBulkStallMs = 12000;
constexpr uint32_t kBulkResponseWaitMs = 60000;
constexpr uint32_t kHeartbeatIntervalMs = 60000;
constexpr uint32_t kOtaCheckIntervalMs = 3600000;
constexpr uint32_t kOtaFirstCheckMs = 300000;
constexpr uint32_t kRetryMinMs = 1000;
constexpr uint32_t kRetryMaxMs = 60000;

constexpr uint32_t kWatchdogSeconds = 60;
constexpr uint32_t kNetworkTaskStack = 16384;

constexpr char kWalFile[] = "/v3.wal";
constexpr char kWalRecoveryFile[] = "/v3.wal.recovery";
constexpr char kOfflineFile[] = "/offline.dat";
constexpr char kOfflinePrimaryLegacy[] = "/offline.legacy.dat";
constexpr char kLegacyBufferFile[] = "/buffer.json";
constexpr char kRejectedFile[] = "/rejected.log";

constexpr uint32_t kWalMagic = 0x334C4157UL;  // "WAL3" little-endian
constexpr char kOfflineMagic[] = "LS02";
constexpr uint8_t kFlagClockUnsynced = 0x01;

static_assert(kWalCheckpointRecords <= 10,
              "LS02 compatibility blocks are limited to ten readings");

}  // namespace fw
