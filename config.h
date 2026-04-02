#pragma once
// ============================================================================
// LineSights LS-6C-IOT v2.0.0 — Configuration
// ============================================================================

#define FIRMWARE_VERSION "2.1.0"

// --- Hardware Pins ---
#define NUM_CT_CHANNELS   6
static const int CT_PINS[NUM_CT_CHANNELS] = {36, 39, 34, 35, 32, 33};
#define LED_PIN           23
#define BOOT_BUTTON_PIN   0

// --- CT Sensor ---
#define ADC_SAMPLES_PER_CH    500     // 500ms sampling window
#define SAMPLE_INTERVAL_US    1000    // 1 kHz ADC rate within window
#define CAL_POINTS            3
// Corrected: manufacturer 0.0123 A/count, with ADC_11db 1 count = 0.756 mV
// So 0.0123 / 0.756 = 0.01627 A/mV
#define DEFAULT_AMPS_PER_MV   0.01627f
#define NOISE_THRESHOLD_W     5.0f
#define DEFAULT_GRID_VOLTAGE  230.0f
#define DEFAULT_PF            0.90f

// --- Network ---
#define HTTP_TIMEOUT_MS           5000
#define HEARTBEAT_TIMEOUT_MS      5000
#define MAX_CONSECUTIVE_FAILURES  5
#define MAX_BACKOFF_MS            5000    // Cap at 5s (was 30s — caused death spiral)
#define MAX_SENDS_PER_LOOP        3
#define DNS_CACHE_TTL_MS          300000  // 5 minutes

// --- Send Mode ---
#define DEFAULT_SEND_INTERVAL    1  // seconds

// --- Offline Storage (tiered, gzip compressed) ---
#define OFFLINE_FILE           "/offline.dat"
#define OFFLINE_MAX_BYTES      550000     // Cap at ~537KB (leave room for other files)
#define OFFLINE_GRACE_MS       30000      // 30s before entering offline mode
#define OFFLINE_BLOCK_READINGS 60         // Compress every 60 readings (1 min blocks)
#define OFFLINE_BLOCK_RAW_SIZE (OFFLINE_BLOCK_READINGS * 12)  // 720 bytes per block
#define OFFLINE_UPLOAD_RETRY_MS 60000     // Retry upload every 60s on failure

// --- Buffers ---
#define MAX_BUFFER_SIZE       30    // 30s pipeline buffer (fixed char arrays, no heap alloc)
#define BUFFER_SAVE_INTERVAL_MS  60000   // 60 seconds (minimize power-loss data window)

// --- Heartbeat ---
#define HEARTBEAT_INTERVAL_MS 60000
#define MAX_ERROR_LOG         30
#define ERROR_SAVE_INTERVAL_MS 300000

// --- OTA ---
#define OTA_CHECK_INTERVAL_MS 3600000  // 1 hour
#define OTA_DOWNLOAD_TIMEOUT  300000   // 300 seconds (1MB at 4KBps worst-case Indian WiFi)
#define OTA_RETRY_DELAY_MS    10000   // 10s before retry on failure
#define OTA_MAX_RETRIES       3       // Retry download up to 3 times
#define MAX_CRASH_COUNT       3
#define OTA_MAX_SIZE          0x140000 // 1,310,720 bytes (partition size)

// --- Diagnostics ---
#define WDT_TIMEOUT_S         60
#define HEALTH_REPORT_INTERVAL_MS 60000
#define LOW_HEAP_THRESHOLD    20480
#define NETWORK_TASK_STACK    16384  // 16 KB (increased from 8 KB)

// --- WiFi ---
#define WIFI_CONNECT_TIMEOUT_MS  15000
#define NTP_SYNC_TIMEOUT_MS      10000
#define AP_SSID_PREFIX           "LineSights-"
#define MDNS_HOSTNAME            "linesights"
#define FACTORY_RESET_HOLD_MS    5000

