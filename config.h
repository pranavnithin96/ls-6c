#pragma once
// ============================================================================
// LineSights LS-6C-IOT v2.7.0 — Configuration
// ============================================================================

#define FIRMWARE_VERSION "2.13.0"

// --- Feature flags ---
// Per-second waveform features (peak/env_peak_ratio/ripple/env5) in the LIVE
// payload only. Default OFF; runtime override via Preferences("lscfg","wfstats").
// When off, the live payload and offline records are byte-identical to v2.6
// except the unconditional LS01->LS02 offline-format bump (C3 needs it).
#define FEATURE_WAVEFORM_STATS 0

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
// Bulk POST guards. The watchdog is fed INSIDE the bounded loops, so the
// total cap does NOT need to sit under WDT_TIMEOUT_S — only the stall abort
// does (asserted in http_sender.h). v2.11.0's flat 45s deadline proved too
// short in the field: a 50-64KB backlog file over Meton's ~600B/s trickle
// needs ~85-110s, so every attempt aborted at 45s, delivered nothing (nginx
// logged the premature closes as 400), and starved the live stream for the
// full deadline. Abort only on a genuine stall — zero bytes accepted for
// BULK_POST_STALL_MS — and cap the whole request at BULK_POST_MAX_MS, sized
// so legacy 50KB rejected archives and 64KB offline chunks still complete
// at trickle rates while bounding live-stream starvation.
#define BULK_POST_STALL_MS        12000UL
// Response wait is a SEPARATE, longer guard: send() success only means bytes
// entered the device's ~5.7KB TCP send buffer, not that they reached the
// server — after the "last" send, that tail still needs 10-60s to crawl out
// over a trickling uplink, and the server cannot respond until it has every
// byte. v2.11.1 reused the 12s stall here and hung up on its own uploads
// (nginx 499s, zero deliveries, bandwidth fully wasted). 60s covers the tail
// at ≥100B/s; the server itself answers in ~11ms.
#define BULK_RESPONSE_WAIT_MS     60000UL
// Total cap: worst-case send at trickle + tail drain + response. Legacy
// 50-100KB archives (pre-8KB rotation) need most of this once; steady-state
// 8KB units finish in seconds. WDT is fed throughout — the cap bounds live-
// stream starvation, not the watchdog.
#define BULK_POST_MAX_MS          300000UL

// --- Send Mode ---
#define DEFAULT_SEND_INTERVAL    1  // seconds

// --- Time sync ---
#define NTP_SYNC_INTERVAL_MS   900000  // SNTP auto re-sync every 15 min (explicit, not SDK default)
#define NTP_STALE_RESYNC_S     7200    // re-kick syncNTP if no successful sync for 2h while WiFi up

// --- Maintenance reboot ---
// Planned reboot beats an unplanned wedge, but it costs ~5-20s of sampling —
// a deliberate, owner-approved zero-loss exception. Deterministic per-device
// jitter (±3h from MAC) staggers the fleet. 0 = disabled.
#define SCHEDULED_REBOOT_DAYS  7       // runtime override: Preferences "rebootdays" / heartbeat set_reboot_days

// --- Offline chunking (poor-WiFi resumability) ---
// Cap each offline file at 64KB: uploads become short bursts a spotty link can
// finish, and delivered chunks are progress a dying stream can't take back.
#define OFFLINE_CHUNK_BYTES    65536

// --- Offline Storage (tiered, gzip compressed) ---
#define OFFLINE_FILE           "/offline.dat"
#define OFFLINE_MAX_BYTES      550000     // Cap at ~537KB (leave room for other files)
#define OFFLINE_GRACE_MS       30000      // 30s before entering offline mode
#define OFFLINE_BLOCK_READINGS 10         // Compress every 10 readings (10s blocks, max 9s loss on crash)
#define OFFLINE_BLOCK_RAW_SIZE (OFFLINE_BLOCK_READINGS * 12)  // 120 bytes per block
#define OFFLINE_UPLOAD_RETRY_MS 60000     // Retry upload every 60s on failure

// --- Buffers ---
#define MAX_BUFFER_SIZE       30    // 30s pipeline buffer (fixed char arrays, no heap alloc)
// Background (offline/rejected) uploads normally wait for the live send ring to
// drain first, so fresh readings ship ahead of backlog. Waiting for the ring to
// be *completely* empty deadlocks on spotty-but-connected units whose ring never
// empties: the drain never runs, /rejected.log never clears (proven: pcs1/pcs7
// drained 0 bytes). So allow a background upload once the ring is at/below this
// low-water mark, and force one after BG_UPLOAD_STARVE_MS no matter what.
#define BG_UPLOAD_LOWATER      (MAX_BUFFER_SIZE / 4)   // 7 of 30 — "mostly caught up"
#define BG_UPLOAD_STARVE_MS    120000UL                // never stall a pending drain/upload >2min
// Rejected-log recovery. Whenever reject data is parked AND the device is online
// (boot with backlog, or a live 200 after reconnect), it drains first — lossless
// upload; the idempotent server dedups. The dispose backstop watches DRAIN
// PROGRESS, not wall-clock time: every file delivered restarts the clock, so a
// drain that is getting somewhere is never cut off no matter how big the backlog
// or how slowly the ring lets it run. Only a drain that is genuinely stuck —
// nothing delivered for REJECTED_STALL_MS while live data keeps flowing — gets
// the log DISPOSED, because a full rejected log drags throughput (FS churn) and
// its contents are largely duplicates the server already has.
//
// Why progress and not a deadline: the drain is gated on a quiet ring, and on a
// saturated unit it only runs via the starvation escape every BG_UPLOAD_STARVE_MS.
// A fixed grace shorter than that escape disposes the backlog before the drain is
// ever ALLOWED to run — which is exactly the saturated unit that has a full log.
// The static_assert keeps that relationship from being broken by a later tweak.
#define REJECTED_ONLINE_WINDOW_MS  60000UL   // a live 200 this recent == "online"
#define REJECTED_STALL_MS         300000UL   // no drain progress this long while online -> dispose
static_assert(REJECTED_STALL_MS > BG_UPLOAD_STARVE_MS,
              "REJECTED_STALL_MS must outlast BG_UPLOAD_STARVE_MS, or a ring-blocked "
              "drain is disposed before the starvation escape ever lets it run");
#define BUFFER_SAVE_INTERVAL_MS  60000   // 60 seconds (minimize power-loss data window)
// Trouble cadence for the same save: while sending is failing (or WiFi is
// down) the RAM ring flushes to flash every 5 s instead of 60 s. A network
// bad enough to starve the watchdog reboots the device before the healthy
// cadence ever fires — Meton 2026-07-25: ~100 s reboot loop x 60 s cadence
// ate ~50% of a day's readings, all of it RAM-only ring. Healthy devices
// never take this path, so steady-state flash wear is unchanged.
#define TROUBLE_SAVE_INTERVAL_MS 5000

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

