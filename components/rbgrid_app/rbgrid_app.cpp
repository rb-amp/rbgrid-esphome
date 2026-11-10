/* SPDX-FileCopyrightText: 2025-2026 Zhuhai RobotDyn Technology Co., Ltd.
 * SPDX-License-Identifier: MIT
 */
/**
 * @file main.cpp
 * @brief RBgrid application entry point: boot sequence, configuration, Wi-Fi/web/MQTT start-up, the rbAmp
 *        metering bridge and the debug console verbs.
 *
 * app_main() brings up NVS, storage, the rbpower home network from the stored configuration, the rbAmp I2C
 * fleet (main/rbamp_bridge.c), Wi-Fi, the web server, telemetry, MQTT and Home Assistant discovery, and the
 * optional ESP-NOW hub. Debug builds (CONFIG_RBGRID_DEBUG_BRIDGE or CONFIG_RBGRID_BENCH_FULL) also register
 * the rbbench console verbs defined in this file; they run in the rbbench console task (UART or
 * POST /api/v1/debug/cmd).
 */

// ===== BUILD FEATURE FLAGS (must precede the includes that use them) =====
// 🔴 EXPLICIT sdkconfig.h include — the `-include sdkconfig.h` force-include is NOT reliably
// in effect this early (same trap that silently flipped CONFIG_SPIRAM/MAX_UNITS off before).
// Without this, CONFIG_RBGRID_BENCH_FULL reads as undefined here → RBGRID_RBBENCH=0 → the whole
// rbbench console is compiled OUT of the unified build (dead console + no diag).
#include "rbgrid_tz.h"   /* RBgrid local time: own zone, not the process TZ */
#include "rbgrid_app.h"
#include "sdkconfig.h"
// CONFIG_RBGRID_DEBUG_BRIDGE / CONFIG_RBGRID_BENCH_FULL come from sdkconfig.h (included above).
// - RBAMP_BRIDGE_ENABLE / RBGRID_RBAMP_FLEET: rbAmp I2C fleet as the telemetry source.
// - RBGRID_RBBENCH: rbbench console (load/meter/i2cscan verbs) — test-rig control.
// - RBGRID_WIFI: wifi + webserver + telemetry + MQTT.
// DECOUPLED so rbbench and wifi COEXIST in ONE firmware (PSRAM removed the RAM limit that forced
// them apart). RBGRID_BENCH_FULL=1 → the non-debug (wifi) build ALSO gets rbbench, so a load
// cross-check can be automated via the `load` verb in the MQTT build.
// Needs CONSOLE_NONE in sdkconfig (rbbench owns UART0; app ESP_LOG still prints via ROM). The
// pure debug-bridge build is unchanged (wifi off). A product build sets RBGRID_BENCH_FULL=0.
#ifndef RBAMP_BRIDGE_ENABLE
/** @brief 1 = the rbAmp I2C fleet is the metering source (default). */
#define RBAMP_BRIDGE_ENABLE 1
#endif
// RBGRID_BENCH_FULL mirrors the Kconfig CONFIG_RBGRID_BENCH_FULL (single source of truth so the
// CMake component gate and the C feature gate agree). A -D override on the command line still wins.
#ifndef RBGRID_BENCH_FULL
/** @brief 0 = release build (no rbbench console in the Wi-Fi build). */
#define RBGRID_BENCH_FULL 0
#endif
/** @brief 1 = start the rbAmp fleet bridge. */
#define RBGRID_RBAMP_FLEET 1
/** @brief 0 = no rbbench console / debug verbs. */
#define RBGRID_RBBENCH 0
/** @brief 1 = Wi-Fi, web server, telemetry and MQTT enabled. */
#define RBGRID_WIFI 1

#include <math.h>
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rbpower.h"
#include "rbgrid_core.h"           // core API: lifecycle hooks, factory reset, context
#include "rbhome_tariff.h"   // rbpower_tariff_get_current_plan — provisioning-status tariff_set
#include "rbpower_inject_test.h"   // external-source injection self-test (opt-in)
#include "rbpower_logic_test.h"    // deterministic logic regression suite (selftest verb)
#include "rbamp_bridge.h"          // rbAmp I2C metering bridge
#include "esp_now_hub.h"           // ESP-NOW hub transport (beacon master + node recv)
#include "rbevlog.h"               // on-hub event log + remote console verbs (for OTA-only hubs)
extern "C" void modfw_register_http(void);   // main/modfw.c — STD module firmware update over the hub
extern "C" char *rbi2c_action(const cJSON *req, int *status);   // rbi2c_cmd.c — I2C fleet actions (HTTP + cmd channel)
extern "C" void rbi2c_register_http(void);   // main/rbi2c_api.c — I2C buses / module address API
#include "esp_system.h"
#include "rbhome_types.h"          // rbpower_get_unit_200ms_data for the `unit` verb
#include "esp_task_wdt.h"
/* Task-watchdog ISR hook: with the ESP console at NONE the TWDT panic print is lost; stash the task
 * running on each core in the RTC boot marker so the boot line names it. */
static void rbgrid_twdt_msg(void *opaque, const char *msg) { (void)opaque; rbpower_boot_note_wdt_msg(msg); }
extern "C" void esp_task_wdt_isr_user_handler(void)
{
    TaskHandle_t t0 = xTaskGetCurrentTaskHandleForCore(0);
    TaskHandle_t t1 = xTaskGetCurrentTaskHandleForCore(1);
    rbpower_boot_note_wdt(t0 ? pcTaskGetName(t0) : NULL, t1 ? pcTaskGetName(t1) : NULL);
    /* The running task is often just IDLE: the entry that STARVED is what matters —
     * walk the TWDT entry list the same way the IDF panic print does and stash the names too. */
    int cpus_fail = 0;
    esp_task_wdt_print_triggered_tasks(rbgrid_twdt_msg, NULL, &cpus_fail);
}
#include "rbpower_lifetime_api.h"  // etstat verb: et totalizers + restore-src
#include "esp_mac.h"               // esp_read_mac — mdns hostname rbgrid-<mac4>
#include "esp_app_desc.h"          // fw version in the _rbgrid._tcp TXT record
#include "rbtelemetry.h"
#include "rbmqtt.h"
#include "esp_mac.h"
#include "esp_wifi.h"   // esp_wifi_disconnect() — `wifidrop` debug verb
#include "esp_ota_ops.h" // OTA rollback: mark the freshly-OTA'd image valid once healthy
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"   // ha_discovery task + signal semaphore
#include "driver/i2c_master.h"   /* test-rig load-bank relay fail-safe OFF at boot */
#include "rbjson.h"
#include "nvs_flash.h"   /* debug-bridge build inits NVS here (rbwifi_init is gated out) */
#include "nvs.h"
#include "esp_random.h"
#include "esp_event.h"
#include "rbgrid.h"
#include "rbgrid_config_api.h"  // rbgrid_config_create and related config API
#include "rbgrid_config_storage.h"  // For config storage on external Flash
#include "littlefs_raw.h"           // littlefs_raw_remove — rbampcfg clear
#include "littlefs_raw.h"  // For raw file access

// Flash modules test
#include "rbextflash_config.h"
#include "rbextflash_config_runtime.h"
#include "rbextflash_operations.h"
#include "rbstats_v4_types.h"     // statmeta: raw v4 record view (set_meta check)
#include "rbstorage_transfer_types.h"   /* rbpower_electrical_transfer_t — stattest known-record */
#include "external_flash_config.h"

//#include "rbpower_console.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

// ===== ADD FOR CONSOLE =====
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs.h"
#include "driver/uart_vfs.h"
#include "driver/uart.h"
#include "driver/uart_select.h"
#include "linenoise/linenoise.h"
#include "rbstrutil.h"     /* rb_scat — bounded snprintf append */

// ===== WI-FI removed - now in rbwifi module

// WiFi credentials: provisioned via a GITIGNORED local header so the secret never enters
// the tracked repo. Copy components/rbgrid_app/wifi_secrets.h.example -> components/rbgrid_app/wifi_secrets.h and fill it in.
// If the file is absent, the firmware falls back to NVS-stored credentials (AP portal).
#if defined(__has_include)
#  if __has_include("wifi_secrets.h")
#    include "wifi_secrets.h"
#  endif
#endif

// MQTT broker URI for the telemetry MQTT channel. Overridable in wifi_secrets.h or with -D. The default
// is "" (MQTT stays off until the broker is configured via /api/system/mqtt-config; rbmqtt treats empty
// as no-op). A persisted mqtt-config always wins.
#ifndef RBGRID_MQTT_BROKER_URI
/** @brief Default MQTT broker URI: empty = MQTT off until configured. */
#  define RBGRID_MQTT_BROKER_URI ""
#endif



// CONFIG_* constants for console
#ifndef CONFIG_ESP_CONSOLE_UART_NUM
/** @brief Console UART fallback when the Kconfig option is absent. */
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#endif

/** @brief 1 = development default fleet (modules 0x50/0x51/0x52 with test CTs and a voltage trim for an
 *  uncalibrated sample); 0 (product) = one factory-calibrated module at 0x50, no host trim. Affects only a hub
 *  without a saved fleet config (/rbamp.json). */
#ifndef RBGRID_DEFAULT_BENCH_FLEET
#define RBGRID_DEFAULT_BENCH_FLEET 0
#endif

static const char* TAG = "RB-APP";

/* AP password: the setup password from provisioning (ap_provision.h, RBGRID_AP_SETUP_PASSWORD)
 * until the USER sets their own (POST /api/system/ap → NVS rbwifi/ap_pw_user). The hub never generates or changes it
 * by itself; a random per-device password stored by older firmware (NVS rbwifi/ap_pw) is dropped at the first boot. */
/* 
 * @brief Read the user-set AP password from NVS (rbwifi/ap_pw_user).
 * @param[out] out  Password buffer.
 * @param[in]  n    Size of @p out.
 * @return true if the user has set a password.
 */
extern "C" bool rbgrid_ap_user_password(char *out, size_t n);
/** Drop the hub-generated AP password stored by older firmware (NVS rbwifi/ap_pw). */
static void rbgrid_ap_drop_auto_password(void)
{
    nvs_handle_t h;
    esp_err_t oe = nvs_open("rbwifi", NVS_READWRITE, &h);
    if (oe == ESP_ERR_NVS_NOT_INITIALIZED && nvs_flash_init() == ESP_OK) oe = nvs_open("rbwifi", NVS_READWRITE, &h);
    if (oe != ESP_OK) return;
    if (nvs_erase_key(h, "ap_pw") == ESP_OK) {
        nvs_commit(h);
        rbpower_diag_printf("wifi: the hub-generated AP password is dropped — setup password until the user sets one");
    }
    nvs_close(h);
}


/** Period-boundary callback from the stats task: at each 15-min boundary, request the rbAmp period collect. */
static void rbgrid_rbamp_period_cb(rbpower_stat_period_type_t period, uint64_t ts) {
    (void)ts;
    if (period != RBPOWER_STAT_PERIOD_15MIN) return;
    /* hand the collect to the bridge poll task: done here (rbpwr_stats, task watchdog 5 s) a slow BASIC bus
     * transaction at the quarter boundary could starve the stats task and reset the hub */
    rbamp_bridge_request_collect(RBPOWER_STAT_PERIOD_15MIN, rbpower_stats_quarter_boundary_now());
}

/** Is this rbAmp (module_index, channel) still bound to a unit in the PERSISTED
 * fleet (/rbamp.json)? Returns false ONLY when a matching entry exists with unit_id==0 — i.e. a unit
 * was deleted via the API and rbamp_bridge_unbind_unit cleared its fleet binding. No saved fleet, an
 * absent/unknown entry, or a module index past the saved set → true (default-bound), so a fresh
 * device (no /rbamp.json yet) still creates all the default units. */
static bool rbamp_channel_bound(const rbamp_bridge_config_t *fleet, uint8_t module_index, uint8_t channel) {
    if (!fleet) return true;
    /* A slot BEYOND the saved fleet is not configured at all (dropped by `rbampcfg count`) — no default
     * units for it (otherwise units of dropped slots would come back at every boot after deletion). */
    if (module_index >= fleet->module_count || module_index >= RBAMP_MAX_MODULES) return false;
    const rbamp_module_cfg_t *mod = &fleet->modules[module_index];
    for (uint8_t e = 0; e < mod->entry_count && e < RBAMP_MAX_ENTRIES; ++e) {
        if (mod->entries[e].source == channel) return mod->entries[e].unit_id != 0;
    }
    return true;
}

// ===== TEMPORARY SOLUTION: Forward declaration for access to internal structures =====
// This is temporary until the API is fully developed
/** @brief Mirror of the private rbgrid config-context layout (rbgrid_config_api), used to reach the built
 *         configuration. Must match the component's definition field for field. */
struct rbgrid_config_context_s {
    rbpower_extended_config_t* extended_config;    ///< extended configuration
    rbpower_hardware_config_t* hardware_config;    ///< hardware configuration
    rbpower_electrical_network_t* network;         ///< electrical network being built
    rbpower_unit_definition_t* units_array;        ///< unit definitions
    uint16_t units_allocated;                      ///< capacity of units_array
    uint16_t units_count;                          ///< used entries of units_array
    struct {
        int next_sensor_id;
        int next_unit_id;
        uint8_t adc_channels_used;
        bool has_voltage_bus;
        bool has_main_supply;
        bool has_alternative_supply;
    } builder;                                     ///< builder state (layout mirror only)
    struct {
        bool base_initialized;
        bool network_configured;
        bool sensors_configured;
        bool units_configured;
    } status;                                      ///< build-step flags
    bool is_initialized;                           ///< context initialized
    bool is_applied;                               ///< configuration applied
};
typedef struct rbgrid_config_context_s rbgrid_config_context_t;   ///< see rbgrid_config_context_s

// (feature flags RBAMP_BRIDGE_ENABLE / RBGRID_RBAMP_FLEET / RBGRID_BENCH_FULL / RBGRID_RBBENCH /
//  RBGRID_WIFI are defined at the top of the file — needed by the early includes.)


// ====== GLOBAL VARIABLES ======
unsigned long last_print = 0;     ///< legacy print timestamp (unused)
bool power_ready = false;         ///< legacy readiness flag (unused)
float last_voltage = 0.0;         ///< legacy last voltage, V (unused)
float last_frequency = 0.0;       ///< legacy last frequency, Hz (unused)
uint32_t last_cycle = 0;          ///< legacy cycle counter (unused)

rbgrid_unit_uid_t heater_sensor_uid;   ///< optional example "heater" unit (RBGRID_INVALID_UNIT_UID = not configured)

// ===== ADD GLOBAL VARIABLES =====
rbpower_electrical_network_t home_network;   ///< the home electrical network built from the stored configuration
static rbpower_extended_config_t* g_extended_config = NULL;
/** Site nominal L-N voltage for consumers outside the rbpower component (ESP-NOW hub: P = V × I for
 * current-only nodes). Reads the live hardware config (hardware-config POST updates it), falls back to
 * 230 V when unset or outside 50..500 V.
 * @return Nominal voltage in V. */
extern "C" float rbgrid_nominal_voltage_v(void)
{
    if (g_extended_config && g_extended_config->base_hardware_config) {
        float v = g_extended_config->base_hardware_config->network.nominal_voltage;
        if (v >= 50.0f && v <= 500.0f) return v;
    }
    return 230.0f;
}
static rbgrid_json_context_t* g_json_ctx = NULL;

bool rbhome_initialized = false;   ///< legacy flag (unused)

// ====== CALLBACKS FOR RBWIFI ======
// Callback for WiFi events
// MQTT topics/ids derived once from the STA MAC: client-id "rbgrid-<mac>",
// telemetry topic "rbgrid/<mac>/telemetry", status/LWT "rbgrid/<mac>/status".
static char s_mqtt_client_id[32]  = {0};
static char s_mqtt_topic[48]      = {0};
static char s_mqtt_lwt_topic[48]  = {0};
static char s_mqtt_cmd_topic[48]  = {0};
static char s_mqtt_resp_topic[56] = {0};

static bool s_mqtt_started = false;


/** Build the mac-derived MQTT identifiers ONCE. Idempotent + safe to call before WiFi connects
 * (esp_read_mac reads efuse, no link needed) — so start_webserver() can build the telemetry config
 * with the canonical "rbgrid/<mac>/telemetry" topic instead of the non-mac fallback. Entity model:
 * identity/transport = mac; per-device topics rbgrid/<mac>/{telemetry,status,cmd}. */
static void rbgrid_build_mqtt_ids(void)
{
    if (s_mqtt_topic[0]) return;   // already built
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char dev[13];
    snprintf(dev, sizeof(dev), "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    snprintf(s_mqtt_client_id, sizeof(s_mqtt_client_id), "rbgrid-%s", dev);
    snprintf(s_mqtt_topic, sizeof(s_mqtt_topic), "rbgrid/%s/telemetry", dev);
    snprintf(s_mqtt_lwt_topic, sizeof(s_mqtt_lwt_topic), "rbgrid/%s/status", dev);
    snprintf(s_mqtt_cmd_topic, sizeof(s_mqtt_cmd_topic), "rbgrid/%s/cmd", dev);
    snprintf(s_mqtt_resp_topic, sizeof(s_mqtt_resp_topic), "rbgrid/%s/cmd/resp", dev);
}


#define OTA_CONFIRM_GRACE_S 60   ///< uptime on the network (s) before a fresh OTA image is marked valid
/** Mark a fresh OTA image valid after OTA_CONFIRM_GRACE_S, then release a deferred stats migration. */
/* esp_timer one-shot (runs in the esp_timer task): no stack of its own, so it can't fail to start when internal RAM
 * is short at boot — a confirm that never runs rolls a good image back at the next reset. */
static void ota_confirm_cb(void *arg)
{
    (void)arg;
    esp_err_t mv = esp_ota_mark_app_valid_cancel_rollback();
    if (mv == ESP_OK) {
        ESP_LOGI(TAG, "OTA image marked VALID (rollback cancelled)");
        rbpower_diag_printf("ota: new image confirmed valid (rollback cancelled)");
    } else {
        rbpower_diag_printf("FAIL ota: confirm failed: %s", esp_err_to_name(mv));
    }
    rbgrid_core_image_confirmed();   /* releases a deferred stats migration */
}

/** Boot: did the bootloader roll back a fresh image? esp_ota_get_last_invalid_partition() stays set until
 * the next OTA, so report each rejected image once (its ELF sha remembered in NVS). */
static void ota_report_rollback(void)
{
    const esp_partition_t *bad = esp_ota_get_last_invalid_partition();
    if (!bad) return;
    esp_app_desc_t d;
    if (esp_ota_get_partition_description(bad, &d) != ESP_OK) return;
    char sha[17];
    for (int i = 0; i < 8; i++) sprintf(&sha[i * 2], "%02x", d.app_elf_sha256[i]);
    nvs_handle_t h;
    if (rbgrid_nvs_open("rbota", NVS_READWRITE, &h) != ESP_OK) return;
    char seen[17] = {0}; size_t n = sizeof(seen);
    if (nvs_get_str(h, "rb_seen", seen, &n) != ESP_OK || strcmp(seen, sha) != 0) {
        const esp_partition_t *run = esp_ota_get_running_partition();
        rbpower_diag_printf("FAIL ota: ROLLED BACK — image in '%s' (elf %s) did not confirm; running '%s'",
                            bad->label, sha, run ? run->label : "?");
        nvs_set_str(h, "rb_seen", sha);
        nvs_commit(h);
    }
    nvs_close(h);
}

/** Image confirm, once the hub is on the network (Wi-Fi up, or at boot when a host owns the network):
 *  a fresh OTA image is marked valid after OTA_CONFIRM_GRACE_S; an already valid one releases a deferred
 *  stats migration at once. */
static void app_image_confirm_on_network(void)
{
    esp_ota_img_states_t ost;
    bool pending = esp_ota_get_state_partition(esp_ota_get_running_partition(), &ost) == ESP_OK &&
                   ost == ESP_OTA_IMG_PENDING_VERIFY;
    if (pending) {
        /* Fresh OTA image: confirm only after OTA_CONFIRM_GRACE_S of uptime on the network — an image that
         * reaches Wi-Fi but crashes/reboots within that window still rolls back (OTA-only hubs have no
         * cable to recover a bad image). One confirm task per boot. */
        static bool s_confirm_started = false;
        if (!s_confirm_started) {
            rbpower_diag_printf("ota: new image on network — confirming in %ds unless it resets", OTA_CONFIRM_GRACE_S);
            /* a failed create would leave a good image unconfirmed → rolled back at the next reset:
             * retry on the next Wi-Fi event instead of marking it started */
            static esp_timer_handle_t s_cf_timer = NULL;
                const esp_timer_create_args_t ta = { .callback = ota_confirm_cb, .arg = NULL,
                                                     .dispatch_method = ESP_TIMER_TASK, .name = "ota_confirm" };
                s_confirm_started = (s_cf_timer || esp_timer_create(&ta, &s_cf_timer) == ESP_OK) &&
                                    esp_timer_start_once(s_cf_timer, (uint64_t)OTA_CONFIRM_GRACE_S * 1000000ull) == ESP_OK;
            if (!s_confirm_started) rbpower_diag_printf("FAIL ota: confirm timer not created (low RAM) — retry on the next Wi-Fi event");
        }
    } else {
        esp_ota_mark_app_valid_cancel_rollback();   /* no-op unless pending; kept for symmetry */
        rbgrid_core_image_confirmed();   /* image already valid: release a deferred stats migration */
    }
}



/** Auto-start the ESP-NOW hub on boot once WiFi is associated, if the registry has nodes. Fixes the recurring
 * gotcha where `espnow on` issued before WiFi-ready returned "ok" but left running=0 (no beacons → nodes can't
 * acquire). A configured hub (≥1 registered node) now brings ESP-NOW up itself — important after an OTA reboot
 * so remote nodes reconnect without a manual verb. One-shot; a manual `espnow off` later still wins. */
/* 500 ms esp_timer (esp_timer task): no stack of its own — a heap-stack task failed to start when internal RAM was
 * short at boot, and the hub then never started. Waits up to ~60 s for STA association (hub channel = router channel). */
static esp_timer_handle_t s_as_timer = NULL;
static void espnow_autostart_cb(void *arg)
{
    (void)arg;
    static int s_tries = 0;
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK && ++s_tries < 120) return;
    esp_timer_stop(s_as_timer);
    espnow_registry_t reg;
    int8_t en = esp_now_hub_enabled_setting();   /* off wins; on starts even without a node (ready to pair) */
    if (!esp_now_hub_running() && en != 0 && (en == 1 || (esp_now_registry_load(&reg) && reg.node_count > 0))) {
        esp_err_t e = esp_now_hub_start(0);
        ESP_LOGW(TAG, "ESP-NOW auto-start (setting %d): %s, running=%d",
                 (int)en, esp_err_to_name(e), (int)esp_now_hub_running());
    }
}

/** ESP-NOW hooks the core calls (MQTT `cmd get/set espnow`, point deletion) and the hub auto-start: every
 *  build with the hub, whatever the transports. The auto-start waits for STA association (ours or the host's). */
static void app_espnow_start(void)
{
    rbgrid_register_espnow_status_provider(esp_now_hub_status_json);   // MQTT `cmd get espnow` → status JSON
    rbgrid_register_unit_deleted_hook(esp_now_on_unit_deleted);        // a deleted node point stays deleted
    rbgrid_register_i2c_action_handler(rbi2c_action);                  // `cmd get|set i2c` → the I2C fleet actions (rbi2c_cmd.c)
    rbgrid_register_espnow_action_handler(esp_now_hub_action);         // MQTT `cmd set espnow` → same actions as HTTP
    rbpower_diag_printf("heap at hub start: internal free %u largest %u min %u, psram free %u",
                        (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                        (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                        (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    const esp_timer_create_args_t ta = { .callback = espnow_autostart_cb, .arg = NULL,
                                         .dispatch_method = ESP_TIMER_TASK, .name = "espnow_as" };
    if (!s_as_timer && esp_timer_create(&ta, &s_as_timer) != ESP_OK) s_as_timer = NULL;
    if (!s_as_timer || esp_timer_start_periodic(s_as_timer, 500000) != ESP_OK)
        rbpower_diag_printf("FAIL espnow: auto-start timer not created — start the hub from the app");
}


/**
 * @brief Test of the period-history API (rbgrid_get_period_history()).
 *
 * Reads the history of unit 0x0004 for every period and logs a summary:
 * - last 48 hours (HOUR)
 * - last 14 days (DAY)
 * - last 8 weeks (WEEK)
 * - last 12 months (MONTH)
 * - last 5 years (YEAR)
 *
 * Debug aid; its call in app_main is commented out (kept for manual testing).
 */
static void test_period_statistics_api(void)
{
    ESP_LOGI("API_TEST", "╔═══════════════════════════════════════════════════════════════╗");
    ESP_LOGI("API_TEST", "║  PERIOD STATISTICS API TEST (rbgrid_get_period_history)        ║");
    ESP_LOGI("API_TEST", "╚═══════════════════════════════════════════════════════════════╝");

    // Unit under test
    const rbpower_unit_id_t test_unit = 0x0004;

    // ═══════════════════════════════════════════════════════════════════════════════
    // TEST 1: last 48 hours (HOUR period)
    // ═══════════════════════════════════════════════════════════════════════════════
    ESP_LOGI("API_TEST", "");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");
    ESP_LOGI("API_TEST", "TEST 1: last 48 hours (HOUR period)");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");

    const uint32_t hour_count = 48;
    rbpower_period_statistics_t* hour_stats = (rbpower_period_statistics_t*)malloc(
        hour_count * sizeof(rbpower_period_statistics_t));

    if (hour_stats) {
        uint32_t actual_count = 0;
        rbgrid_err_t err = rbgrid_get_period_history(
            test_unit,
            RBPOWER_STAT_PERIOD_1HOUR,
            0,  // from the start of the buffer (oldest first)
            hour_count,
            hour_stats,
            &actual_count
        );

        if (err == RBGRID_OK) {
            ESP_LOGI("API_TEST", "✅ Read %lu records HOUR", (unsigned long)actual_count);

            // Totals
            float total_consumed = 0.0f;
            float total_supplied = 0.0f;
            float avg_voltage = 0.0f;
            uint32_t valid_records = 0;

            for (uint32_t i = 0; i < actual_count; i++) {
                total_consumed += hour_stats[i].energy_period.consumed_wh;
                total_supplied += hour_stats[i].energy_period.supplied_wh;
                if (hour_stats[i].voltage_rms > 0.0f) {
                    avg_voltage += hour_stats[i].voltage_rms;
                    valid_records++;
                }
            }

            if (valid_records > 0) {
                avg_voltage /= valid_records;
            }

            ESP_LOGI("API_TEST", "  ├─ Total consumed: %.2f Wh", total_consumed);
            ESP_LOGI("API_TEST", "  ├─ Total supplied: %.2f Wh", total_supplied);
            ESP_LOGI("API_TEST", "  ├─ Average V_rms: %.2f V (from %lu valid records)",
                     avg_voltage, (unsigned long)valid_records);
            ESP_LOGI("API_TEST", "  └─ Time span: %llu - %llu us",
                     (unsigned long long)hour_stats[0].period_start_us,
                     (unsigned long long)hour_stats[actual_count - 1].period_end_us);

            // Show the first 3 and the last 3 records
            ESP_LOGI("API_TEST", "");
            ESP_LOGI("API_TEST", "  First 3 records:");
            for (uint32_t i = 0; i < 3 && i < actual_count; i++) {
                ESP_LOGI("API_TEST", "    [%lu] V=%.1f, consumed=%.2f Wh, supplied=%.2f Wh, completeness=%d%%",
                         (unsigned long)i,
                         hour_stats[i].voltage_rms,
                         hour_stats[i].energy_period.consumed_wh,
                         hour_stats[i].energy_period.supplied_wh,
                         hour_stats[i].data_completeness_percent);
            }

            ESP_LOGI("API_TEST", "  Last 3 records:");
            uint32_t start_idx = (actual_count > 3) ? (actual_count - 3) : 0;
            for (uint32_t i = start_idx; i < actual_count; i++) {
                ESP_LOGI("API_TEST", "    [%lu] V=%.1f, consumed=%.2f Wh, supplied=%.2f Wh, completeness=%d%%",
                         (unsigned long)i,
                         hour_stats[i].voltage_rms,
                         hour_stats[i].energy_period.consumed_wh,
                         hour_stats[i].energy_period.supplied_wh,
                         hour_stats[i].data_completeness_percent);
            }

        } else {
            ESP_LOGE("API_TEST", "❌ Read error HOUR: %d", err);
        }

        free(hour_stats);
    } else {
        ESP_LOGE("API_TEST", "❌ Out of memory for %lu HOUR records",
                 (unsigned long)hour_count);
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // TEST 2: last 14 days (DAY period)
    // ═══════════════════════════════════════════════════════════════════════════════
    ESP_LOGI("API_TEST", "");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");
    ESP_LOGI("API_TEST", "TEST 2: last 14 days (DAY period)");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");

    const uint32_t day_count = 14;
    rbpower_period_statistics_t* day_stats = (rbpower_period_statistics_t*)malloc(
        day_count * sizeof(rbpower_period_statistics_t));

    if (day_stats) {
        uint32_t actual_count = 0;
        rbgrid_err_t err = rbgrid_get_period_history(
            test_unit,
            RBPOWER_STAT_PERIOD_DAILY,
            0,
            day_count,
            day_stats,
            &actual_count
        );

        if (err == RBGRID_OK) {
            ESP_LOGI("API_TEST", "✅ Read %lu records DAY", (unsigned long)actual_count);

            float total_consumed = 0.0f;
            float total_supplied = 0.0f;

            for (uint32_t i = 0; i < actual_count; i++) {
                total_consumed += day_stats[i].energy_period.consumed_wh;
                total_supplied += day_stats[i].energy_period.supplied_wh;
            }

            ESP_LOGI("API_TEST", "  ├─ Total consumed: %.2f Wh (%.3f kWh)",
                     total_consumed, total_consumed / 1000.0f);
            ESP_LOGI("API_TEST", "  └─ Total supplied: %.2f Wh (%.3f kWh)",
                     total_supplied, total_supplied / 1000.0f);

            // Show every daily record
            ESP_LOGI("API_TEST", "");
            ESP_LOGI("API_TEST", "  All daily records:");
            for (uint32_t i = 0; i < actual_count; i++) {
                ESP_LOGI("API_TEST", "    Day [%lu]: consumed=%.2f Wh, supplied=%.2f Wh, cost=%.2f EUR",
                         (unsigned long)i,
                         day_stats[i].energy_period.consumed_wh,
                         day_stats[i].energy_period.supplied_wh,
                         day_stats[i].result_base.cost_consumed);
            }

        } else {
            ESP_LOGE("API_TEST", "❌ Read error DAY: %d", err);
        }

        free(day_stats);
    } else {
        ESP_LOGE("API_TEST", "❌ Out of memory for DAY");
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // TEST 3: last 8 weeks (WEEK period)
    // ═══════════════════════════════════════════════════════════════════════════════
    ESP_LOGI("API_TEST", "");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");
    ESP_LOGI("API_TEST", "TEST 3: last 8 weeks (WEEK period)");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");

    const uint32_t week_count = 8;
    rbpower_period_statistics_t* week_stats = (rbpower_period_statistics_t*)malloc(
        week_count * sizeof(rbpower_period_statistics_t));

    if (week_stats) {
        uint32_t actual_count = 0;
        rbgrid_err_t err = rbgrid_get_period_history(
            test_unit,
            RBPOWER_STAT_PERIOD_WEEKLY,
            0,
            week_count,
            week_stats,
            &actual_count
        );

        if (err == RBGRID_OK) {
            if (actual_count > 0) {
                ESP_LOGI("API_TEST", "✅ Read %lu records WEEK", (unsigned long)actual_count);

                for (uint32_t i = 0; i < actual_count; i++) {
                    ESP_LOGI("API_TEST", "    Week [%lu]: consumed=%.2f Wh, supplied=%.2f Wh",
                             (unsigned long)i,
                             week_stats[i].energy_period.consumed_wh,
                             week_stats[i].energy_period.supplied_wh);
                }
            } else {
                ESP_LOGW("API_TEST", "⚠️  No WEEK data yet (expected on a new device)");
            }
        } else {
            ESP_LOGE("API_TEST", "❌ Read error WEEK: %d", err);
        }

        free(week_stats);
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // TEST 4: last 12 months (MONTH period)
    // ═══════════════════════════════════════════════════════════════════════════════
    ESP_LOGI("API_TEST", "");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");
    ESP_LOGI("API_TEST", "TEST 4: last 12 months (MONTH period)");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");

    const uint32_t month_count = 12;
    rbpower_period_statistics_t* month_stats = (rbpower_period_statistics_t*)malloc(
        month_count * sizeof(rbpower_period_statistics_t));

    if (month_stats) {
        uint32_t actual_count = 0;
        rbgrid_err_t err = rbgrid_get_period_history(
            test_unit,
            RBPOWER_STAT_PERIOD_MONTHLY,
            0,
            month_count,
            month_stats,
            &actual_count
        );

        if (err == RBGRID_OK) {
            if (actual_count > 0) {
                ESP_LOGI("API_TEST", "✅ Read %lu records MONTH", (unsigned long)actual_count);

                for (uint32_t i = 0; i < actual_count; i++) {
                    ESP_LOGI("API_TEST", "    Month [%lu]: consumed=%.2f Wh, supplied=%.2f Wh, cost=%.2f EUR",
                             (unsigned long)i,
                             month_stats[i].energy_period.consumed_wh,
                             month_stats[i].energy_period.supplied_wh,
                             month_stats[i].result_base.cost_consumed);
                }
            } else {
                ESP_LOGW("API_TEST", "⚠️  No MONTH data yet (expected on a new device)");
            }
        } else {
            ESP_LOGE("API_TEST", "❌ Read error MONTH: %d", err);
        }

        free(month_stats);
    }

    // ═══════════════════════════════════════════════════════════════════════════════
    // TEST 5: last 5 years (YEAR period)
    // ═══════════════════════════════════════════════════════════════════════════════
    ESP_LOGI("API_TEST", "");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");
    ESP_LOGI("API_TEST", "TEST 5: last 5 years (YEAR period)");
    ESP_LOGI("API_TEST", "─────────────────────────────────────────────────────────────");

    const uint32_t year_count = 5;
    rbpower_period_statistics_t* year_stats = (rbpower_period_statistics_t*)malloc(
        year_count * sizeof(rbpower_period_statistics_t));

    if (year_stats) {
        uint32_t actual_count = 0;
        rbgrid_err_t err = rbgrid_get_period_history(
            test_unit,
            RBPOWER_STAT_PERIOD_YEARLY,
            0,
            year_count,
            year_stats,
            &actual_count
        );

        if (err == RBGRID_OK) {
            if (actual_count > 0) {
                ESP_LOGI("API_TEST", "✅ Read %lu records YEAR", (unsigned long)actual_count);

                for (uint32_t i = 0; i < actual_count; i++) {
                    ESP_LOGI("API_TEST", "    Year [%lu]: consumed=%.2f Wh (%.3f kWh), supplied=%.2f Wh",
                             (unsigned long)i,
                             year_stats[i].energy_period.consumed_wh,
                             year_stats[i].energy_period.consumed_wh / 1000.0f,
                             year_stats[i].energy_period.supplied_wh);
                }
            } else {
                ESP_LOGW("API_TEST", "⚠️  No YEAR data yet (expected on a new device)");
            }
        } else {
            ESP_LOGE("API_TEST", "❌ Read error YEAR: %d", err);
        }

        free(year_stats);
    }

    ESP_LOGI("API_TEST", "");
    ESP_LOGI("API_TEST", "╔═══════════════════════════════════════════════════════════════╗");
    ESP_LOGI("API_TEST", "║             API TEST FINISHED                                  ║");
    ESP_LOGI("API_TEST", "╚═══════════════════════════════════════════════════════════════╝");
    ESP_LOGI("API_TEST", "");
}


/* NOTE (boot-recovery / GPIO12 VDD_SDIO strap): GPIO12 (MTDI) is BOTH the external NOR-flash MISO and the
 * ESP32 VDD_SDIO flash-voltage strap, so a warm reset (esp_restart/OTA) intermittently bricks (ROM mis-reads
 * the float). A firmware gpio_hold-the-strap mitigation was tried + REVERTED: it didn't reliably fix it AND
 * made recovery sticky (the RTC hold survives an RTS-reset → needs a VCC power-cycle). The real fix is the
 * flash-voltage eFuse (espefuse set_flash_voltage 3.3V → ROM ignores the strap), provisioned per device — see
 * the hardware documentation. No firmware change carries that. */
/* cJSON → PSRAM (under web-app load internal RAM dropped to a few dozen bytes free; 8 parallel HTTP clients
 * reproduce it). Every cJSON node/string is < 16 KB, so with
 * SPIRAM_MALLOC_ALWAYSINTERNAL=16384 they all landed in INTERNAL RAM — a 200-record history document is
 * thousands of nodes. JSON is never touched from ISRs/flash-cache-off code, so PSRAM is safe; fall back to
 * the default heap if PSRAM is exhausted. free() handles both (IDF heap_caps). */
#include "cJSON.h"
/** cJSON allocator: PSRAM first, default heap as fallback. */
static void *rbgrid_cjson_malloc(size_t sz)
{
    void *p = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : malloc(sz);
}
/** cJSON deallocator (free() handles both heaps). */
static void rbgrid_cjson_free(void *p) { free(p); }

/** @def RBPOWER_RUN_INJECT_TESTS
 *  @brief 1 = run the external-source injection self-test at boot (opt-in, default 0; defined in app_main). */

/**
 * @brief Application entry point (ESP-IDF main task; stack 12288, pinned to CPU1 in sdkconfig).
 *
 * Boot order: cJSON hooks → event log → (debug-bridge build) NVS + event loop → Wi-Fi AP+STA → configuration
 * create + storage load → sensor/unit config (saved or default) → time config → rbAmp fleet units → ESP-NOW
 * node units → tariff plan → rbgrid/rbpower init → web server → rbAmp bridge start → event-log storage + boot
 * forensics → debug console verbs. In debug builds rbbench owns
 * UART0; otherwise the IDF console/ESP_LOG serves UART0.
 */
/* ==== Boot stages. app_main() runs them in this order: network (Wi-Fi), core init (storage, config, points,
 * tariffs, statistics engine), interfaces (web server, WebSocket, telemetry; MQTT/mDNS start from the Wi-Fi
 * callback), core start (rbAmp bridge = metering source + period collection). A host other than this
 * application (e.g. ESPHome) would skip the network and interface stages and run the two core stages. ==== */

/** Boot stage 1: Wi-Fi (AP+STA). @return true if Wi-Fi is up (the interface stage needs it). */
static bool app_net_init(void)
{
    /* A host owns Wi-Fi (or there is none). RBgrid still needs NVS, netif and the default event loop;
     * tolerate a host that created them already. */
    esp_err_t ne = nvs_flash_init();
    if (ne == ESP_ERR_NVS_NO_FREE_PAGES || ne == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); ne = nvs_flash_init(); }
    if (ne != ESP_OK) ESP_LOGE(TAG, "NVS init: %s", esp_err_to_name(ne));
    esp_err_t ni = esp_netif_init();
    if (ni != ESP_OK && ni != ESP_ERR_INVALID_STATE) ESP_LOGE(TAG, "netif init: %s", esp_err_to_name(ni));
    esp_err_t el = esp_event_loop_create_default();
    if (el != ESP_OK && el != ESP_ERR_INVALID_STATE) ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(el));
    (void)rbgrid_nvs_init();
    return false;
}

/** Boot stage 2: core init — config store, hardware/points/time, ESP-NOW node points, telemetry config,
 *  flash statistics, rbgrid_init_system + rbpower_init_extended, tariffs. @return extended config, or NULL. */
static rbpower_extended_config_t* app_core_init(void)
{
    printf("\n=== RBgrid ===\n");

    // 1. Start-up banner
    printf("Init rbpower...\n");
    
    ESP_LOGI("MAIN", "=== rbpower + rbhome Full Access Test ===");
    

    
    // ===== NEW APPROACH: Using rbgrid_config_api =====
    // Create configuration through API
    rbgrid_config_handle_t config_handle = rbgrid_config_create("Test House", "HongKong");
    if (!config_handle) {
        ESP_LOGE(TAG, "Failed to create config via rbgrid_config_api");
        return NULL;
    }
    
    // Get pointers to internal structures for additional configuration
    // TEMPORARY SOLUTION: direct access to structures until full API implementation
    rbgrid_config_context_t* config_ctx = (rbgrid_config_context_t*)config_handle;
    rbpower_extended_config_t* extended_config = config_ctx->extended_config;
    rbpower_hardware_config_t* base_hardware_config = config_ctx->hardware_config;
    
    // Additional settings not yet covered by API
    base_hardware_config->network.nominal_frequency = 50;  // Default 50Hz (hardware-config / saved config override it)
    
    // Initialize JSON module early for storage operations
    ESP_LOGI(TAG, "Initializing JSON module early for storage tests...");
    static rbgrid_json_context_t json_ctx = {};
    rbgrid_json_device_info_t json_info = {
        .device_id = "ESP32-RBGrid-001",
        .device_name = "RBGrid Storage Test",
        .location = "Test Location",
        .timezone = "UTC",
    };
    rbgrid_json_init(&json_ctx, &json_info);
    g_json_ctx = &json_ctx;
    ESP_LOGI(TAG, "JSON module initialized successfully");
    
    // Initialize configuration storage
    ESP_LOGI(TAG, "Initializing configuration storage...");
    esp_err_t storage_ret = rbgrid_storage_init();
    if (storage_ret == ESP_OK) {
        // Try to load existing configuration into global context
        if (rbgrid_storage_load_system(extended_config) == ESP_OK) {
            ESP_LOGI(TAG, "Configuration loaded from external Flash to global context");
        } else {
            ESP_LOGI(TAG, "No stored configuration found, saving current settings...");
            // Save current configuration as default
            if (rbgrid_storage_save_system(extended_config) == ESP_OK) {
                ESP_LOGI(TAG, "Default configuration saved to external Flash");
            }
        }
        
        // Check storage status
        rbgrid_storage_check_files();
        
        // Show storage statistics
        size_t total = 0, used = 0;
        if (rbgrid_storage_get_stats(&total, &used) == ESP_OK) {
            ESP_LOGI(TAG, "Storage: %zu KB used of %zu KB total", used/1024, total/1024);
        }
    } else {
        ESP_LOGW(TAG, "Configuration storage not available: %s", esp_err_to_name(storage_ret));
    }


    // === SENSOR + UNIT config: load-or-default-and-save ===
    // If the configurator (web app) saved BOTH the sensor layer (/hardware.json) AND the unit layer
    // (/units.json), restore them: load_hardware brings back the sensor channel definitions, load_units
    // recreates the units + supplies (by type/role, deriving sensor UIDs from the saved channels). The
    // hardcoded supply block below self-gates on the sensor UIDs — when loaded the UIDs stay INVALID
    // so that block is skipped (load_units already made the supplies). Otherwise (first boot) build
    // the hardcoded defaults; they're persisted after init (below) so the NEXT boot loads them.
    // NOTE: rbAmp units (add_rbamp_module, further down) are ALWAYS built from the fleet config — they
    // are not in units.json by design (load_units skips I2C), so they must not be gated. Filtering stays
    // separate (always). No hardcode-fallback on a partial load (would double-add).
    rbgrid_unit_uid_t voltage_sensor_uid = RBGRID_INVALID_UNIT_UID;
    rbgrid_unit_uid_t current_sensor_uid = RBGRID_INVALID_UNIT_UID;
    bool have_saved_hw_units = littlefs_raw_exists("/hardware.json") && littlefs_raw_exists("/units.json");
    if (have_saved_hw_units) {
        esp_err_t lh = rbgrid_storage_load_hardware(base_hardware_config);
        /* Repair: a /hardware.json saved by a build that did not write the ADC-typed definitions has none, and
         * load_units then drops the default units 0x0006/0x0007 that bind to them. Put the two legacy
         * definitions back (no ADC is sampled; the rbAmp bridge feeds these units) and re-save the file. */
        if (lh == ESP_OK && base_hardware_config->adc.channels_count == 0) {
            rbgrid_unit_uid_t v = rbgrid_config_add_adc_sensor(config_handle, GPIO_NUM_35, RBSENSOR_TYPE_VOLTAGE_ZMPT107_1);
            rbgrid_unit_uid_t c = rbgrid_config_add_adc_sensor(config_handle, GPIO_NUM_34, RBSENSOR_TYPE_CURRENT_SCT013_10A);
            esp_err_t sh = rbgrid_storage_save_hardware(base_hardware_config);
            rbpower_diag_printf("config: /hardware.json had no sensor definitions for the default units — restored "
                                "(0x%04X, 0x%04X), saved: %s", (unsigned)v, (unsigned)c, esp_err_to_name(sh));
        }
        esp_err_t lu = rbgrid_storage_load_units(config_handle);
        ESP_LOGI(TAG, "✅ Sensor+unit config LOADED from flash (hardware:%s units:%s) — supplies recreated by load_units",
                 esp_err_to_name(lh), esp_err_to_name(lu));
    } else {
        ESP_LOGI(TAG, "=== No saved hardware+units — building DEFAULT sensors (will persist after init) ===");
        // Default voltage unit: legacy ADC-typed definition (ZMPT107-1, GPIO35) — no on-chip ADC is sampled,
        // it only binds the default unit that the rbAmp bridge feeds
        voltage_sensor_uid = rbgrid_config_add_adc_sensor(config_handle, GPIO_NUM_35, RBSENSOR_TYPE_VOLTAGE_ZMPT107_1);
        if (voltage_sensor_uid == RBGRID_INVALID_UNIT_UID) ESP_LOGE(TAG, "Failed to add voltage sensor");
        else ESP_LOGI(TAG, "Voltage sensor added with unit_UID: 0x%04X", voltage_sensor_uid);
        // Default main-supply current unit: legacy ADC-typed definition (SCT013-10A, GPIO34), fed by rbAmp
        current_sensor_uid = rbgrid_config_add_adc_sensor(config_handle, GPIO_NUM_34, RBSENSOR_TYPE_CURRENT_SCT013_10A);
        if (current_sensor_uid == RBGRID_INVALID_UNIT_UID) ESP_LOGE(TAG, "Failed to add main current sensor");
        else ESP_LOGI(TAG, "Main current sensor added with unit_UID: 0x%04X", current_sensor_uid);
    }
    
    /* The default alternative-supply (0x0005) and heater (0x0004) units are not created: only
     * 0x0006 (main_supply) / 0x0007 (voltage_bus) remain as defaults, fed by the rbAmp inject.
     * Further sources come from the rbAmp fleet (add_rbamp_module) and ESP-NOW nodes. */
    rbgrid_unit_uid_t alt_sensor_uid = RBGRID_INVALID_UNIT_UID;
    heater_sensor_uid = RBGRID_INVALID_UNIT_UID;
    int config_err = RBGRID_CONFIG_OK;

    // Filtering configuration (remains manual)
    base_hardware_config->adc_sensors.enable_filtering = false;
    base_hardware_config->adc_sensors.filter_samples = 1;
    
    // === TIME CONFIGURATION LOAD FROM FLASH ===
    ESP_LOGI(TAG, "=== Loading time configuration from Flash ===");

    // Try to load saved time configuration
    if (littlefs_raw_exists("/time.json")) {
        ESP_LOGI(TAG, "Found /time.json file, attempting to load...");

        esp_err_t time_load_ret = rbgrid_storage_load_time(config_handle);
        if (time_load_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Time configuration loaded and applied from Flash");
            ESP_LOGI(TAG, "Time settings will override defaults from main.cpp");
        } else if (time_load_ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "Time configuration file not found, using defaults");
        } else {
            ESP_LOGE(TAG, "Failed to load time configuration: %s", esp_err_to_name(time_load_ret));
            ESP_LOGW(TAG, "Continuing with default time settings");
        }
    } else {
        ESP_LOGI(TAG, "No /time.json file found, using default time configuration");
 
        // Time configuration through API functions
        /* Fresh device: UTC until the user sets a zone (setup / time-sync save_config). The default must be a
         * POSIX TZ string with an offset ("UTC0"): a name without an offset is rejected by the timezone
         * validator ("timezone rejected" on every new hub). */
        config_err = rbgrid_config_set_sntp_time(config_handle,
                                                "pool.ntp.org",
                                                "time.nist.gov",
                                                "UTC0");
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGW(TAG, "Failed to configure SNTP time: %d", config_err);
        } else {
            ESP_LOGI(TAG, "✅ SNTP time configuration completed");
        }
        
        // Time fallback configuration
        config_err = rbgrid_config_set_time_fallback(config_handle,
                                                    true,   // enable RTC fallback
                                                    true,   // enable manual fallback
                                                    1704067020); // January 1, 2024, 07:57:00 UTC = December 31, 23:57 local (UTC-8)
                                                                 // → 3 min later DAY + WEEK(Mon) + MONTH(1st) + YEAR(Jan 1) all roll over at once
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGW(TAG, "Failed to configure time fallback: %d", config_err);
        }
        
        // Advanced time settings
        config_err = rbgrid_config_set_time_advanced(config_handle,
                                                    3600,   // sync every hour
                                                    10000,  // 10 second timeout
                                                    3,      // 3 retry attempts
                                                    true);  // enable DST
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGW(TAG, "Failed to configure advanced time settings: %d", config_err);
        }
    }
    
    // ESP-NOW settings remain in direct access until API is available
    memset(base_hardware_config->time_config.esp_now_hub_mac, 0, 6);
    base_hardware_config->time_config.esp_now_timeout_ms = 1000;
    base_hardware_config->time_config.on_time_sync_status_changed = NULL;
    
    // External flash is now initialized automatically through rbpower_external_flash_init()
    // base_hardware_config->external_flash_config = &external_flash_config;  // DEPRECATED - now using consolidated Flash manager

    // Set Flash status flag for rbpower modules
    // Note: external_flash_is_available() checks both initialized AND otp_validated
    // For development/testing, we check only if config is mounted (Flash is physically working)
    if (external_flash_is_config_mounted()) {
        base_hardware_config->external_flash.status.initialized = true;
        ESP_LOGI(TAG, "External Flash status set: initialized = true (config mounted)");
    } else {
        base_hardware_config->external_flash.status.initialized = false;
        ESP_LOGW(TAG, "External Flash config not mounted, status.initialized = false");
    }

   
    // 1. Create Voltage Bus using voltage sensor unit_UID
    if (voltage_sensor_uid != RBGRID_INVALID_UNIT_UID) {
        config_err = rbgrid_config_set_voltage_bus(config_handle, voltage_sensor_uid, "Main Voltage Bus");
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGE(TAG, "Failed to configure voltage bus: %d", config_err);
        } else {
            ESP_LOGI(TAG, "✅ Voltage bus configured successfully with unit_UID: 0x%04X", voltage_sensor_uid);
        }
    }
    
    // 2. Create Main Supply using current sensor unit_UID
    if (current_sensor_uid != RBGRID_INVALID_UNIT_UID) {
        // Main supply = the primary grid feed → FULL statistics depth (3SEC..YEARLY incl. day/week/
        // month/year), matching the role intent RBPOWER_AUTO_PERIODS_BY_ROLE(MAIN_SUPPLY)=COMPLETE.
        // Main units need long-period history, so the depth is not capped at 1HOUR. NOTE: day/week+
        // records finalize on a wall-clock boundary hit.
        config_err = rbgrid_config_set_main_supply(config_handle, current_sensor_uid, "Main Supply Current", RBPOWER_STAT_PERIOD_YEARLY);
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGE(TAG, "Failed to configure main supply: %d", config_err);
        } else {
            ESP_LOGI(TAG, "✅ Main supply configured successfully with unit_UID: 0x%04X", current_sensor_uid);
        }
    }
    

    // Alternative Supply (0x0005) + Heater (0x0004) units are skipped while no source is
    // defined for them (see above) — no orphan units. Guarded so they can be re-enabled
    // cleanly when a source is assigned.
    if (alt_sensor_uid != RBGRID_INVALID_UNIT_UID) {
        // Alt supply = a primary source too → FULL depth like main supply (was 1HOUR). See note above.
        config_err = rbgrid_config_set_alternative_supply(config_handle, alt_sensor_uid, "Alternative Supply", RBPOWER_STAT_PERIOD_YEARLY);
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGE(TAG, "Failed to set alternative supply: %d", config_err);
        } else {
            ESP_LOGI(TAG, "✅ Alternative supply configured");
        }
    }

    if (heater_sensor_uid != RBGRID_INVALID_UNIT_UID) {
        config_err = rbgrid_config_add_unit(config_handle, heater_sensor_uid, "Heater 10A", RBPOWER_STAT_PERIOD_1HOUR);
        if (config_err != RBGRID_CONFIG_OK) {
            ESP_LOGE(TAG, "Failed to set Heater: %d", config_err);
        } else {
            ESP_LOGI(TAG, "✅ Heater configured");
        }
    }


    /* ESP-NOW nodes: register from the PERSISTED registry (/espnow_nodes.bin) as STANDARD config
     * sensor-units → full registration at init (flash buffers, source mappings, accumulators), like a local
     * rbAmp. Paired/added nodes (espnow addnode verb / over-the-air pairing) live in the registry, which
     * persists. An empty registry stays empty — nodes join via `espnow pair` (over the air) or
     * `espnow addnode`. */
    {
        espnow_registry_t ereg;
        esp_now_registry_load(&ereg);
        if (ereg.node_count == 0)
            ESP_LOGW(TAG, "ESP-NOW registry empty → no nodes (use `espnow pair` to add over the air)");
        int total = 0;
        for (int nn = 0; nn < ereg.node_count; nn++) {
            const espnow_reg_node_t* nd = &ereg.nodes[nn];
            rbgrid_rbamp_channel_t chs[ESPNOW_REG_MAX_CH];
            for (int c = 0; c < nd->ch_count; c++) {
                rbsensor_type_t st;
                switch (nd->channels[c].ct_code) {
                    case 3: st = RBSENSOR_TYPE_CURRENT_SCT013_30A; break;
                    case 4: st = RBSENSOR_TYPE_CURRENT_SCT013_30A; break;  /* no 50A type → 30A */
                    case 6: st = RBSENSOR_TYPE_CURRENT_SCT013_20A; break;
                    default: st = RBSENSOR_TYPE_CURRENT_SCT013_10A; break; /* 1/2 → 10A */
                }
                chs[c].channel = nd->channels[c].ch;
                chs[c].sensor_type = st;
                chs[c].name = NULL;   /* auto-name */
            }
            rbgrid_config_add_espnow_module(config_handle, nd->mac, chs, nd->ch_count);
            total += nd->ch_count;
        }
        ESP_LOGW(TAG, "ESP-NOW nodes from registry: %d node(s), %d unit(s)", ereg.node_count, total);
    }
/*
  
    // Test loading units configuration from Flash
    ESP_LOGI(TAG, "=== Testing units configuration load from Flash ===");
    
    // First, let's check if units.json file exists
    if (littlefs_raw_exists("/units.json")) {
        ESP_LOGI(TAG, "Found /units.json file, attempting to load...");
        
        if (config_handle) {
            // Load units configuration from file
            esp_err_t load_ret = rbgrid_storage_load_units(config_handle);
            if (load_ret == ESP_OK) {
                ESP_LOGI(TAG, "✅ Units configuration loaded successfully from Flash");
            } else if (load_ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "Units configuration file not found");
            } else {
                ESP_LOGE(TAG, "Failed to load units configuration: %s", esp_err_to_name(load_ret));
            }
        } else {
            ESP_LOGE(TAG, "Failed to create test config handle");
        }
    } else {
        ESP_LOGW(TAG, "No /units.json file found, skipping load test");
    }
    
    ESP_LOGI(TAG, "=== End of units configuration load test ===");
*/
 
    // === TARIFF PLAN ===
    // The saved plan (/tariffs.json) is applied AFTER rbpower_init_extended(): the tariff engine
    // init installs the built-in placeholder plan, so a plan loaded here was overwritten on every
    // boot (fixed in 1.1.5). The old builder-based load here also dropped the plan name, zone types
    // and weekday masks, and seeded a "Builder Plan DE" file that never became live. Both removed;
    // see rbgrid_storage_reapply_tariffs() after rbpower_init_extended() below.

    // Or configure individually (commented for example):
    // Basic telemetry settings
    rbgrid_config_set_telemetry_basic(config_handle, true, 4096, true);
    
    // Configure MQTT channel
    rbgrid_config_set_telemetry_mqtt(config_handle, false, 
                                     "mqtt://localhost:1883", 
                                     "rbpower/measurements", 
                                     1, 3000);
    
    // Configure HTTP channel  
    rbgrid_config_set_telemetry_http(config_handle, false,
                                     "https://api.example.com/rbpower/data",
                                     "your-api-token",
                                     15000);
    
    // Configure WebSocket channel
    rbgrid_config_set_telemetry_websocket(config_handle, true,
                                          "",   /* server URL: informational only (the hub itself serves WS) */
                                          8080,
                                          3000);

    
    // Additional extended_config settings through obtained pointer
    extended_config->rbhome_settings.logging_level = RBGRID_LOG_LEVEL_DEBUG;
    extended_config->enable_time_synchronization = true;
    
    // Configure auto-start measurements (default true)
    extended_config->auto_start_measurements = true;  // Automatically start measurements after initialization
    
    // Pass units array created through API functions
    extended_config->units_config.predefined_units = config_ctx->units_array;
    extended_config->units_config.predefined_units_count = config_ctx->units_count;
    extended_config->units_config.use_predefined_only = true;  // Use only predefined
    
    // Set global pointer
    g_extended_config = extended_config;
    rbgrid_core_set_context(extended_config, g_json_ctx);   /* the core's builders (dispatcher, units) use these */
    size_t extflash_total = 0, extflash_used = 0;
    if (rbgrid_storage_get_stats(&extflash_total, &extflash_used) == ESP_OK) {
            ESP_LOGI(TAG, "Storage: %zu KB used of %zu KB total", extflash_used/1024, extflash_total/1024);
    }
//    esp_err_t HRW_ret = rbgrid_storage_save_hardware(config_ctx->hardware_config);

//    esp_err_t HRW_ret= rbgrid_storage_load_hardware(config_ctx->hardware_config);

    // ===== FLASH MODULE TESTS AND INITIALIZATION =====
    // IMPORTANT: must stay here, AFTER the units are configured and before rbgrid_init_system()
    if (rbpower_flash_is_available()) {
         ESP_LOGI(TAG, " ===  FLASH MODULES TESTS  ===");
 
        // Test 1: JSON configuration (old version)
        ESP_LOGI(TAG, "Test 1: Flash config (old version)...");
        rbpower_flash_config_init();
        rbpower_flash_config_test();
        ESP_LOGI(TAG, "");

        // Test 2: runtime configuration (optimized version)
        ESP_LOGI(TAG, "Test 2: Runtime config (optimized)...");
        rbpower_runtime_config_test();
        ESP_LOGI(TAG, "");

        // Test 3: load all JSON configs
        ESP_LOGI(TAG, "Test 3: Load all JSON configs...");
        rbpower_runtime_config_manager_t test_manager;
        if (rbpower_runtime_config_init(&test_manager) == ESP_OK) {
            esp_err_t load_ret = rbpower_runtime_config_load_all(&test_manager);
            if (load_ret == ESP_OK) {
                ESP_LOGI(TAG, "✓ load_all() succeeded:");
                rbpower_runtime_config_dump(&test_manager);
            } else if (load_ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "No config files found (this is OK for first run)");
            } else {
                ESP_LOGE(TAG, "✗ load_all() failed: %s", esp_err_to_name(load_ret));
            }
            rbpower_runtime_config_deinit(&test_manager);
        }
        ESP_LOGI(TAG, "");

        // Test 4: flash operations (record write/read through sector markers)
        ESP_LOGI(TAG, "Test 4: Flash operations (Sector Markers)...");
        rbpower_flash_operations_init();
    } else {
        ESP_LOGW(TAG, "===  FLASH MODULES TESTS SKIPPED (Flash not available)  ===");
    }
   
    // Initialize rbgrid system (unit_UID mapping, NVS)

    ESP_LOGI("MAIN", "Initializing rbgrid system...");
    rbgrid_err_t rbgrid_err = rbgrid_init_system();
    if (rbgrid_err != RBGRID_OK) {
        ESP_LOGE("MAIN", "Failed to initialize rbgrid system: %d", rbgrid_err);
        ESP_LOGW("MAIN", "Unit UID mapping features may be limited");
    } else {
        ESP_LOGI("MAIN", "✅ rbgrid system initialized successfully");
    }
    
  
    // Initialize with home network
    rbpower_err_t power_err = rbpower_init_extended(extended_config);
    if (power_err != RBPOWER_OK) {
        ESP_LOGE("MAIN", "Failed to initialize extended rbpower: %d", power_err);
        ESP_LOGW("MAIN", "Continuing with limited functionality");
    }

    // Saved tariff plan: applied now that the tariff engine is initialized (its init installs the
    // placeholder plan). Same parser as a live POST /api/system/tariff-plan-config.
    {
        esp_err_t tr = rbgrid_storage_reapply_tariffs();
        ESP_LOGI("MAIN", "Saved tariff plan: %s", tr == ESP_OK ? "applied" : esp_err_to_name(tr));
    }
    if (have_saved_hw_units) {   /* ESP-NOW node points come back from the registry with default fields */
        esp_err_t er = rbgrid_storage_reapply_espnow_units();
        ESP_LOGI("MAIN", "ESP-NOW point edits: %s", esp_err_to_name(er));
    }

    // Persist the DEFAULT sensor+unit config on FIRST boot (no saved files) so it reloads next boot
    // and the web app can edit from it. If config was LOADED from flash, do NOT re-save (preserve user
    // edits). g_home_network is fully built by now → save_units(NULL) serializes the live units.
    if (!have_saved_hw_units) {
        esp_err_t sh = rbgrid_storage_save_hardware(base_hardware_config);
        esp_err_t su = rbgrid_storage_save_units(NULL);
        ESP_LOGI("MAIN", "Default sensor+unit config persisted (hardware:%s units:%s)",
                 esp_err_to_name(sh), esp_err_to_name(su));
    } else {
        ESP_LOGI("MAIN", "Sensor+unit config was loaded from flash — not re-saving (preserve edits)");
    }

    // JSON context already initialized earlier
    return extended_config;
}

/** Boot stage 3: interfaces (web server + WebSocket + telemetry + ESP-NOW HTTP), only with Wi-Fi up. */
static void app_ifaces_start(rbpower_extended_config_t* extended_config, bool wifi_initialized)
{
    if (extended_config->rbhome_JSON_settings.enable_json_api) {
        // Update JSON context if needed for production use
        ESP_LOGI(TAG, "JSON module already initialized, updating for production use");
        if (g_json_ctx) {
            // Update device info if needed
            g_json_ctx->device_info.device_id[0] = '\0';
            strlcpy(g_json_ctx->device_info.device_id, "rbgrid_main_001", sizeof(g_json_ctx->device_info.device_id));
            strlcpy(g_json_ctx->device_info.device_name, "MainPowerUnit", sizeof(g_json_ctx->device_info.device_name));
            strlcpy(g_json_ctx->device_info.location, "Main Panel", sizeof(g_json_ctx->device_info.location));
        }
/*
        // Save units configuration to Flash
        ESP_LOGI(TAG, "=== Saving units configuration to Flash ===");
        esp_err_t save_ret = rbgrid_storage_save_units(NULL);
        if (save_ret == ESP_OK) {
            ESP_LOGI(TAG, "✅ Units configuration saved successfully");
        } else {
            ESP_LOGE(TAG, "Failed to save units configuration: %s", esp_err_to_name(save_ret));
        }

*/        
        // start web server if WiFi is initialized
        (void)wifi_initialized;
    }
    
    // ===== TEST 5: Network structure =====
}

/** Boot stage 4: core start — the rbAmp bridge (metering source) and the 15-min period collection. */
static void app_core_start(void)
{
    rbpower_home_print_network_structure();

    // ===== TEST 6 (opt-in): external-source injection API self-test =====
    // Build with -DRBPOWER_RUN_INJECT_TESTS=1 to run on-target (a test script parses the
    // "RBTEST inject SUMMARY: <P> passed, <F> failed" UART line). 0x0004 is a unit
    // registered above by the demo config (same unit used by test_period_statistics_api).
#ifndef RBPOWER_RUN_INJECT_TESTS
/* 1 = run the injection self-test at boot (opt-in, default 0) */
#define RBPOWER_RUN_INJECT_TESTS 0  /* opt-in: build with -DRBPOWER_RUN_INJECT_TESTS=1 to run on-target */
#endif

    // ===== rbAmp I2C metering bridge bring-up =====
    // RBAMP_BRIDGE_ENABLE (default 1) probes + polls the rbAmp modules on I2C (default: mains UI1 @0x50).
    // NOTE: check the SDA/SCL pins of the rbAmp bus in the fleet config for your hardware.
    /* Start the rbAmp fleet as the source (RBGRID_RBAMP_FLEET — debug-bridge or wifi build). */
    /* rbpower_stop() halts the built-in measurement engine, so the rbAmp injection is the SOLE feeder of units
     * 0x0006/0x0007 (the external-source path); rbAmp data shows via telemetry / `unit 0006`. */
    rbpower_stop();
    {
        /* Default config = single module @0x50 (voltage->0x0007, ch0 10A code2 ->0x0006). */
        rbamp_bridge_config_t rbamp_cfg = RBAMP_BRIDGE_DEFAULT_CONFIG();
        rbamp_cfg.inject_enabled = true;     // feed rbAmp readings into rbhome stats
        rbamp_cfg.poll_period_ms = 1000;     // 1s read. NOTE: the 3-sec period stats (what MQTT/
                                             // telemetry serialize) are built by the 200ms-cadence
                                             // accumulator, which SKIPS the first sample per setup
                                             // and needs several samples/period — at 3000ms there
                                             // was only 1 inject/period (skipped) → period stats 0.
                                             // 1000ms gives ~3 injects/period → non-zero. (~5x
                                             // fewer injects than a 200ms cadence.) Possible follow-on:
                                             // rbAmp 3-sec should write period stats directly
                                             // (device RMS) instead of SoS-accumulating, like the
                                             // 15-min inject_period_result path.
        rbamp_cfg.current_gain   = 1.0f;     // no host cal (per-unit gains in module flash)
        rbamp_cfg.voltage_gain   = 1.0f;     // product default: factory-calibrated modules, no host trim
        /* product default fleet: one module at 0x50 (RBAMP_BRIDGE_DEFAULT_CONFIG); more modules are added by
         * the user (`rbampcfg`). */

        /* Config persistence: prefer the saved fleet config (/rbamp.json) if present; otherwise persist
         * THIS default so it survives reboot and can be edited later (the /api/v1/i2c API or the
         * `rbampcfg` verb). First boot saves the 3-module default fleet, subsequent boots load it. */
        rbamp_bridge_config_t rbamp_loaded;
        if (rbgrid_storage_load_rbamp(&rbamp_loaded) == ESP_OK) {
            rbamp_cfg = rbamp_loaded;
            ESP_LOGI("MAIN", "rbAmp fleet config LOADED from flash (%u modules)",
                     (unsigned)rbamp_cfg.module_count);
        } else {
            esp_err_t se = rbgrid_storage_save_rbamp(&rbamp_cfg);
            ESP_LOGI("MAIN", "rbAmp fleet config persisted (default fleet, %u module(s)): %s",
                     (unsigned)rbamp_cfg.module_count, esp_err_to_name(se));
        }


        esp_err_t rbamp_err = rbamp_bridge_start(&rbamp_cfg);
        ESP_LOGI("MAIN", "rbAmp bridge start: %s", esp_err_to_name(rbamp_err));
        /* Auto-collect rbAmp periods on the rbpower 15-min boundary (every build — energy statistics depend on it). */
        rbpower_register_period_boundary_cb(rbgrid_rbamp_period_cb);
        ESP_LOGI("MAIN", "rbAmp auto 15-min period collection armed");
    }

}

/** Boot sequence. @p hosted: a host (ESPHome) owns Wi-Fi, OTA and image confirmation; only the core stages run
 *  and the host calls rbgrid_core_image_confirmed() when it decides the image is good. @return false if the core
 *  could not start. */
static bool rbgrid_boot(bool hosted) {
    { cJSON_Hooks h = { rbgrid_cjson_malloc, rbgrid_cjson_free }; cJSON_InitHooks(&h); }   /* before any JSON */
    /* factory reset (POST /api/system/factory-reset) runs here, before NVS and the config file system open */
    const bool factory_reset_done = rbgrid_core_factory_reset_at_boot();
    rbevlog_init();   /* event log ring + diag log sink: before the first diag line */
    if (factory_reset_done) rbpower_diag_printf("factory reset carried out at boot — starting as on first boot");



    // NOTE: the flash module tests run after rbgrid_config_create(), after the flash init and before
    // rbgrid_init_system() (see the flash test block further down).

    const bool wifi_initialized = app_net_init();
    rbpower_extended_config_t* extended_config = app_core_init();
    if (!extended_config) return false;
    app_ifaces_start(extended_config, wifi_initialized);
    app_core_start();
    app_espnow_start();
    if (!hosted) app_image_confirm_on_network();   /* no Wi-Fi callback in this build: confirm from boot */

    /* Event log files (after rbpower storage init — it may own the `logs` mount) + boot forensics in EVERY
     * build: the classification feeds system.boot and the log; debug builds also echo it on the console. */
    rbevlog_attach_storage();
    static char s_boot_line[200];
    rbpower_boot_check(s_boot_line, sizeof(s_boot_line));
    rbpower_diag_printf("%s", s_boot_line);
    ota_report_rollback();


    ESP_LOGI("MAIN", "=====Setup complete!=======");

    // After rbpower_flash_init_buffers_from_network()
 //       vTaskDelay(pdMS_TO_TICKS(5000));  // wait a few seconds for data to be written
/*
        ESP_LOGI("MAIN", "=== Testing Flash buffer reading ===");

        // Test 1: read the last 10 records of unit 0x0004 (single record read)
        ESP_LOGI("MAIN", "--- Test 1: Single record read (recent 10) ---");
        rbpower_flash_test_read_recent(0x0004, RBPOWER_PERIOD_HOUR, RBPOWER_DATA_TYPE_ELECTRICAL, 10);

        // Test 2: batch read of records 0-9 of unit 0x0004
        ESP_LOGI("MAIN", "--- Test 2: Batch read [0..9] ---");
        rbpower_flash_test_read_batch(0x0004, RBPOWER_PERIOD_HOUR, RBPOWER_DATA_TYPE_ELECTRICAL, 0, 10);

        // Test 3: batch read of records 10-19 of unit 0x0004 (older records)
        ESP_LOGI("MAIN", "--- Test 3: Batch read [10..19] ---");
        rbpower_flash_test_read_batch(0x0004, RBPOWER_PERIOD_HOUR, RBPOWER_DATA_TYPE_ELECTRICAL, 10, 10);

        // ═══════════════════════════════════════════════════════════════════════════════
        // PERIOD HISTORY API TEST: rbgrid_get_period_history()
        // ═══════════════════════════════════════════════════════════════════════════════
        ESP_LOGI("MAIN", "");
        ESP_LOGI("MAIN", "=== Running the period statistics API test ===");
        vTaskDelay(pdMS_TO_TICKS(1000)); // delay for log readability

        test_period_statistics_api();
*/
    (void)hosted;
    return true;
}

// ====== LOOP ======
/** Dead reference code (an old main-loop body, entirely inside while(0)).
 * Kept as documentation of the realtime read API; never called. */
__attribute__((unused)) static void demo_readout_loop(void) {
    // Main loop with periodic status output
    while (0) {
        
        // User code: get voltage statistics every 10 seconds
            statistics_3sec_final_t voltage_stats;
            rbgrid_err_t result = rbgrid_realtime_get_voltage(&voltage_stats);
            
            if (result == RBGRID_OK) {
                ESP_LOGI(TAG, "Voltage: %.2f V, Frequency: %.2f Hz, THD: %.2f%%",
                         voltage_stats.voltage_rms_final,
                         voltage_stats.frequency_avg_final,
                         voltage_stats.thd_voltage_avg_final);
            } else {
                ESP_LOGE(TAG, "Failed to get voltage statistics: %d", result);
            }
        
            
            // Get real-time Main Supply status (200ms data)
            float current_rms, power_active, power_reactive, power_apparent, power_factor, energy_increment;
            uint32_t samples_count;
            rbpower_current_direction_t direction;
            result = rbgrid_realtime_get_main_supply(&current_rms, &power_active, &direction,
                                                                      &power_reactive, &power_apparent, &power_factor,
                                                                      &samples_count, &energy_increment);
            
            if (result == RBGRID_OK) {
                const char* direction_str = rbpower_current_direction_to_string(direction);
                ESP_LOGI(TAG, "Main Supply Status: Current: %.3f A, Active: %.1f W, Reactive: %.1f VAR, Apparent: %.1f VA, PF: %.3f, Samples: %lu, Energy: %.3f Wh, Direction: %s",
                         current_rms, power_active, power_reactive, power_apparent, power_factor, samples_count, energy_increment, direction_str);
            } else {
                ESP_LOGE(TAG, "Failed to get main supply status: %d", result);
            }
            
            // Get real-time Alternative Supply status (200ms data)
            float alt_current_rms, alt_power_active, alt_power_reactive, alt_power_apparent, alt_power_factor, alt_energy_increment;
            float alt_efficiency, alt_dc_power;
            uint32_t alt_samples_count;
            rbpower_current_direction_t alt_direction;
            result = rbgrid_realtime_get_alt_supply(&alt_current_rms, &alt_power_active, &alt_direction,
                                                                         &alt_power_reactive, &alt_power_apparent, &alt_power_factor,
                                                                         &alt_samples_count, &alt_energy_increment,
                                                                         &alt_efficiency, &alt_dc_power);
            
            if (result == RBGRID_OK) {
                const char* alt_direction_str = rbpower_current_direction_to_string(alt_direction);
                ESP_LOGI(TAG, "Alt Supply Status: Current: %.3f A, Active: %.1f W, Reactive: %.1f VAR, Apparent: %.1f VA, PF: %.3f, Samples: %lu, Energy: %.3f Wh, Efficiency: %.1f%%, DC: %.1f W, Direction: %s",
                         alt_current_rms, alt_power_active, alt_power_reactive, alt_power_apparent, alt_power_factor, alt_samples_count, alt_energy_increment, alt_efficiency, alt_dc_power, alt_direction_str);
            } else {
                ESP_LOGE(TAG, "Failed to get alternative supply status: %d", result);
            }
            
            // Get 200ms data of regular unit (Heater) by unit_UID
            float unit_current, unit_power_active, unit_power_reactive, unit_power_apparent, unit_power_factor;
            uint32_t unit_samples;
            result = rbgrid_realtime_get_unit(heater_sensor_uid, &unit_current, 
                                                                    &unit_power_active, &unit_power_reactive, 
                                                                    &unit_power_apparent, &unit_power_factor, 
                                                                    &unit_samples);
            
            if (result == RBGRID_OK) {
                ESP_LOGI(TAG, "Heater Unit Status: I: %.3f A, P: %.1f W, Q: %.1f VAR, S: %.1f VA, PF: %.3f, Samples: %lu",
                         unit_current, unit_power_active, unit_power_reactive, unit_power_apparent, unit_power_factor, unit_samples);
            } else {
                ESP_LOGE(TAG, "Failed to get heater unit status: %d", result);
            }
            
            // Get 3-second economic data for Main Supply
            rbgrid_snapshot_base_t main_supply_economics;
            result = rbgrid_snapshot_get_main_supply(&main_supply_economics);
            
            if (result == RBGRID_OK) {
                const char* direction_str = rbpower_current_direction_to_string(main_supply_economics.direction);
                
                ESP_LOGI(TAG, "Main Supply Economics (3sec): Cost: %.4f, Revenue: %.4f, Rate: %.4f, Direction: %s, Consumed: %.3f Wh, Supplied: %.3f Wh",
                         main_supply_economics.cost_consumed, main_supply_economics.revenue_supplied, main_supply_economics.current_tariff_rate, 
                         direction_str, main_supply_economics.energy_consumed_wh, main_supply_economics.energy_supplied_wh);
            } else {
                ESP_LOGE(TAG, "Failed to get main supply economics: %d", result);
            }
            
            // Get 3-second economic data for Alternative Supply
            rbgrid_snapshot_base_t alt_supply_economics;
            result = rbgrid_snapshot_get_alt_supply(&alt_supply_economics);
            
            if (result == RBGRID_OK) {
                const char* alt_direction_str = rbpower_current_direction_to_string(alt_supply_economics.direction);
                
                ESP_LOGI(TAG, "Alt Supply Economics (3sec): GenCost: %.4f, GenRevenue: %.4f, Rate: %.4f, Direction: %s, Consumed: %.3f Wh, Used: %.3f Wh",
                         alt_supply_economics.cost_consumed, alt_supply_economics.revenue_supplied, 
                         alt_supply_economics.current_tariff_rate, 
                         alt_direction_str, alt_supply_economics.energy_consumed_wh, 
                         alt_supply_economics.energy_supplied_wh);
            } else {
                ESP_LOGE(TAG, "Failed to get alternative supply economics: %d", result);
            }
            
            // Get 3-second economic data for Heater Unit
            rbgrid_snapshot_base_t heater_unit_snapshot_base;
            result = rbgrid_snapshot_get_unit(heater_sensor_uid, &heater_unit_snapshot_base);

            if (result == RBGRID_OK) {
                ESP_LOGI(TAG, "Heater Unit Economics (3sec): CostConsumed: %.6f EUR, RevenueSupplied: %.6f EUR, NetCost: %.6f EUR, AvgTariff: %.4f EUR/kWh, Consumed: %.3f Wh, Supplied: %.3f Wh, NetBalance: %.3f Wh, Samples: %lu",
                         heater_unit_snapshot_base.cost_consumed, heater_unit_snapshot_base.revenue_supplied, (heater_unit_snapshot_base.cost_consumed - heater_unit_snapshot_base.revenue_supplied),
                         heater_unit_snapshot_base.avg_tariff_per_kwh, heater_unit_snapshot_base.energy_consumed_wh, heater_unit_snapshot_base.energy_supplied_wh,
                         (heater_unit_snapshot_base.energy_consumed_wh - heater_unit_snapshot_base.energy_supplied_wh), heater_unit_snapshot_base.samples_count);
            } else {
                ESP_LOGE(TAG, "Failed to get heater unit economics: %d", result);
            }

        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 seconds
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/* ---- Entry points ---- */
/** Standalone application (main/main.c → app_main). */
extern "C" void rbgrid_app_main(void) { (void)rbgrid_boot(false); }

/** Hosted start (ESPHome external component): run after the host has NVS, netif, the event loop and Wi-Fi up. */
extern "C" esp_err_t rbgrid_host_start(void) { return rbgrid_boot(true) ? ESP_OK : ESP_FAIL; }
