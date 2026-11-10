/* SPDX-FileCopyrightText: 2025-2026 Zhuhai RobotDyn Technology Co., Ltd.
 * SPDX-License-Identifier: MIT
 */
/**
 * @file rbevlog.c
 * @brief On-hub event log: PSRAM ring of diag lines, rotated LittleFS files, HTTP read-out.
 *
 * See rbevlog.h for the public API.
 *
 * Ring (PSRAM, this boot): every diag line, incl. the periodic reconciliation trace.
 * Files (/logs/ev0.log newest … ev5.log oldest, 64 KB each, rotated): event + FAIL lines only, one per
 * text line: "<seq> <boot> <ts> <up_s> <lvl> <msg>". seq is monotonic across boots (continued from the
 * newest file at attach), boot counts hub boots seen by the log. ts = wall clock (0 = unknown; lines
 * written before NTP get their ts back-filled from uptime at the first flush with a valid clock).
 *
 * Calling contexts: rbevlog_add() is called from any task that emits a diag line (guarded by s_mtx, 50 ms
 * timeout, then the line is counted as dropped); the writer task flushes to the files every EV_FLUSH_MS or
 * at once on a FAIL line; the HTTP handlers run in the httpd task.
 */
#include "rbstrutil.h"     /* rb_scat — bounded snprintf append */
#include "rbevlog.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <unistd.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_littlefs.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "rbpower.h"
#include "rbhome_types.h"
#include "rbgrid_core.h"
/** @brief 0: remote console verbs are not compiled in (release builds). */
#define RBEVLOG_REMOTE_CMD 0


static const char *TAG = "rbevlog";

#define EV_MSG        200             ///< max line length (bytes, incl. NUL)
#define EV_SLOTS      1536            ///< ring lines (~320 KB PSRAM); halved on allocation failure
#define EV_DIR        "/logs"         ///< mount point of the `logs` partition
#define EV_FILES      6               ///< number of rotated files (ev0 newest .. ev5 oldest)
#define EV_FILE_MAX   (64 * 1024)     ///< rotate when ev0.log reaches this size (bytes)
#define EV_BATCH      (16 * 1024)     ///< file write batch buffer (bytes, PSRAM)
#define EV_FLUSH_MS   30000           ///< periodic flush interval (ms)
#define EV_NTP_WAIT_S 180             ///< hold the first flush this long (s) for NTP (boot lines get a ts)
#define EV_SEQ_GUARD  1000            ///< seq jump at boot: past unflushed lines of a power-cut boot

/** @brief Line level: periodic reconciliation trace (RAM only), event, or FAIL invariant. */
enum { LVL_PERIODIC = 0, LVL_EVENT = 1, LVL_FAIL = 2 };
static const char *const LVL_NAME[] __attribute__((unused)) = { "periodic", "event", "fail" };

/** @brief One ring line. */
typedef struct {
    uint32_t seq, ts, up_s;
    uint8_t  lvl;
    char     msg[EV_MSG];
} ev_slot_t;
/** @var ev_slot_t::seq
 *  Monotonic sequence number (across boots).  @var ev_slot_t::ts
 *  Wall clock (Unix s), 0 = unknown.  @var ev_slot_t::up_s
 *  Uptime (s).  @var ev_slot_t::lvl
 *  LVL_PERIODIC / LVL_EVENT / LVL_FAIL.  @var ev_slot_t::msg
 *  Line text (CR/LF replaced by spaces). */

static ev_slot_t        *s_ring;
static uint32_t          s_nslots;
static SemaphoreHandle_t s_mtx;
static uint32_t          s_base;        /* seq of line k = s_base + k + 1 */
static uint32_t          s_count;       /* lines added this boot */
static uint32_t          s_persist_k;   /* lines [0, s_persist_k) already considered for the files */
static uint32_t          s_boot;
static uint32_t          s_dropped;
static uint32_t          s_fail_boot;
static bool              s_persist_ok;
static bool              s_ts_fixed;
static TaskHandle_t      s_writer;
static char             *s_batch;

static inline uint32_t up_s(void) { return (uint32_t)(esp_timer_get_time() / 1000000LL); }
static inline uint32_t oldest_k(void) { return s_count > s_nslots ? s_count - s_nslots : 0; }

/** Classify a line: "FAIL ..." → LVL_FAIL, the periodic reconciliation trace → LVL_PERIODIC, else LVL_EVENT. */
static uint8_t classify(const char *m)
{
    if (!strncmp(m, "FAIL ", 5)) return LVL_FAIL;
    /* periodic reconciliation: "m<N> +<s>s cnt=" */
    const char *p = m;
    if (*p++ != 'm' || !isdigit((unsigned char)*p)) return LVL_EVENT;
    while (isdigit((unsigned char)*p)) p++;
    if (p[0] != ' ' || p[1] != '+' || !isdigit((unsigned char)p[2])) return LVL_EVENT;
    p += 2;
    while (isdigit((unsigned char)*p)) p++;
    return strncmp(p, "s cnt=", 6) ? LVL_EVENT : LVL_PERIODIC;
}

static void log_sink(const char *line) { rbevlog_add(line); }

static void (*s_listener)(const char *msg, bool fail, uint32_t seq) = NULL;
void rbevlog_set_listener(void (*fn)(const char *msg, bool fail, uint32_t seq)) { s_listener = fn; }

void rbevlog_init(void)
{
    if (s_ring) return;
    s_mtx = xSemaphoreCreateMutex();
    for (uint32_t n = EV_SLOTS; n >= 128 && !s_ring; n /= 2) {
        s_ring = heap_caps_calloc(n, sizeof(ev_slot_t), MALLOC_CAP_SPIRAM);
        if (s_ring) s_nslots = n;
    }
    s_batch = heap_caps_malloc(EV_BATCH, MALLOC_CAP_SPIRAM);
    if (!s_ring || !s_mtx || !s_batch) { ESP_LOGE(TAG, "no memory — event log off"); free(s_ring); s_ring = NULL; return; }
    rbpower_diag_set_log_sink(log_sink);
    ESP_LOGI(TAG, "ring %lu lines (%u KB PSRAM)", (unsigned long)s_nslots,
             (unsigned)(s_nslots * sizeof(ev_slot_t) / 1024));
}

void rbevlog_add(const char *msg)
{
    if (!s_ring || !msg) return;
    uint32_t ts = rbtime_is_time_valid() ? (uint32_t)rbtime_get_unix_timestamp() : 0;
    uint8_t lvl = classify(msg);
    if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(50)) != pdTRUE) { s_dropped++; return; }
    uint32_t k = s_count++;
    ev_slot_t *sl = &s_ring[k % s_nslots];
    const uint32_t seq = s_base + k + 1;
    sl->seq = seq; sl->ts = ts; sl->up_s = up_s(); sl->lvl = lvl;
    strlcpy(sl->msg, msg, sizeof(sl->msg));
    for (char *c = sl->msg; *c; c++) if (*c == '\r' || *c == '\n') *c = ' ';
    if (lvl == LVL_FAIL) s_fail_boot++;
    xSemaphoreGive(s_mtx);
    if (lvl == LVL_FAIL && s_writer) xTaskNotifyGive(s_writer);
    if (s_listener && lvl != LVL_PERIODIC) s_listener(msg, lvl == LVL_FAIL, seq);   /* host copy (ESPHome → HA events) */
}

/* ---- files ---- */
/* Lines of this boot's ring after `since` (seq), as JSON for hosts without the HTTP endpoint (ESPHome → HA). */
char *rbevlog_json(uint32_t since, uint32_t limit, int minlvl)
{
    if (limit < 1) limit = 1;
    if (limit > 100) limit = 100;
    cJSON *o = cJSON_CreateObject(), *a = cJSON_CreateArray();
    if (!o || !a) { cJSON_Delete(o); cJSON_Delete(a); return NULL; }
    cJSON_AddStringToObject(o, "type", "events");
    cJSON_AddNumberToObject(o, "boot", s_boot);
    uint32_t last = since; bool more = false;
    if (s_ring) {
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        uint32_t first_seq = s_base + oldest_k() + 1;
        cJSON_AddNumberToObject(o, "first", first_seq);           /* older lines are only in the files (HTTP) */
        uint32_t k = (since >= s_base) ? since - s_base : 0;
        if (k < oldest_k()) k = oldest_k();
        for (; k < s_count; k++) {
            const ev_slot_t *sl = &s_ring[k % s_nslots];
            if (sl->lvl < minlvl) continue;
            if ((uint32_t)cJSON_GetArraySize(a) >= limit) { more = true; break; }
            cJSON *e = cJSON_CreateObject();
            cJSON_AddNumberToObject(e, "seq", sl->seq);
            cJSON_AddNumberToObject(e, "ts", sl->ts);
            cJSON_AddNumberToObject(e, "up", sl->up_s);
            cJSON_AddStringToObject(e, "lvl", sl->lvl == LVL_FAIL ? "fail" : sl->lvl == LVL_EVENT ? "event" : "periodic");
            cJSON_AddStringToObject(e, "msg", sl->msg);
            cJSON_AddItemToArray(a, e);
            last = sl->seq;
        }
        if (!more) last = s_base + s_count > last ? s_base + s_count : last;   /* cursor past skipped levels */
        xSemaphoreGive(s_mtx);
    }
    cJSON_AddNumberToObject(o, "next", last);
    cJSON_AddBoolToObject(o, "more", more);
    cJSON_AddItemToObject(o, "lines", a);
    char *r = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    return r;
}

static void ev_path(char *p, size_t n, int i) { snprintf(p, n, EV_DIR "/ev%d.log", i); }

/** Rotate the files when ev0.log reached EV_FILE_MAX: drop the oldest, shift ev<i> to ev<i+1>. */
static void rotate_if_full(void)
{
    char p0[24]; ev_path(p0, sizeof(p0), 0);
    struct stat st;
    if (stat(p0, &st) != 0 || st.st_size < EV_FILE_MAX) return;
    char a[24], b[24];
    ev_path(a, sizeof(a), EV_FILES - 1); unlink(a);
    for (int i = EV_FILES - 2; i >= 0; i--) {
        ev_path(a, sizeof(a), i); ev_path(b, sizeof(b), i + 1);
        rename(a, b);
    }
}

/** Append the not-yet-persisted event/FAIL lines to ev0.log (in EV_BATCH chunks). Unless @p force, waits up to
 *  EV_NTP_WAIT_S after boot for a valid clock, then back-fills the wall time of earlier lines. */
static void flush_once(bool force)
{
    if (!s_ring) return;
    bool tv = rbtime_is_time_valid();
    uint32_t now_up = up_s();
    if (!force && !tv && now_up < EV_NTP_WAIT_S) return;
    for (;;) {
        size_t len = 0;
        bool more = false;
        xSemaphoreTake(s_mtx, portMAX_DELAY);
        if (tv && !s_ts_fixed) {   /* back-fill wall time of the lines logged before NTP */
            uint32_t now = (uint32_t)rbtime_get_unix_timestamp();
            for (uint32_t k = oldest_k(); k < s_count; k++) {
                ev_slot_t *sl = &s_ring[k % s_nslots];
                if (!sl->ts && now > now_up - sl->up_s) sl->ts = now - (now_up - sl->up_s);
            }
            s_ts_fixed = true;
        }
        uint32_t k = s_persist_k < oldest_k() ? oldest_k() : s_persist_k;
        for (; k < s_count; k++) {
            ev_slot_t *sl = &s_ring[k % s_nslots];
            if (sl->lvl == LVL_PERIODIC) continue;
            if (len + EV_MSG + 64 > EV_BATCH) { more = true; break; }
            len = rb_scat(s_batch, EV_BATCH, len, "%lu %lu %lu %lu %u %s\n",
                            (unsigned long)sl->seq, (unsigned long)s_boot, (unsigned long)sl->ts,
                            (unsigned long)sl->up_s, (unsigned)sl->lvl, sl->msg);
        }
        s_persist_k = k;
        xSemaphoreGive(s_mtx);
        if (len && s_persist_ok) {
            char p0[24]; ev_path(p0, sizeof(p0), 0);
            FILE *f = fopen(p0, "a");
            if (f) { fwrite(s_batch, 1, len, f); fclose(f); rotate_if_full(); }
            else ESP_LOGW(TAG, "append %s failed", p0);
        }
        if (!more) break;
    }
}

/** Writer task: logs the running image id once, then flushes every EV_FLUSH_MS or when notified (FAIL line). */
static void writer_task(void *arg)
{
    (void)arg;
    {   /* which firmware image booted: md5 of the running image (first 8 hex = image id), once per boot */
        const esp_partition_t *run = esp_ota_get_running_partition();
        const char *md5 = rbgrid_core_running_image_md5();
        char l[96];
        snprintf(l, sizeof(l), "fw: image %.8s (md5 %s) on '%s'", md5[0] ? md5 : "?", md5, run ? run->label : "?");
        rbevlog_add(l);
    }
    for (;;) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(EV_FLUSH_MS));
        flush_once(false);
    }
}

/* esp_restart path: the last lines (e.g. "ota: … rebooting") must reach the file before the reset */
static void shutdown_flush(void) { flush_once(true); }

/* last "<seq> <boot>" in the newest non-empty file */
static bool read_last(uint32_t *seq, uint32_t *boot)
{
    for (int i = 0; i < EV_FILES; i++) {
        char p[24]; ev_path(p, sizeof(p), i);
        FILE *f = fopen(p, "r");
        if (!f) continue;
        char tail[640];
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        long off = sz > (long)sizeof(tail) - 1 ? sz - (long)sizeof(tail) + 1 : 0;
        fseek(f, off, SEEK_SET);
        size_t n = fread(tail, 1, sizeof(tail) - 1, f);
        fclose(f);
        tail[n] = 0;
        while (n && (tail[n - 1] == '\n' || tail[n - 1] == '\r')) tail[--n] = 0;
        if (!n) continue;
        char *line = strrchr(tail, '\n');
        line = line ? line + 1 : tail;
        unsigned long s = 0, b = 0;
        if (sscanf(line, "%lu %lu", &s, &b) == 2) { *seq = s; *boot = b; return true; }
    }
    return false;
}

void rbevlog_attach_storage(void)
{
    if (!s_ring || s_writer) return;
    if (esp_littlefs_mounted("logs")) {
        s_persist_ok = true;
    } else {
        esp_vfs_littlefs_conf_t conf = {
            .base_path = EV_DIR, .partition_label = "logs",
            .format_if_mount_failed = true, .dont_mount = false,
        };
        esp_err_t e = esp_vfs_littlefs_register(&conf);
        s_persist_ok = (e == ESP_OK);
        if (e != ESP_OK) ESP_LOGW(TAG, "mount logs: %s — RAM only", esp_err_to_name(e));
    }
    uint32_t last_seq = 0, last_boot = 0;
    if (s_persist_ok) read_last(&last_seq, &last_boot);
    /* Lines of the previous boot that never reached the file (power cut before a flush) may already have
     * been SEEN by a client (seq cursor) — start this boot past anything that boot could have numbered so a
     * `since=` cursor never skips new lines; the jump is visible as a seq gap, not as reuse. */
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_base = last_seq ? last_seq + EV_SEQ_GUARD : 0;
    s_boot = last_boot + 1;
    for (uint32_t k = oldest_k(); k < s_count; k++) s_ring[k % s_nslots].seq = s_base + k + 1;
    xSemaphoreGive(s_mtx);
    xTaskCreate(writer_task, "rbevlog", 4096, NULL, 2, &s_writer);
    esp_register_shutdown_handler(shutdown_flush);   /* every esp_restart (reboot API, OTA, verbs) */
    char l[96];
    snprintf(l, sizeof(l), "evlog: boot %lu, seq from %lu, files %s", (unsigned long)s_boot,
             (unsigned long)(s_base + 1), s_persist_ok ? "ok" : "OFF (RAM only)");
    rbevlog_add(l);
    /* reserve this boot number + keep the boot lines even if power goes before the first periodic flush
     * (which waits up to EV_NTP_WAIT_S for NTP); otherwise two quick power cycles can reuse a boot number */
    flush_once(true);
}

void rbevlog_register_http(void) {}   /* no HTTP server in this build */
