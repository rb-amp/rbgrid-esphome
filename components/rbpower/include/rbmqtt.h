/* SPDX-FileCopyrightText: 2025-2026 Zhuhai RobotDyn Technology Co., Ltd.
 * SPDX-License-Identifier: MIT
 */
/**
 * @file    rbmqtt.h
 * @brief   RBgrid MQTT client — real esp-mqtt publisher for the telemetry MQTT channel.
 *
 * Encapsulates the esp-mqtt lifecycle (init / event-handler / connected-flag / publish /
 * LWT). The telemetry layer (rbtelemetry.c send_mqtt) calls rbmqtt_publish; init/stop are
 * tied to Wi-Fi STA up/down. Broker URL is configurable (no hardcoded host). No secrets
 * live here — Wi-Fi credentials are provisioned separately (NVS / portal).
 */
#ifndef RBMQTT_H
#define RBMQTT_H

#include <stdbool.h>
#include "esp_err.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef char* (*rbmqtt_cmd_handler_t)(const char *data, int data_len);
typedef void (*rbmqtt_connect_cb_t)(void);
static inline esp_err_t rbmqtt_init(const char *u, const char *c, const char *l) { (void)u; (void)c; (void)l; return ESP_ERR_NOT_SUPPORTED; }
static inline bool rbmqtt_is_connected(void) { return false; }
static inline const char *rbmqtt_broker_uri(void) { return ""; }
static inline esp_err_t rbmqtt_publish(const char *t, const char *p, int q, bool r) { (void)t; (void)p; (void)q; (void)r; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t rbmqtt_publish_offline(void) { return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t rbmqtt_publish_nb(const char *t, const char *p, int q, bool r) { (void)t; (void)p; (void)q; (void)r; return ESP_ERR_NOT_SUPPORTED; }
static inline esp_err_t rbmqtt_enable_commands(const char *c, const char *r, rbmqtt_cmd_handler_t h) { (void)c; (void)r; (void)h; return ESP_ERR_NOT_SUPPORTED; }
static inline void rbmqtt_register_connect_cb(rbmqtt_connect_cb_t cb) { (void)cb; }
static inline void rbmqtt_stop(void) {}
static inline const char *rbmqtt_tls_state(void) { return "off"; }
static inline bool rbmqtt_tls_custom_ca(void) { return false; }

/** The user's broker CA (PEM, one or more certificates); when present it replaces the public CA bundle for mqtts://. */
#define RBMQTT_CA_FILE "/mqtt_ca.pem"
/** Largest accepted CA file (bytes). */
#define RBMQTT_CA_MAX  8192

#ifdef __cplusplus
}
#endif

#endif // RBMQTT_H
