/* SPDX-FileCopyrightText: 2026 Zhuhai RobotDyn Technology Co., Ltd.
 * SPDX-License-Identifier: MIT */
/**
 * @file rbfault.h
 * @brief Fault injection for crash-consistency tests. TEST BUILDS ONLY (RBGRID_FAULT_INJECT): in a release
 *        build every rbfault_hit() is a constant false and the debug endpoint does not exist.
 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Places a fault can be injected. */
typedef enum {
    RBFAULT_FLASH_RECORD_WRITE = 0,   ///< rbpower_flash_write_record → ESP_FAIL (history record set)
    RBFAULT_LFS_SYNC,                 ///< littlefs file sync → error (config/token writes)
    RBFAULT_LFS_MOUNT_IO,             ///< littlefs mount → LFS_ERR_IO (armed in NVS, consumed at the next boot)
    RBFAULT_ESPNOW_ACK_DROP,          ///< ESP-NOW hub: the PERIOD_ACK after an inject is not sent (a lost ACK)
    RBFAULT_GAP_PLUS_DAY,             ///< rbAmp reboot gap: +25 h added to the measured gap (armed in NVS for the next boot)
    RBFAULT_COUNT
} rbfault_point_t;

static inline bool rbfault_hit(rbfault_point_t p) { (void)p; return false; }
static inline void rbfault_load_boot(void) {}

#ifdef __cplusplus
}
#endif
