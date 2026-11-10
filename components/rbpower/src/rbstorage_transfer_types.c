/* SPDX-FileCopyrightText: 2025-2026 Zhuhai RobotDyn Technology Co., Ltd.
 * SPDX-License-Identifier: MIT
 */
/**
 * @file rbstorage_transfer_types.c
 * @brief Transfer-structure size, ring-capacity and sector tables, plus helper lookups
 */

#include "rbstorage_transfer_types.h"
#include "rbstats_v4_types.h"        /* v4 record structs + retention tables */
#include "rbextflash_structures.h"   /* RBPOWER_RECORD_FORMAT_VERSION */
#include "esp_log.h"
#include <string.h>

static const char* TAG = "RBSTORAGE_TRANSFER";

/* ================================================================================================
 * CONSTANTS AND TABLES
 * ================================================================================================ */

/**
 * @brief Transfer-structure (on-flash record) size for each data type, bytes
 */
const uint16_t RBPOWER_TRANSFER_SIZES[RBPOWER_DATA_TYPE_MAX] = {
    [RBPOWER_DATA_TYPE_ELECTRICAL] = sizeof(rbpower_electrical_transfer_t),  // 32 bytes (with min/max envelope)
    [RBPOWER_DATA_TYPE_ENERGY]     = sizeof(rbpower_energy_transfer_t),      // 16 bytes
    [RBPOWER_DATA_TYPE_ECONOMICS]  = sizeof(rbpower_economics_transfer_t),   // 16 bytes
    [RBPOWER_DATA_TYPE_QUALITY]    = sizeof(rbpower_quality_transfer_t),     // 8 bytes
    [RBPOWER_DATA_TYPE_DIRECTION]  = sizeof(rbpower_direction_transfer_t)    // 32 bytes
};

/**
 * @brief Logical ring-buffer capacity per period, records (public, for the API)
 * This is the number of records guaranteed to be available through the API.
 */
const uint32_t RBPOWER_RECORDS_COUNT[RBPOWER_PERIOD_MAX] = {
    [RBPOWER_PERIOD_HOUR]  = 1440, // 60 days × 24 hours — retention grown using the sectors freed by
                                   //   the quality/direction removal. HOUR's
                                   //   RECORDS_COUNT is purely ring capacity (DAY aggregates a literal 24
                                   //   HOUR records), so growing it does NOT affect aggregation counts.
    [RBPOWER_PERIOD_DAY]   = 30,   // 30 days — DO NOT GROW via RECORDS_COUNT: it doubles as the child
                                   //   count for MONTH aggregation (finalize reads RECORDS_COUNT[DAY] days).
    [RBPOWER_PERIOD_WEEK]  = 8,    // ~2 months (8 weeks)
    [RBPOWER_PERIOD_MONTH] = 24,   // 2 years × 12 months — also the YEAR-aggregation child count.
    [RBPOWER_PERIOD_YEAR]  = 5     // 5 years
};

/**
 * @brief Number of 4 KB flash sectors for each period + data type combination
 * One sector more than the minimum is allocated so that the wrap-around erase of
 * the oldest sector never drops below the logical capacity.
 *
 * Layout: [period][data_type] = number_of_sectors
 */
const uint8_t RBPOWER_BUFFER_SECTORS[RBPOWER_PERIOD_MAX][RBPOWER_DATA_TYPE_MAX] = {
    // HOUR period (1440 records = 60 days; retention grown via the quality/direction-freed sectors)
    [RBPOWER_PERIOD_HOUR] = {
        [RBPOWER_DATA_TYPE_ELECTRICAL] = 10, // 28 bytes: 146 rec/sector × 10 = 1460 ≥ 1440 (60 days)
        [RBPOWER_DATA_TYPE_ENERGY]     = 6,  // 16 bytes: 256 rec/sector × 6 = 1536 ≥ 1440
        [RBPOWER_DATA_TYPE_ECONOMICS]  = 6,  // 16 bytes: 256 rec/sector × 6 = 1536 ≥ 1440
        [RBPOWER_DATA_TYPE_QUALITY]    = 0,  // REMOVED from active pipeline (rbAmp has no ZC/THD source)
        [RBPOWER_DATA_TYPE_DIRECTION]  = 0   // REMOVED from active pipeline — sectors freed
    },
    // DAY period (30 records)
    [RBPOWER_PERIOD_DAY] = {
        [RBPOWER_DATA_TYPE_ELECTRICAL] = 2,  // 28 bytes: min 1, allocate 2 (292 records)
        [RBPOWER_DATA_TYPE_ENERGY]     = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_ECONOMICS]  = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_QUALITY]    = 0,  // REMOVED from active pipeline
        [RBPOWER_DATA_TYPE_DIRECTION]  = 0   // REMOVED from active pipeline — sectors freed
    },
    // WEEK period (8 records)
    [RBPOWER_PERIOD_WEEK] = {
        [RBPOWER_DATA_TYPE_ELECTRICAL] = 2,  // 28 bytes: min 1, allocate 2 (292 records)
        [RBPOWER_DATA_TYPE_ENERGY]     = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_ECONOMICS]  = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_QUALITY]    = 0,  // Not used for WEEK
        [RBPOWER_DATA_TYPE_DIRECTION]  = 0   // Not used for WEEK
    },
    // MONTH period (24 records)
    [RBPOWER_PERIOD_MONTH] = {
        [RBPOWER_DATA_TYPE_ELECTRICAL] = 2,  // 28 bytes: min 1, allocate 2 (292 records)
        [RBPOWER_DATA_TYPE_ENERGY]     = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_ECONOMICS]  = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_QUALITY]    = 0,  // Not used for MONTH
        [RBPOWER_DATA_TYPE_DIRECTION]  = 0   // Not used for MONTH
    },
    // YEAR period (5 records)
    [RBPOWER_PERIOD_YEAR] = {
        [RBPOWER_DATA_TYPE_ELECTRICAL] = 2,  // 28 bytes: min 1, allocate 2 (292 records)
        [RBPOWER_DATA_TYPE_ENERGY]     = 2,  // 16 bytes: min 1, allocate 2 (512 records)
        [RBPOWER_DATA_TYPE_ECONOMICS]  = 0,  // Not used for YEAR
        [RBPOWER_DATA_TYPE_QUALITY]    = 0,  // Not used for YEAR
        [RBPOWER_DATA_TYPE_DIRECTION]  = 0   // Not used for YEAR
    }
};

/**
 * @brief Data type names
 */
static const char* DATA_TYPE_NAMES[RBPOWER_DATA_TYPE_MAX] = {
    [RBPOWER_DATA_TYPE_ELECTRICAL] = "electrical",
    [RBPOWER_DATA_TYPE_ENERGY]     = "energy",
    [RBPOWER_DATA_TYPE_ECONOMICS]  = "economics",
    [RBPOWER_DATA_TYPE_QUALITY]    = "quality",
    [RBPOWER_DATA_TYPE_DIRECTION]  = "direction"
};

/**
 * @brief Period names
 */
static const char* PERIOD_NAMES[RBPOWER_PERIOD_MAX] = {
    [RBPOWER_PERIOD_HOUR]  = "hour",
    [RBPOWER_PERIOD_DAY]   = "day",
    [RBPOWER_PERIOD_WEEK]  = "week",
    [RBPOWER_PERIOD_MONTH] = "month",
    [RBPOWER_PERIOD_YEAR]  = "year"
};

/* ================================================================================================
 * FUNCTIONS
 * ================================================================================================ */

uint16_t rbpower_get_transfer_size(rbpower_data_type_t data_type)
{
    if (data_type >= RBPOWER_DATA_TYPE_MAX) {
        ESP_LOGE(TAG, "Invalid data type: %d", data_type);
        return 0;
    }
    
    return RBPOWER_TRANSFER_SIZES[data_type];
}

uint32_t rbpower_get_records_count(rbpower_period_type_t period_type)
{
    if (period_type >= RBPOWER_PERIOD_MAX) {
        ESP_LOGE(TAG, "Invalid period type: %d", period_type);
        return 0;
    }
    
    return RBPOWER_RECORDS_COUNT[period_type];
}

bool rbpower_is_version_compatible(const rbpower_version_t* version)
{
    if (version == NULL) {
        ESP_LOGE(TAG, "Version pointer is NULL");
        return false;
    }
    
    // The major version must match
    if (version->major != RBPOWER_STORAGE_VERSION_MAJOR) {
        ESP_LOGW(TAG, "Major version mismatch: expected %d, got %d", 
                 RBPOWER_STORAGE_VERSION_MAJOR, version->major);
        return false;
    }
    
    // The minor version must be >= the minimum supported one
    if (version->minor < RBPOWER_MIN_COMPATIBLE_MINOR) {
        ESP_LOGW(TAG, "Minor version too old: minimum %d, got %d", 
                 RBPOWER_MIN_COMPATIBLE_MINOR, version->minor);
        return false;
    }
    
    return true;
}

void rbpower_get_current_version(rbpower_version_t* version)
{
    if (version == NULL) {
        ESP_LOGE(TAG, "Version pointer is NULL");
        return;
    }
    
    version->major = RBPOWER_STORAGE_VERSION_MAJOR;
    version->minor = RBPOWER_STORAGE_VERSION_MINOR;
    version->patch = RBPOWER_STORAGE_VERSION_PATCH;
    version->reserved = 0;
}

const char* rbpower_get_data_type_name(rbpower_data_type_t data_type)
{
    if (data_type >= RBPOWER_DATA_TYPE_MAX) {
        ESP_LOGW(TAG, "Invalid data type: %d", data_type);
        return "unknown";
    }
    
    return DATA_TYPE_NAMES[data_type];
}

const char* rbpower_get_period_name(rbpower_period_type_t period_type)
{
    if (period_type >= RBPOWER_PERIOD_MAX) {
        ESP_LOGW(TAG, "Invalid period type: %d", period_type);
        return "unknown";
    }

    return PERIOD_NAMES[period_type];
}

uint8_t rbpower_get_buffer_sectors(rbpower_period_t period, rbpower_data_type_t data_type)
{
    if (period >= RBPOWER_PERIOD_MAX) {
        ESP_LOGE(TAG, "Invalid period: %d", period);
        return 0;
    }

    if (data_type >= RBPOWER_DATA_TYPE_MAX) {
        ESP_LOGE(TAG, "Invalid data type: %d", data_type);
        return 0;
    }

    return RBPOWER_BUFFER_SECTORS[period][data_type];
}

uint32_t rbpower_get_physical_capacity(rbpower_period_t period, rbpower_data_type_t data_type)
{
    uint8_t sectors = rbpower_get_buffer_sectors(period, data_type);
    if (sectors == 0) {
        return 0;
    }

    uint16_t record_size = rbpower_get_transfer_size(data_type);
    if (record_size == 0) {
        return 0;
    }

    // Records per sector
    uint32_t records_per_sector = 4096 / record_size;

    // Physical capacity = sectors × records_per_sector
    return sectors * records_per_sector;
}

uint32_t rbpower_get_buffer_size_bytes(rbpower_period_t period, rbpower_data_type_t data_type)
{
    uint8_t sectors = rbpower_get_buffer_sectors(period, data_type);
    if (sectors == 0) {
        return 0;
    }

    return sectors * 4096;
}