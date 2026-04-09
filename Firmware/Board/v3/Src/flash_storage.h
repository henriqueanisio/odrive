#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* ── Flash sector selection (STM32F405RG) ────────────────────────────────────
 *   Sector 11 — last 128 KB at 0x080E0000.
 *   Safe choice: sits above all code, large enough for many future versions.
 * ─────────────────────────────────────────────────────────────────────────── */
#define FLASH_CFG_SECTOR        FLASH_SECTOR_11
#define FLASH_CFG_ADDRESS       0x080E0000UL
#define FLASH_CFG_VOLTAGE_RANGE FLASH_VOLTAGE_RANGE_3  /* 2.7–3.6 V → 32-bit */

/* ── Persistent store layout ──────────────────────────────────────────────── */
#define FLASH_CFG_MAGIC         0x4F445256UL   /* "ODRV" in ASCII             */
#define FLASH_CFG_VERSION       1UL

typedef struct __attribute__((packed)) {
    uint32_t       magic;    /* Must equal FLASH_CFG_MAGIC                    */
    uint32_t       version;  /* Schema version — bump when ODriveConfig_t changes */
    ODriveConfig_t config;   /* Payload                                       */
    uint32_t       crc;      /* CRC-32 of {magic, version, config}            */
} FlashConfig_t;

/* ── API ───────────────────────────────────────────────────────────────────── */

/* Validate and load stored config into g_config.
 * Returns true on success; g_config is untouched on failure (keeps defaults).*/
bool flash_load_config(void);

/* Persist current g_config to flash.
 * IMPORTANT: disables interrupts briefly during erase/write (~ms range).
 * Must be called from a non-ISR context (e.g. RTOS task or main loop).
 * Returns true only if write succeeded AND readback verified correctly.      */
bool flash_save_config(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORAGE_H */
