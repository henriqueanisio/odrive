#include "flash_storage.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stddef.h>   /* offsetof */

/* ── CRC-32 (IEEE 802.3, reflected / LSB-first) ──────────────────────────────
 * Software implementation — avoids contention with the STM32 CRC peripheral
 * which may be in use elsewhere (e.g. USB).
 * ─────────────────────────────────────────────────────────────────────────── */
static uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; i++) {
            /* Branchless: mask is 0xEDB88320 when LSB==1, else 0 */
            crc = (crc >> 1) ^ (0xEDB88320U & (uint32_t)(-(int32_t)(crc & 1U)));
        }
    }
    return ~crc;
}

/* Compute CRC over all fields that precede the crc member. */
static uint32_t flash_crc(const FlashConfig_t *s)
{
    return crc32((const uint8_t *)s, offsetof(FlashConfig_t, crc));
}

/* ── flash_load_config ───────────────────────────────────────────────────────
 * Read-only: directly dereferences flash via memory-mapped address.
 * ─────────────────────────────────────────────────────────────────────────── */
bool flash_load_config(void)
{
    const FlashConfig_t *stored = (const FlashConfig_t *)FLASH_CFG_ADDRESS;

    if (stored->magic   != FLASH_CFG_MAGIC)   return false;
    if (stored->version != FLASH_CFG_VERSION) return false;
    if (stored->crc     != flash_crc(stored)) return false;

    g_config = stored->config;
    return true;
}

/* ── flash_save_config ───────────────────────────────────────────────────────
 * 1. Build FlashConfig_t in RAM.
 * 2. Erase sector.
 * 3. Write word-by-word (HAL requires 32-bit aligned writes in RANGE_3).
 * 4. Readback verification.
 * ─────────────────────────────────────────────────────────────────────────── */
bool flash_save_config(void)
{
    /* Build image in RAM */
    FlashConfig_t img;
    img.magic   = FLASH_CFG_MAGIC;
    img.version = FLASH_CFG_VERSION;
    img.config  = g_config;
    img.crc     = flash_crc(&img);

    HAL_StatusTypeDef status;

    /* ── Unlock ── */
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) return false;

    /* ── Erase sector ── */
    FLASH_EraseInitTypeDef erase_cfg = {
        .TypeErase    = FLASH_TYPEERASE_SECTORS,
        .VoltageRange = FLASH_CFG_VOLTAGE_RANGE,
        .Sector       = FLASH_CFG_SECTOR,
        .NbSectors    = 1U,
    };
    uint32_t sector_error = 0xFFFFFFFFU;
    status = HAL_FLASHEx_Erase(&erase_cfg, &sector_error);
    if (status != HAL_OK || sector_error != 0xFFFFFFFFU) {
        HAL_FLASH_Lock();
        return false;
    }

    /* ── Write word by word ── */
    const uint32_t *src    = (const uint32_t *)&img;
    uint32_t        dest   = FLASH_CFG_ADDRESS;
    size_t          words  = (sizeof(FlashConfig_t) + 3U) / 4U;

    for (size_t i = 0; i < words; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, dest, (uint64_t)src[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        dest += 4U;
    }

    HAL_FLASH_Lock();

    /* ── Readback verify ── */
    return (memcmp((const void *)FLASH_CFG_ADDRESS, &img, sizeof(FlashConfig_t)) == 0);
}
