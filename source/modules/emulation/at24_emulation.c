/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!****************************************************************************
 *  @file       at24_emulation.c
 *  @brief      Emulation library for at24-style I2C EEPROM
 *
 *  See at24_emulation.h for the storage scheme.
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/dl_flashctl.h>
#include <ti/driverlib/dl_i2c.h>
#include "fw_layout.h"
#include "at24_emulation.h"

#define AT24_SIZE               FW_EEPROM_DATA_SIZE
#define AT24_PAGE_SIZE          64
#define SECTOR_SIZE             FW_FLASH_SECTOR_SIZE
#define SECTOR_WORDS            (SECTOR_SIZE / 4)
#define SECTOR_COUNT            (AT24_SIZE / SECTOR_SIZE)
#define NO_SECTOR               0xFF

/* Flush log: an array of 8 byte records filling the log sector.  Word 0 is
 * MAGIC << 16 | sector << 8 | state, word 1 the CRC-32 of the scratch
 * sector.  The state goes from BEGIN to DONE by programming the same word
 * again, which only clears a bit; the L130x parts have no flash ECC, so that
 * is allowed.  The log is the L1305's top sector, and the CPU can't read the
 * last 8 bytes of flash, so the last two records are never used. */
#define LOG_MAGIC               0xA5C3u
#define LOG_STATE_BEGIN         0xFEu
#define LOG_STATE_DONE          0xFCu
#define LOG_SLOTS               (SECTOR_SIZE / 8 - 2)

#define ERASED_WORD             0xFFFFFFFFu

#if (AT24_SIZE & (AT24_SIZE - 1)) || (SECTOR_COUNT >= NO_SECTOR)
#error Unsupported EEPROM size
#endif

/* RAM copy of one EEPROM sector.  Kept word aligned for flash programming. */
static uint32_t at24_cacheWords[SECTOR_WORDS];
#define at24_cache ((uint8_t *)at24_cacheWords)
static uint8_t at24_cacheSector = NO_SECTOR;
static bool at24_cacheDirty;
static volatile uint32_t at24_writeCount;
static uint32_t at24_serviceWriteCount;

static uint16_t at24_rw_addr;

static i2c_tar_driver_t *at24_i2c_tar_driver_handle;

/*
 * Flash primitives.  Interrupt handlers run from flash, which can't be read
 * while a flash command is running, so interrupts are masked for each
 * command (restoring the caller's mask, so this is safe in an ISR too).
 */

static bool at24_flashErase(uint32_t addr)
{
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    DL_FlashCTL_unprotectSector(FLASHCTL, addr, DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, addr,
        DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    __set_PRIMASK(primask);
    return status != DL_FLASHCTL_COMMAND_STATUS_FAILED;
}

static bool at24_flashProgram64(uint32_t addr, const uint32_t *data)
{
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    DL_FlashCTL_unprotectSector(FLASHCTL, addr, DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_programMemoryFromRAM64(FLASHCTL, addr,
        (uint32_t *)data);
    __set_PRIMASK(primask);
    return status != DL_FLASHCTL_COMMAND_STATUS_FAILED;
}

static const uint32_t *at24_flashWords(uint32_t addr)
{
    return (const uint32_t *)addr;
}

static bool at24_sectorIsBlank(uint32_t addr)
{
    const uint32_t *f = at24_flashWords(addr);
    uint32_t i;

    for (i = 0; i < SECTOR_WORDS; i++)
    {
        if (f[i] != ERASED_WORD)
        {
            return false;
        }
    }
    return true;
}

/* Program a freshly erased sector from RAM, skipping words that stay
 * erased.  data must be word aligned. */
static bool at24_programSector(uint32_t addr, const uint32_t *data)
{
    uint32_t i;

    for (i = 0; i < SECTOR_WORDS; i += 2)
    {
        if (data[i] == ERASED_WORD && data[i + 1] == ERASED_WORD)
        {
            continue;
        }
        if (!at24_flashProgram64(addr + i * 4, &data[i]))
        {
            return false;
        }
    }
    return memcmp((const void *)addr, data, SECTOR_SIZE) == 0;
}

static uint32_t at24_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    uint32_t i;
    uint8_t bit;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static uint32_t at24_sectorAddr(uint8_t sector)
{
    return FW_EEPROM_DATA_ADDR + (uint32_t)sector * SECTOR_SIZE;
}

/*
 * Flush log
 */

static int32_t at24_logFreeSlot(void)
{
    const uint32_t *log = at24_flashWords(FW_EEPROM_LOG_ADDR);
    int32_t slot;

    for (slot = 0; slot < LOG_SLOTS; slot++)
    {
        if (log[slot * 2] == ERASED_WORD && log[slot * 2 + 1] == ERASED_WORD)
        {
            return slot;
        }
    }
    return -1;
}

static bool at24_logWrite(int32_t slot, uint8_t sector, uint8_t state,
    uint32_t crc)
{
    uint32_t rec[2];

    rec[0] = ((uint32_t)LOG_MAGIC << 16) | ((uint32_t)sector << 8) | state;
    rec[1] = crc;
    return at24_flashProgram64(FW_EEPROM_LOG_ADDR + slot * 8, rec);
}

/* Finish a flush that was interrupted by a reset or power loss. */
static void at24_recover(void)
{
    const uint32_t *log = at24_flashWords(FW_EEPROM_LOG_ADDR);
    uint32_t w0;
    uint32_t w1;
    uint32_t i;
    uint32_t words[2];
    uint8_t sector;
    int32_t slot;

    for (slot = 0; slot < LOG_SLOTS; slot++)
    {
        w0 = log[slot * 2];
        w1 = log[slot * 2 + 1];
        if (w0 == ERASED_WORD && w1 == ERASED_WORD)
        {
            break;
        }
        if ((w0 >> 16) != LOG_MAGIC || (w0 & 0xFF) != LOG_STATE_BEGIN)
        {
            continue;
        }
        sector = (w0 >> 8) & 0xFF;
        if (sector < SECTOR_COUNT && at24_crc32( \
            (const uint8_t *)FW_EEPROM_SCRATCH_ADDR, SECTOR_SIZE) == w1)
        {
            /* The scratch copy is complete: redo the erase and copy. */
            uint32_t addr = at24_sectorAddr(sector);
            const uint32_t *scratch = at24_flashWords(FW_EEPROM_SCRATCH_ADDR);

            at24_flashErase(addr);
            for (i = 0; i < SECTOR_WORDS; i += 2)
            {
                words[0] = scratch[i];
                words[1] = scratch[i + 1];
                if (words[0] != ERASED_WORD || words[1] != ERASED_WORD)
                {
                    at24_flashProgram64(addr + i * 4, words);
                }
            }
        }
        /* A record whose CRC doesn't match was torn while being written,
         * before the target was touched; just retire it. */
        at24_logWrite(slot, sector, LOG_STATE_DONE, w1);
    }
}

/*
 * Sector cache
 */

/* Write the cached sector back to flash if it has changed. */
static void at24_flush(void)
{
    uint32_t addr;
    const uint32_t *f;
    uint32_t i;
    bool inPlace;
    int32_t slot;
    uint32_t crc;

    if (!at24_cacheDirty)
    {
        return;
    }
    at24_cacheDirty = false;

    addr = at24_sectorAddr(at24_cacheSector);
    f = at24_flashWords(addr);
    if (memcmp(f, at24_cacheWords, SECTOR_SIZE) == 0)
    {
        return;
    }

    /* If every changed flash word is still erased, program those words in
     * place: no erase, nothing to lose on power failure but the bytes being
     * written. */
    inPlace = true;
    for (i = 0; i < SECTOR_WORDS && inPlace; i += 2)
    {
        if ((f[i] != at24_cacheWords[i] || f[i + 1] != at24_cacheWords[i + 1])
            && (f[i] != ERASED_WORD || f[i + 1] != ERASED_WORD))
        {
            inPlace = false;
        }
    }
    if (inPlace)
    {
        for (i = 0; i < SECTOR_WORDS; i += 2)
        {
            if (f[i] != at24_cacheWords[i] || f[i + 1] != at24_cacheWords[i + 1])
            {
                at24_flashProgram64(addr + i * 4, &at24_cacheWords[i]);
            }
        }
        if (memcmp(f, at24_cacheWords, SECTOR_SIZE) == 0)
        {
            return;
        }
        /* Didn't verify; fall through to a full rewrite */
    }

    /* Full rewrite through the scratch sector */
    slot = at24_logFreeSlot();
    if (slot < 0)
    {
        /* Every record is done (at24_recover and each flush finish theirs),
         * so the log can be cleared. */
        at24_flashErase(FW_EEPROM_LOG_ADDR);
        slot = 0;
    }
    if (!at24_sectorIsBlank(FW_EEPROM_SCRATCH_ADDR))
    {
        at24_flashErase(FW_EEPROM_SCRATCH_ADDR);
    }
    if (!at24_programSector(FW_EEPROM_SCRATCH_ADDR, at24_cacheWords))
    {
        /* Scratch didn't verify; leave the target untouched */
        at24_cacheDirty = true;
        return;
    }
    crc = at24_crc32(at24_cache, SECTOR_SIZE);
    at24_logWrite(slot, at24_cacheSector, LOG_STATE_BEGIN, crc);

    at24_flashErase(addr);
    at24_programSector(addr, at24_cacheWords);

    at24_logWrite(slot, at24_cacheSector, LOG_STATE_DONE, crc);
}

static void at24_selectSector(uint8_t sector)
{
    if (sector == at24_cacheSector)
    {
        return;
    }
    at24_flush();
    memcpy(at24_cacheWords, (const void *)at24_sectorAddr(sector), SECTOR_SIZE);
    at24_cacheSector = sector;
}

static uint8_t at24_readByte(uint16_t addr)
{
    if ((addr / SECTOR_SIZE) == at24_cacheSector)
    {
        return at24_cache[addr % SECTOR_SIZE];
    }
    return *(const uint8_t *)(FW_EEPROM_DATA_ADDR + addr);
}

static void at24_writeByte(uint16_t addr, uint8_t data)
{
    at24_selectSector(addr / SECTOR_SIZE);
    if (at24_cache[addr % SECTOR_SIZE] != data)
    {
        at24_cache[addr % SECTOR_SIZE] = data;
        at24_cacheDirty = true;
    }
}

/*
 * Public API
 */

uint32_t at24_open(i2c_tar_driver_t *pI2CTarDriverInst)
{
    at24_i2c_tar_driver_handle = pI2CTarDriverInst;
    at24_rw_addr = 0x0000;
    at24_cacheSector = NO_SECTOR;
    at24_cacheDirty = false;

    at24_recover();

    return 0;
}

void at24_service(void)
{
    I2C_Regs *module;
    uint32_t primask;

    if (!at24_cacheDirty)
    {
        return;
    }
    if (at24_writeCount != at24_serviceWriteCount)
    {
        /* Written since the last tick; wait for a quiet one */
        at24_serviceWriteCount = at24_writeCount;
        return;
    }

    /* Take the target offline (so we NAK) only between bus transactions, and
     * never in the middle of one of ours. */
    module = at24_i2c_tar_driver_handle->pModule;
    primask = __get_PRIMASK();
    __disable_irq();
    if ((at24_i2c_tar_driver_handle->state != I2C_TAR_DRV_STATE__IDLE) ||
        (DL_I2C_getTargetStatus(module) & DL_I2C_TARGET_STATUS_BUS_BUSY))
    {
        __set_PRIMASK(primask);
        return;
    }
    DL_I2C_disableTarget(module);
    __set_PRIMASK(primask);

    at24_flush();

    DL_I2C_enableTarget(module);
}

void at24_i2c_rx_callback(uint32_t bytes, i2c_tar_driver_call_trig_t trig)
{
    uint16_t addr;
    uint16_t pageBase;
    uint16_t pageOffset;

    /* Runs from the I2C ISR with the target disabled, so the bus controller
     * gets NAKs while any flash work here is in progress. */
    if (bytes < 2)
    {
        return;
    }

    addr = (uint16_t)(I2CTarDriver_read(at24_i2c_tar_driver_handle)) << 8;
    addr |= (uint16_t)(I2CTarDriver_read(at24_i2c_tar_driver_handle));
    addr %= AT24_SIZE;
    bytes -= 2;
    at24_rw_addr = addr;

    /* An address-only write, or data followed by a RESTART (a read is
     * coming, not a write cycle), only moves the address pointer. */
    if ((bytes == 0) || (trig == I2C_TAR_DRV_CALL_TRIG__RESTART))
    {
        return;
    }

    /* Page write: the address wraps within the 64 byte page */
    pageBase = addr & ~(uint16_t)(AT24_PAGE_SIZE - 1);
    pageOffset = addr & (AT24_PAGE_SIZE - 1);
    while (bytes--)
    {
        addr = pageBase + pageOffset;
        at24_writeByte(addr, I2CTarDriver_read(at24_i2c_tar_driver_handle));
        pageOffset = (pageOffset + 1) & (AT24_PAGE_SIZE - 1);
    }
    at24_rw_addr = pageBase + pageOffset;
    at24_writeCount++;

    /* A sequential writer has finished this sector: write it back now
     * rather than waiting for the next sector or the idle timer. */
    if (((addr + 1) % SECTOR_SIZE) == 0)
    {
        at24_flush();
    }
}

void at24_i2c_tx_callback(void)
{
    uint8_t i;

    /* Called each time the transmit FIFO runs dry during a read; queue the
     * next few bytes.  Reads wrap at the end of memory. */
    for (i = 0; i < 4; i++)
    {
        I2CTarDriver_write(at24_i2c_tar_driver_handle, \
            at24_readByte(at24_rw_addr));
        at24_rw_addr = (at24_rw_addr + 1) % AT24_SIZE;
    }
}
