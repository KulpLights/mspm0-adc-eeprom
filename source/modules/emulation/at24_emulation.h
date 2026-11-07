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
 *  @file       at24_emulation.h
 *  @brief      Emulation library for at24-style I2C EEPROM
 *
 *  Emulates an AT24C256-style EEPROM: two address bytes, 64 byte pages that
 *  wrap within the page on writes, and reads that wrap at the end of memory.
 *  The size is FW_EEPROM_DATA_SIZE: 16KB by default, so a host driving it
 *  as a 32KB 24c256 sees the contents twice; 32KB with FW_LAYOUT_64K.
 *
 *  The contents are stored as-is in flash at FW_EEPROM_DATA_ADDR (see
 *  board/fw_layout.h), so reads come straight from flash.  Writes go to a
 *  RAM copy of one 1KB flash sector, which is written back to flash
 *  ("flushed") when a write lands in a different sector, when the last byte
 *  of the sector is written, or once no write has arrived for 10ms (the
 *  flush timer, restarted by every write).  Reads of the cached sector come
 *  from the RAM copy, so they always see the latest writes.
 *
 *  A flush that only programs flash words that are still erased is done in
 *  place.  Anything else is power-fail safe: the new sector contents are
 *  programmed into a scratch sector, a log record naming the target sector
 *  and the CRC of the scratch contents is written, then the target is
 *  erased and reprogrammed and the record is marked done.  at24_open()
 *  finishes any flush the log shows as interrupted.
 *
 *  While a flush runs the I2C target is disabled, so the bus controller
 *  sees NAKs and must retry (Linux at24: set at24.write_timeout high enough;
 *  an erase on worn flash can take up to 200ms).
 ******************************************************************************/

#ifndef COMMUNICATION_AT24_EMULATION_H_
#define COMMUNICATION_AT24_EMULATION_H_

#include <stdint.h>
#include "communication/i2c_tar_driver.h"

/* Start the emulation; completes any interrupted flush.  Returns 0. */
extern uint32_t at24_open(i2c_tar_driver_t *pI2CTarDriverInst);

extern void at24_i2c_rx_callback(uint32_t bytes, \
    i2c_tar_driver_call_trig_t trig);

extern void at24_i2c_tx_callback(void);

/* Call from the main loop when the flush timer expires.  Flushes the RAM
 * sector copy, or rearms the timer if a bus transaction is in progress. */
extern void at24_idle(void);

/* Call from the main loop on each wake timer tick.  A backstop for a flush
 * that failed: flushes once a full tick has passed with no writes. */
extern void at24_service(void);

#endif /* COMMUNICATION_AT24_EMULATION_H_ */
