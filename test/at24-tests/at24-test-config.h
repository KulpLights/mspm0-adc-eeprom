
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
 *  @file       at24-test-config.h
 *  @brief      Tests for at24 emulation
 *
 ******************************************************************************/

#ifndef __AT24_TEST_CONFIG_H__
#define __AT24_TEST_CONFIG_H__

// I2C adapter on host (controller)
//
#define TEST__I2C_ADAPTER 2

// I2C bus address of target device (DUT)
#define TEST__I2C_TAR_ADDR 0x50

// EEPROM size of target device (DUT)
#define TEST__EEPROM_TOTAL_SIZE 1024

// EEPROM page size of target device (DUT)
#define TEST__EEPROM_PAGE_SIZE 32

// EEPROM page count of target device (DUT)
#define TEST__EEPROM_PAGE_CNT (TEST__EEPROM_TOTAL_SIZE /\
    TEST__EEPROM_PAGE_SIZE)

// EEPROM write delay
#define TEST__EEPROM_WRITE_DELAY 10000

// EEPROM write re-try count
#define TEST__EEPROM_RETRY_CNT 10

#endif /* __AT24_TEST_CONFIG_H__ */