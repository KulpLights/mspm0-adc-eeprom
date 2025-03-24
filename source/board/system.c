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
 *  @file       system.c
 *  @brief      System management and ISR's for Processor Companion Utility
 *
 *  This file contains the interrupt handlers and system
 *  management routines used by the application.
 *
 ******************************************************************************/

#include "ti_msp_dl_config.h"
#include "communication/i2c_tar_driver.h"
#include "emulation/at24_emulation.h"
#include "emulation/ad7291_emulation.h"
#include "system.h"

/*!
 * @brief   Length of the I2C receive buffer in bytes
 *
 * This is the maximum number of bytes which can be received by the I2C
 * target driver.  For the EEPROM target function, this buffer needs to be
 * at least the size of one page (to support page writes), plus an
 * additional two bytes for the address.
 */
#define TAR_I2C_RXBUF_LEN 128

/*!
 * @brief   Length of the I2C transmit buffer in bytes
 *
 * This is the # of bytes which may be transmitted at once before another
 * callback is issued from the I2C target driver to this library.
 */
#define TAR_I2C_TXBUF_LEN 128

/**
 * @brief   I2C target driver RX buffer
 */
uint8_t tar_i2c_rxbuf[TAR_I2C_RXBUF_LEN];

/**
 * @brief   I2C target driver TX buffer
 */
uint8_t tar_i2c_txbuf[TAR_I2C_TXBUF_LEN];


/**
 * The I2C target driver data structure instance for the I2C I/F we
 * are using for communication with external I2C bus controllers
 */
i2c_tar_driver_t tar_i2c =
{
    .pModule = TAR_I2C_INST,
    .pRxBuf = &tar_i2c_rxbuf[0],
    .rxBufLen = TAR_I2C_RXBUF_LEN,
    .pTxBuf = &tar_i2c_txbuf[0],
    .txBufLen = TAR_I2C_TXBUF_LEN,
    .rxCallback = &ad7291_i2c_rx_callback,
    .txCallback = &ad7291_i2c_tx_callback,
    .rxCallback2 = &at24_i2c_rx_callback,
    .txCallback2 = &at24_i2c_tx_callback
};

/*!
 * @brief Indicates if the I2C EEPROM emulation module started properly
 */
bool System_eepromOnline;

/*!
 * @brief Indicates pending system events
 */
volatile uint32_t System_pendingEvents;

static const DL_I2C_ClockConfig gI2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

extern void setupAppSpecific();

void System_init(void)
{
    System_pendingEvents = SYSEVENT__NONE_PENDING;

    SYSCFG_DL_init();
    delay_cycles(1000000);
    setupAppSpecific();

    /* Move I2C pins to GPI, check for bus-high condition before
     * continuing to start the I2C target driver. */
    DL_GPIO_initDigitalInputFeatures(GPIO_TAR_I2C_IOMUX_SDA, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN, \
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(GPIO_TAR_I2C_IOMUX_SCL, \
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN, \
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    //while (DL_GPIO_readPins(GPIOA, 0x03) != 0x03);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_TAR_I2C_IOMUX_SDA,
        GPIO_TAR_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_TAR_I2C_IOMUX_SCL,
        GPIO_TAR_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_TAR_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_TAR_I2C_IOMUX_SCL);


    /* Start the EEPROM emulation module to ensure the EEPROM emulation
     * is online before starting the I2C target driver */
    if(at24_open(&tar_i2c) == 0)
    {
        System_eepromOnline = true;
    }
    else
    {
        System_eepromOnline = false;
    }

    /* If EEPROM module did not load correctly, de-associate
     * EEPROM emulation library handlers from I2C target driver
     * so that the I2C target driver does not call handlers
     * into an invalid EEPROM configuration. */
    if (System_eepromOnline == false)
    {
        tar_i2c.rxCallback2 = I2C_TAR_DRIVER_NO_CALLBACK;
        tar_i2c.txCallback2 = I2C_TAR_DRIVER_NO_CALLBACK;
    }

    /* Start the ADC emulation module to ensure it is online before starting
     * the I2C target driver
     */
    ad7291_open(&tar_i2c);

    /* Temporarily leave peripheral SPI disabled until we add that functionality */
    DL_SPI_disable(PER_SPI_INST);
    /* Start the I2C target driver */
    I2CTarDriver_open(&tar_i2c);

    /* Enable interrupts used by the application */
    NVIC_EnableIRQ(WAKEUP_TIMER_INST_INT_IRQN);
    NVIC_EnableIRQ(TAR_I2C_INST_INT_IRQN);
//    NVIC_EnableIRQ(PER_SPI_INST_INT_IRQN);
}

void System_sleepUntilInterrupt(void)
{
    /* Disable interrupts to allow safe check of pending events.
     * If no events are pending, enter low power mode.  If an event
     * was pending, do not enter low power mode.  In all cases, exit with
     * interrupts left enabled. */
    __disable_irq();
    if (System_pendingEvents == SYSEVENT__NONE_PENDING)
    {
        __WFI();
    }
    __enable_irq();
}

void WAKEUP_TIMER_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(WAKEUP_TIMER_INST, \
        DL_TIMER_INTERRUPT_ZERO_EVENT);
    System_pendingEvents |= SYSEVENT__WAKE_TIMER;
}

void TAR_I2C_INST_IRQHandler(void)
{
    I2CTarDriver_ISR(&tar_i2c);
}
