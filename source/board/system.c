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
#define SSD1306_CNTRL_CMD                                          0x00
#define SSD1306_CNTRL_DATA                                         0x40

#define LED_DISPLAY_WIDTH 128
#define LED_DISPLAY_HEIGHT 64

#define SSD1306_TARGET 0x3c

/* SSD1306 Commands */
#define SSD1306_DISPLAY_OFF                                        0xAE
#define SSD1306_SET_DISP_CLK                                       0xD5
#define SSD1306_SET_MULTIPLEX                                      0xA8
#define SSD1306_SET_DISP_OFFSET                                    0xD3
#define SSD1306_SET_DISP_START_LINE                                0x40// | 0x00)
#define SSD1306_CONFIG_CHARGE_PUMP                                 0x8D
#define SSD1306_SET_MEM_ADDR_MODE                                  0x20
#define SSD1306_SEG_REMAP                                          (0xA0 | 0x01)               //Rotate 180 Degrees
#define SSD1306_SET_COMSCANDEC                                     0xC8
#define SSD1306_SET_COMPINS                                        0xDA
#define SSD1306_SET_CONTRAST                                       0x81
#define SSD1306_SET_PRECHARGE                                      0xD9
#define SSD1306_SET_VCOMDETECT                                     0xDB
#define SSD1306_DISPLAYALLON_RESUME                                0xA4
#define SSD1306_NORMAL_DISPLAY                                     0xA6
#define SSD1306_DISPLAYON                                          0xAF
#define SSD1306_SET_COL_ADDR                                       0x21
#define SSD1306_PAGEADDR                                           0x22
#define SSD1306_INVERT_DISPLAY                                     0x01
#define SSD1306_NORMALIZE_DISPLAY                                  0x00
/* SDD1306 Scroll Commands */
#define SSD1306_SET_VERTICAL_SCROLL_AREA                           0xA3
#define SSD1306_ACTIVATE_SCROLL                                    0x2F
#define SSD1306_DEACTIVATE_SCROLL                                  0x2E
#define SSD1306_RIGHT_HORIZONTAL_SCROLL                            0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL                             0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL               0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL                0x2A
#define SSD1306_INVERTDISPLAY                                      0xA7
/* SSD1306 Configuration Commands */
#define SSD1306_DISPCLK_DIV                                        0x80
#define SSD1306_MULT_64                                            0x3F
#define SSD1306_DISP_OFFSET_VAL                                    0x00
#define SSD1306_COL_START_ADDR                                     0x00                          //Reset to = 0
#define SSD1306_COL_END_ADDR                                       (LED_DISPLAY_WIDTH - 1)        //Reset to = 127
#define SSD1306_PG_START_ADDR                                      0x00
#define SSD1306_PG_END_ADDR                                        7
#define SSD1306_CHARGE_PUMP_EN                                     0x14
#define SSD1306_CONFIG_COM_PINS_64                                 0x12
#define SSD1306_CONFIG_COM_PINS_32                                 0x02
//Enable COM left to right Re-map
#define SSD1306_CONTRAST_VAL                                       0xCF                         //207
#define SSD1306_PRECHARGE_VAL                                      0xF1
#define SSD1306_VCOMH_VAL                                          0x40
#define SSD1306_MULT_DAT                                           (LED_DISPLAY_HEIGHT - 1)
#define SSD1306_HOR_MM                                             0x00

#define LED_DISPLAY_HEIGHT 64

void OLED_writeRegister(uint8_t reg, uint8_t v)  {
    uint8_t gTxPacket[2] = { reg, v };
    DL_I2C_fillControllerTXFIFO(tar_i2c.pModule, &gTxPacket[0], 2);

    /* Wait for I2C to be Idle */
    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;
    DL_I2C_startControllerTransfer(tar_i2c.pModule, SSD1306_TARGET, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    /* Poll until the Controller writes all bytes */
    while (DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;

    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    delay_cycles(1000);
}
void OLED_multiWrite(uint8_t sz, uint8_t *c) {

    uint16_t idx = DL_I2C_fillControllerTXFIFO(tar_i2c.pModule, &c[0], sz);
    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    DL_I2C_startControllerTransfer(tar_i2c.pModule, SSD1306_TARGET, DL_I2C_CONTROLLER_DIRECTION_TX, sz);
    while (idx < sz) {
        /*
        //Waiting for txfifo empty
        while (!(DL_I2C_getRawInterruptStatus(tar_i2c.pModule, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY))) {

        }
        //clear interrupt
        DL_I2C_clearInterruptStatus(tar_i2c.pModule, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_EMPTY);
        */
        //Write more data to I2C FIFO
        idx += DL_I2C_fillControllerTXFIFO(tar_i2c.pModule, &c[idx], sz - idx);
    }
    while (DL_I2C_getControllerStatus(tar_i2c.pModule) &
           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
        ;

    while (!(DL_I2C_getControllerStatus(tar_i2c.pModule) & DL_I2C_CONTROLLER_STATUS_IDLE))
        ;

    delay_cycles(1000);
}

#define MAX_CHUNK_SIZE 129
uint8_t chunk[MAX_CHUNK_SIZE] = {0};
#define SCREEN_SIZE (LED_DISPLAY_WIDTH*LED_DISPLAY_HEIGHT/8)+MAX_CHUNK_SIZE
static const uint8_t oledScreen[] = {
0x7f, 0x09, 0x09, 0x09, 0x01, 0x00, 0x7f, 0x09, 0x09, 0x09, 0x06, 0x00, 0x7f, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x49, 0x49, 0x49, 0x36, 0x00, 0x38, 0x44,
0x44, 0x44, 0x38, 0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x04, 0x04, 0x3f, 0x44, 0x24, 0x00, 0x00, 0x44, 0x7d, 0x40, 0x00, 0x00, 0x7c, 0x08, 0x04, 0x04, 0x78, 0x00, 0x18, 0xa4, 0xa4, 0x9c,
0x78, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0x70, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xf0, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x70, 0x38, 0x1c, 0x3e, 0x77, 0xe3, 0xc1, 0x80, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe,
0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x0c, 0x06, 0x03, 0x03, 0x07, 0x8e, 0xfc, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x84, 0xde, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xc4, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf7, 0xf7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x37, 0x36, 0x00, 0x01, 0x03, 0x07, 0x06, 0x06, 0x02, 0x03, 0x07, 0x07,
0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0xf0, 0xf0, 0x00, 0x3f, 0x3f, 0x03, 0x02, 0x06, 0x06, 0x07, 0x03, 0x01, 0x00, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x38, 0x3c, 0xbe, 0xff, 0xcf, 0xc3, 0x41, 0x00, 0x00, 0x01, 0xc3, 0xcf, 0xff, 0xfe, 0x3c, 0x38, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xc7, 0x83, 0x01, 0x81, 0x83,
0xc6, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0x06, 0x03, 0x03, 0x03, 0x03, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x03, 0x01, 0x01, 0x00, 0x00, 0x8e, 0x9f, 0x1b, 0x39, 0x31, 0x73,
0xe3, 0xc0, 0x00, 0x00, 0x80, 0x80, 0xc4, 0xe6, 0xe7, 0x7f, 0x7f, 0x39, 0x19, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x39, 0x3b, 0x7b, 0xef, 0xce, 0xc4, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x80, 0x80, 0x83, 0x83, 0x80, 0x80, 0x80, 0x88, 0x98, 0xb1, 0xb1, 0xb1, 0xb1, 0xb1,
0x98, 0x9f, 0x87, 0x80, 0x80, 0x80, 0x83, 0x83, 0x80, 0x80, 0x80, 0x80, 0x80, 0x83, 0x83, 0x80, 0x80, 0x80, 0x80, 0x81, 0x83, 0x83, 0x83, 0x83, 0x82, 0x80, 0x81, 0x83, 0x83, 0x83, 0xc3, 0xc3,
0x41, 0x40, 0x40, 0x61, 0x61, 0x31, 0x31, 0x3b, 0x1e, 0x1e, 0x0e, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x0e, 0x1e, 0x1e, 0x1a, 0x33, 0x33, 0x23, 0x61, 0x61, 0x40, 0x40, 0x40,
0x40, 0xc0, 0xc0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
//uint8_t oledScreen[SCREEN_SIZE] = {0};

inline void clearMem(uint8_t *b, uint16_t sz) {
    for (int x = 0; x < sz; x++) {
        b[x] = 0;
    }
}

void DisplayScreen() {
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_COL_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_COL_START_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_COL_END_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PAGEADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PG_START_ADDR);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PG_END_ADDR);

    short loop_1 = 0, loop_2 = 0;
    short index = 0x00;
    int max = LED_DISPLAY_HEIGHT * LED_DISPLAY_WIDTH / 8;
    for (loop_1 = 0; loop_1 < 1024; loop_1++) {
        chunk[0] = 0x40;
        for(loop_2 = 1; loop_2 < MAX_CHUNK_SIZE; loop_2++) {
            chunk[loop_2] = oledScreen[index++];
        }
        
        OLED_multiWrite(MAX_CHUNK_SIZE, chunk);
        clearMem(chunk, MAX_CHUNK_SIZE);
        
        if (index >= max)
            break;
    }
}

static const DL_I2C_ClockConfig gTAR_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};
void System_InitOLED() {
    DL_I2C_disablePower(tar_i2c.pModule);
    DL_I2C_reset(tar_i2c.pModule);
    DL_I2C_enablePower(tar_i2c.pModule);

    DL_I2C_setClockConfig(tar_i2c.pModule, (DL_I2C_ClockConfig *) &gTAR_I2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(tar_i2c.pModule);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(tar_i2c.pModule);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(tar_i2c.pModule, 7);
    DL_I2C_setControllerTXFIFOThreshold(tar_i2c.pModule, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(tar_i2c.pModule, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(tar_i2c.pModule);
    /* Enable module */
    DL_I2C_enableController(tar_i2c.pModule);

    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPLAY_OFF);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_MULTIPLEX);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_MULT_DAT);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_DISP_CLK);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPCLK_DIV);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_DISP_OFFSET);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISP_OFFSET_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_DISP_START_LINE);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CONFIG_CHARGE_PUMP);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CHARGE_PUMP_EN);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_MEM_ADDR_MODE);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_HOR_MM);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SEG_REMAP);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_COMSCANDEC);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_COMPINS);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CONFIG_COM_PINS_64);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_PRECHARGE);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_PRECHARGE_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_VCOMDETECT);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_VCOMH_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPLAYALLON_RESUME);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_NORMAL_DISPLAY);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_SET_CONTRAST);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_CONTRAST_VAL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DEACTIVATE_SCROLL);
    OLED_writeRegister(SSD1306_CNTRL_CMD, SSD1306_DISPLAYON);
    DisplayScreen();
    DL_I2C_disableController(tar_i2c.pModule);

    DL_I2C_disablePower(tar_i2c.pModule);
    DL_I2C_reset(tar_i2c.pModule);
    DL_I2C_enablePower(tar_i2c.pModule);
    DL_I2C_setClockConfig(TAR_I2C_INST,
        (DL_I2C_ClockConfig *) &gTAR_I2CClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(TAR_I2C_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(TAR_I2C_INST);

    /* Configure Target Mode */
    DL_I2C_setTargetOwnAddress(TAR_I2C_INST, TAR_I2C_TARGET_OWN_ADDR);
    DL_I2C_setTargetOwnAddressAlternate(TAR_I2C_INST, TAR_I2C_TARGET_SEC_OWN_ADDR);
    DL_I2C_enableTargetOwnAddressAlternate(TAR_I2C_INST);
    DL_I2C_setTargetTXFIFOThreshold(TAR_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setTargetRXFIFOThreshold(TAR_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableTargetTXEmptyOnTXRequest(TAR_I2C_INST);

    DL_I2C_enableTargetClockStretching(TAR_I2C_INST);
}

void System_init(void)
{
    System_pendingEvents = SYSEVENT__NONE_PENDING;

    SYSCFG_DL_init();
    delay_cycles(1000000);
    System_InitOLED();
    //SYSCFG_DL_init();

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
