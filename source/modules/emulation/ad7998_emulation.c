/*
 *  ad7998_emulation.c
 *
 *  AD7998 I2C ADC Emulation Library
 *
 *  Every transfer starts with an address pointer byte.  Its low nibble
 *  selects a register; its high nibble is a command:
 *
 *    0000         no conversion, plain register access
 *    0111         convert the channels enabled in the configuration
 *                 register in sequence, one 16 bit result per channel
 *    1xxx         convert the single channel xxx (0-7)
 *
 *  Reading after a conversion command returns conversion results.  Each
 *  result is big endian: bit 15 is the alert flag, bits 14-12 the channel,
 *  bits 11-0 the data.  The AD7998 uses its supply as the reference, which
 *  matches the MSPM0 converting against VDDA.
 *
 *  The Linux ad799x driver probes by writing the configuration register
 *  and reading it back, reads single channels with command 1xxx, and reads
 *  triggered buffers with command 0111.
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "ad7998_emulation.h"
#include "adc_input.h"

enum ad7998_regs
{
    AD7998_REG_CONV_RES = 0x0,
    AD7998_REG_ALERT_STAT = 0x1,
    AD7998_REG_CONFIG = 0x2,
    AD7998_REG_CYCLE_TMR = 0x3,
    /* 0x4-0xF: DATA_LOW, DATA_HIGH and HYST for channels 1-4 */
    AD7998_REG_CNT = 0x10
};

#define AD7998_CMD_SHIFT        4
#define AD7998_CMD_MASK         0xF0
#define AD7998_REG_MASK         0x0F
#define AD7998_CMD_NONE         0x0
#define AD7998_CMD_SEQUENCE     0x7
#define AD7998_CMD_SINGLE       0x8

/* Configuration register: D11-D4 enable CH8-CH1 for sequence mode */
#define AD7998_CONFIG_CH_SHIFT  4
#define AD7998_CONFIG_MASK      0x0FFF
#define AD7998_CONFIG_DEFAULT   0x0008      /* FLTR set, no channels */

#define AD7998_LIMIT_MASK       0x0FFF
#define AD7998_LIMIT_HIGH_DEFAULT 0x0FFF

typedef enum
{
    AD7998_MODE_REGISTER,
    AD7998_MODE_SINGLE,
    AD7998_MODE_SEQUENCE
} ad7998_mode_t;

static uint16_t ad7998_regFile[AD7998_REG_CNT];
static uint8_t ad7998_regPtr;
static ad7998_mode_t ad7998_mode;
static uint8_t ad7998_singleChannel;
static uint8_t ad7998_nextSeqChannel;

static i2c_tar_driver_t *ad7998_i2c_tar_driver_handle;

static bool ad7998_isLimitHigh(uint8_t reg)
{
    return (reg >= 0x4) && (((reg - 0x4) % 3) == 1);
}

void ad7998_open(i2c_tar_driver_t *pI2CTarDriverInst)
{
    uint8_t i;

    ad7998_i2c_tar_driver_handle = pI2CTarDriverInst;

    for (i = 0; i < AD7998_REG_CNT; i++)
    {
        ad7998_regFile[i] = ad7998_isLimitHigh(i) ? AD7998_LIMIT_HIGH_DEFAULT : 0;
    }
    ad7998_regFile[AD7998_REG_CONFIG] = AD7998_CONFIG_DEFAULT;
    ad7998_regPtr = AD7998_REG_CONV_RES;
    ad7998_mode = AD7998_MODE_REGISTER;
    ad7998_singleChannel = 0;
    ad7998_nextSeqChannel = 0;

    adc_input_init();
}

static void ad7998_setPointer(uint8_t ptr)
{
    uint8_t cmd = (ptr & AD7998_CMD_MASK) >> AD7998_CMD_SHIFT;

    ad7998_regPtr = ptr & AD7998_REG_MASK;
    if (cmd & AD7998_CMD_SINGLE)
    {
        ad7998_mode = AD7998_MODE_SINGLE;
        ad7998_singleChannel = cmd & 0x7;
    }
    else if (cmd == AD7998_CMD_SEQUENCE)
    {
        ad7998_mode = AD7998_MODE_SEQUENCE;
        ad7998_nextSeqChannel = 0;
    }
    else
    {
        /* Plain register pointer write.  The reserved command codes
         * 0001-0110 are treated the same way. */
        ad7998_mode = AD7998_MODE_REGISTER;
    }
}

static void ad7998_writeRegister(uint8_t reg, uint16_t value)
{
    switch (reg)
    {
    case AD7998_REG_CONV_RES:
        /* read only */
        break;
    case AD7998_REG_ALERT_STAT:
        /* write to clear; no alerts are ever raised */
        ad7998_regFile[reg] = 0;
        break;
    case AD7998_REG_CONFIG:
        ad7998_regFile[reg] = value & AD7998_CONFIG_MASK;
        break;
    case AD7998_REG_CYCLE_TMR:
        ad7998_regFile[reg] = value & 0xFF;
        break;
    default:
        ad7998_regFile[reg] = value & AD7998_LIMIT_MASK;
        break;
    }
}

void ad7998_i2c_rx_callback(uint32_t bytes, i2c_tar_driver_call_trig_t trig)
{
    uint8_t ptr;
    uint16_t value;

    if (bytes == 0)
    {
        return;
    }

    ptr = I2CTarDriver_read(ad7998_i2c_tar_driver_handle);
    bytes--;
    ad7998_setPointer(ptr);

    if (bytes == 0)
    {
        return;
    }

    /* A write to the cycle timer register is a single byte; every other
     * register is 16 bits, most significant byte first. */
    if (bytes == 1 || ad7998_regPtr == AD7998_REG_CYCLE_TMR)
    {
        value = I2CTarDriver_read(ad7998_i2c_tar_driver_handle);
    }
    else
    {
        value = (uint16_t)I2CTarDriver_read(ad7998_i2c_tar_driver_handle) << 8;
        value |= I2CTarDriver_read(ad7998_i2c_tar_driver_handle);
    }
    ad7998_writeRegister(ad7998_regPtr, value);
}

static uint16_t ad7998_convert(uint8_t channel)
{
    return ((uint16_t)channel << 12) | adc_input_convert(channel);
}

static uint16_t ad7998_nextSequenceResult(void)
{
    uint8_t enabled;
    uint8_t channel;
    uint8_t i;

    enabled = (ad7998_regFile[AD7998_REG_CONFIG] >> AD7998_CONFIG_CH_SHIFT) & 0xFF;
    if (enabled == 0)
    {
        return 0;
    }
    for (i = 0; i < ADC_INPUT_CH_CNT; i++)
    {
        channel = ad7998_nextSeqChannel;
        ad7998_nextSeqChannel = (ad7998_nextSeqChannel + 1) % ADC_INPUT_CH_CNT;
        if (enabled & (1 << channel))
        {
            break;
        }
    }
    return ad7998_convert(channel);
}

void ad7998_i2c_tx_callback(void)
{
    uint16_t value;

    switch (ad7998_mode)
    {
    case AD7998_MODE_SINGLE:
        value = ad7998_convert(ad7998_singleChannel);
        break;
    case AD7998_MODE_SEQUENCE:
        value = ad7998_nextSequenceResult();
        break;
    default:
        if (ad7998_regPtr == AD7998_REG_CYCLE_TMR)
        {
            /* 8 bit register; a second byte read just repeats it */
            I2CTarDriver_write(ad7998_i2c_tar_driver_handle, \
                (uint8_t)ad7998_regFile[ad7998_regPtr]);
            return;
        }
        if (ad7998_regPtr == AD7998_REG_CONV_RES)
        {
            /* Reading the result register without a command converts the
             * last selected single channel again */
            value = ad7998_convert(ad7998_singleChannel);
        }
        else
        {
            value = ad7998_regFile[ad7998_regPtr];
        }
        break;
    }

    I2CTarDriver_write(ad7998_i2c_tar_driver_handle, (uint8_t)(value >> 8));
    I2CTarDriver_write(ad7998_i2c_tar_driver_handle, (uint8_t)value);
}
