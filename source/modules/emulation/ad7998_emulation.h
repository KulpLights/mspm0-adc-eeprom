/*
 *  ad7998_emulation.h
 *
 *  AD7998 I2C ADC Emulation Library
 *
 *  Emulates the 8 channel, 12 bit AD7998 so the stock Linux ad799x driver
 *  can read the inputs.  Unlike ad7291, ad799x is built in common
 *  distribution kernels, including the Raspberry Pi one.
 */

#ifndef EMULATION_AD7998_EMULATION_H_
#define EMULATION_AD7998_EMULATION_H_

#include "communication/i2c_tar_driver.h"

extern void ad7998_open(i2c_tar_driver_t *pI2CTarDriverInst);

extern void ad7998_i2c_rx_callback(uint32_t bytes, \
    i2c_tar_driver_call_trig_t trig);

extern void ad7998_i2c_tx_callback(void);

#endif /* EMULATION_AD7998_EMULATION_H_ */
