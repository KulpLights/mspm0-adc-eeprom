/*
 * i2c_target.h
 *
 *  Created on: Oct 27, 2022
 *      Author: munan
 */

#ifndef I2C_TARGET_H_
#define I2C_TARGET_H_
#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "rtc.h"

/*!****************************************************************************
 *  @file       i2c_target.h
 *  @brief      I2C HAL Interface
 *  @defgroup   I2C  Inter-Integrated Circuit (I2C)
 *
 *  # Overview
 *  The I2C hal layer provides a layer of abstraction above the base MSPM0 I2C
 *  Although the MSPM0 I2C has several native hardware features to take advantage of
 *  For maximum flexibility we implement a HAL in case the code ever needs to be ported
 *  to a platform without a hardware I2C or if the feature set is slightly different.
 *
 *  <hr>
 ******************************************************************************
 */
/**
 *  I2C interface header
 *      Functions associated with setting and configuring the I2C.
 *      This set of functions is used to interface with the hardware I2C on MSPM0
**/
/** @addtogroup I2C
 * @{
 */

/**
 * @brief I2C interrupt service routine (I2C ISR)
 * This ISR implements the state machine for reading and writing RTC registers over I2C.
 * The I2C controller addresses the device issuing a write transaction
**/
void I2C_target_INST_IRQHandler(void);

/**
 * @brief I2C target transmit operation
 * This function retrieves and transmits the appropriate RTC struct member in response to I2C master read.
**/
void i2c_targetTx(uint8_t regPointer);

/**
 * @brief I2C target receive operation
 * This function receives and updates the appropriate RTC struct member in response to I2C master read.
**/
void i2c_targetRx(uint8_t regPointer, uint8_t inputData);

#endif /* I2C_TARGET_H_ */
