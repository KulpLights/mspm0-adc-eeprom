/*
 * Copyright (c) 2021, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_LFXT_PORT                                                     GPIOA
#define GPIO_LFXIN_PIN                                             DL_GPIO_PIN_3
#define GPIO_LFXIN_IOMUX                                          (IOMUX_PINCM8)
#define GPIO_LFXOUT_PIN                                            DL_GPIO_PIN_4
#define GPIO_LFXOUT_IOMUX                                         (IOMUX_PINCM9)



/* Defines for SPI_target */
#define SPI_target_INST                                                    SPI1
#define SPI_target_INST_IRQHandler                              SPI1_IRQHandler
#define SPI_target_INST_INT_IRQN                                  SPI1_INT_IRQn
#define GPIO_SPI_target_PICO_PORT                                         GPIOB
#define GPIO_SPI_target_PICO_PIN                                  DL_GPIO_PIN_8
#define GPIO_SPI_target_IOMUX_PICO                              (IOMUX_PINCM25)
#define GPIO_SPI_target_IOMUX_PICO_FUNC              IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_target_POCI_PORT                                         GPIOB
#define GPIO_SPI_target_POCI_PIN                                  DL_GPIO_PIN_7
#define GPIO_SPI_target_IOMUX_POCI                              (IOMUX_PINCM24)
#define GPIO_SPI_target_IOMUX_POCI_FUNC              IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_target */
#define GPIO_SPI_target_SCLK_PORT                                         GPIOB
#define GPIO_SPI_target_SCLK_PIN                                  DL_GPIO_PIN_9
#define GPIO_SPI_target_IOMUX_SCLK                              (IOMUX_PINCM26)
#define GPIO_SPI_target_IOMUX_SCLK_FUNC              IOMUX_PINCM26_PF_SPI1_SCLK



/* Defines for PIN_0: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_GRP_0_PIN_0_PORT                                            (GPIOA)
#define GPIO_GRP_0_PIN_0_PIN                                     (DL_GPIO_PIN_0)
#define GPIO_GRP_0_PIN_0_IOMUX                                    (IOMUX_PINCM1)

/* Defines for SPI_CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_GRP_0_SPI_CS_PORT                                           (GPIOB)
// pins affected by this interrupt request:["SPI_CS"]
#define GPIO_GRP_0_INT_IRQN                                     (GPIOB_INT_IRQn)
#define GPIO_GRP_0_INT_IIDX                     (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_GRP_0_SPI_CS_IIDX                               (DL_GPIO_IIDX_DIO6)
#define GPIO_GRP_0_SPI_CS_PIN                                    (DL_GPIO_PIN_6)
#define GPIO_GRP_0_SPI_CS_IOMUX                                  (IOMUX_PINCM23)





/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SPI_target_init(void);

void SYSCFG_DL_RTC_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#endif /* ti_msp_dl_config_h */
