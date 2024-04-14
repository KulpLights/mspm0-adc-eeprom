/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  DO NOT EDIT - This file is generated for the MSPM0L110X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0L110X

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

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for WAKEUP_TIMER */
#define WAKEUP_TIMER_INST                                                (TIMG0)
#define WAKEUP_TIMER_INST_IRQHandler                            TIMG0_IRQHandler
#define WAKEUP_TIMER_INST_INT_IRQN                              (TIMG0_INT_IRQn)
#define WAKEUP_TIMER_INST_LOAD_VALUE                                    (16383U)




/* Defines for TAR_I2C */
#define TAR_I2C_INST                                                        I2C0
#define TAR_I2C_INST_IRQHandler                                  I2C0_IRQHandler
#define TAR_I2C_INST_INT_IRQN                                      I2C0_INT_IRQn
#define TAR_I2C_TARGET_OWN_ADDR                                             0x50
#define TAR_I2C_TARGET_SEC_OWN_ADDR                                         0x20
#define GPIO_TAR_I2C_SDA_PORT                                              GPIOA
#define GPIO_TAR_I2C_SDA_PIN                                       DL_GPIO_PIN_0
#define GPIO_TAR_I2C_IOMUX_SDA                                    (IOMUX_PINCM1)
#define GPIO_TAR_I2C_IOMUX_SDA_FUNC                     IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_TAR_I2C_SCL_PORT                                              GPIOA
#define GPIO_TAR_I2C_SCL_PIN                                       DL_GPIO_PIN_1
#define GPIO_TAR_I2C_IOMUX_SCL                                    (IOMUX_PINCM2)
#define GPIO_TAR_I2C_IOMUX_SCL_FUNC                     IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for PER_SPI */
#define PER_SPI_INST                                                       SPI0
#define PER_SPI_INST_IRQHandler                                 SPI0_IRQHandler
#define PER_SPI_INST_INT_IRQN                                     SPI0_INT_IRQn
#define GPIO_PER_SPI_PICO_PORT                                            GPIOA
#define GPIO_PER_SPI_PICO_PIN                                     DL_GPIO_PIN_9
#define GPIO_PER_SPI_IOMUX_PICO                                 (IOMUX_PINCM10)
#define GPIO_PER_SPI_IOMUX_PICO_FUNC                 IOMUX_PINCM10_PF_SPI0_PICO
#define GPIO_PER_SPI_POCI_PORT                                            GPIOA
#define GPIO_PER_SPI_POCI_PIN                                    DL_GPIO_PIN_19
#define GPIO_PER_SPI_IOMUX_POCI                                 (IOMUX_PINCM20)
#define GPIO_PER_SPI_IOMUX_POCI_FUNC                 IOMUX_PINCM20_PF_SPI0_POCI
/* GPIO configuration for PER_SPI */
#define GPIO_PER_SPI_SCLK_PORT                                            GPIOA
#define GPIO_PER_SPI_SCLK_PIN                                    DL_GPIO_PIN_11
#define GPIO_PER_SPI_IOMUX_SCLK                                 (IOMUX_PINCM12)
#define GPIO_PER_SPI_IOMUX_SCLK_FUNC                 IOMUX_PINCM12_PF_SPI0_SCLK
#define GPIO_PER_SPI_CS_PORT                                              GPIOA
#define GPIO_PER_SPI_CS_PIN                                       DL_GPIO_PIN_3
#define GPIO_PER_SPI_IOMUX_CS                                    (IOMUX_PINCM4)
#define GPIO_PER_SPI_IOMUX_CS_FUNC               IOMUX_PINCM4_PF_SPI0_CS1_POCI1



/* Port definition for Pin Group MCU_GPIO */
#define MCU_GPIO_PORT                                                    (GPIOA)

/* Defines for MCU_GPIO0_1: GPIOA.18 with pinCMx 19 on package pin 14 */
#define MCU_GPIO_MCU_GPIO0_1_PIN                                (DL_GPIO_PIN_18)
#define MCU_GPIO_MCU_GPIO0_1_IOMUX                               (IOMUX_PINCM19)


/* Defines for WWDT */
#define WWDT0_INST                                                       (WWDT0)
#define WWDT0_INT_IRQN                                          (WWDT0_INT_IRQn)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_DEBUG_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_WAKEUP_TIMER_init(void);
void SYSCFG_DL_TAR_I2C_init(void);
void SYSCFG_DL_PER_SPI_init(void);

void SYSCFG_DL_WWDT0_init(void);


#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
