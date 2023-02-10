/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_SPI_backupConfig gSPI_targetBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_SPI_target_init();
    SYSCFG_DL_RTC_init();
    /* Ensure backup structures have no valid state */
	gSPI_targetBackup.backupRdy 	= false;

}
/* 
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;
    
	retStatus &= DL_SPI_saveConfiguration(SPI_target_INST, &gSPI_targetBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_SPI_restoreConfiguration(SPI_target_INST, &gSPI_targetBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_SPI_reset(SPI_target_INST);
    DL_RTC_reset(RTC);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_SPI_enablePower(SPI_target_INST);
    DL_RTC_enablePower(RTC);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);


    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_target_IOMUX_SCLK, GPIO_SPI_target_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_target_IOMUX_PICO, GPIO_SPI_target_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_target_IOMUX_POCI, GPIO_SPI_target_IOMUX_POCI_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_GRP_0_PIN_0_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_GRP_0_SPI_CS_IOMUX);

    DL_GPIO_clearPins(GPIOA, GPIO_GRP_0_PIN_0_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_GRP_0_PIN_0_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_6_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_GRP_0_SPI_CS_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_GRP_0_SPI_CS_PIN);



}



static const DL_SYSCTL_LFCLKConfig gSYSCTLConfig = {
    .lowCap   = false,
    .monitor  = false,
    .xt1Drive = DL_SYSCTL_LFXT_DRIVE_STRENGTH_HIGHER,
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);



    DL_SYSCTL_setLFCLKSourceLFXT((DL_SYSCTL_LFCLKConfig *) &gSYSCTLConfig);

    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setPowerPolicyRUN0SLEEP0();
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);


}





static const DL_SPI_Config gSPI_target_config = {
    .mode        = DL_SPI_MODE_PERIPHERAL,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = 8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
};

static const DL_SPI_ClockConfig gSPI_target_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_target_init(void) {
    DL_SPI_setClockConfig(SPI_target_INST, (DL_SPI_ClockConfig *) &gSPI_target_clockConfig);

    DL_SPI_init(SPI_target_INST, (DL_SPI_Config *) &gSPI_target_config);


    /* Configure Peripheral mode */
    DL_SPI_disablePeripheralAlignDataOnChipSelect(SPI_target_INST);
    /* Calculated time for RX timeout: 31.25 ns */
    DL_SPI_setPeripheralReceiveTimeout(SPI_target_INST, 1);



    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_target_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_3_4_EMPTY);

    DL_SPI_enableInterrupt(SPI_target_INST, (DL_SPI_INTERRUPT_IDLE |
		DL_SPI_INTERRUPT_RX));

    /* Enable module */
    DL_SPI_enable(SPI_target_INST);
}

static const DL_RTC_Calendar gRTCCalendarConfig = {
    .seconds    = 0,   /* Seconds = 0 */
    .minutes    = 0,   /* Minute = 0 */
    .hours      = 0,   /* Hour = 0 */
    .dayOfWeek  = 0,    /* Day of week = 0 (Sunday) */
    .dayOfMonth = 1,    /* Day of month = 1 */
    .month      = 1,    /* Month = 1 (January) */
    .year       = 2022, /* Year = 2022 */
};

SYSCONFIG_WEAK void SYSCFG_DL_RTC_init(void)
{
    /* Initialize RTC Calendar */
    DL_RTC_initCalendar(RTC, gRTCCalendarConfig, DL_RTC_FORMAT_BINARY);





}

