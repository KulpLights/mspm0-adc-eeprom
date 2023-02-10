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
    SYSCFG_DL_I2C_target_init();
    SYSCFG_DL_RTC_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_I2C_reset(I2C_target_INST);
    DL_RTC_reset(RTC);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_I2C_enablePower(I2C_target_INST);
    DL_RTC_enablePower(RTC);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_LFXOUT_IOMUX);


    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_target_IOMUX_SDA,
        GPIO_I2C_target_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_target_IOMUX_SCL,
        GPIO_I2C_target_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_target_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_target_IOMUX_SCL);

    DL_GPIO_initDigitalOutput(GPIO_GRP_0_PIN_0_IOMUX);

    DL_GPIO_clearPins(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN);
    DL_GPIO_enableOutput(GPIO_GRP_0_PORT, GPIO_GRP_0_PIN_0_PIN);



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





static const DL_I2C_ClockConfig gI2C_targetClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_8,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_target_init(void) {

    DL_I2C_setClockConfig(I2C_target_INST,
        (DL_I2C_ClockConfig *) &gI2C_targetClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_target_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_target_INST);

    /* Configure Target Mode */
    DL_I2C_setTargetOwnAddress(I2C_target_INST, I2C_target_TARGET_OWN_ADDR);
    DL_I2C_setTargetTXFIFOThreshold(I2C_target_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setTargetRXFIFOThreshold(I2C_target_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);

    /* Configure Interrupts */
    DL_I2C_enableInterrupt(I2C_target_INST,
                           DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_TARGET_START |
                           DL_I2C_INTERRUPT_TARGET_STOP |
                           DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER);

    /* Enable module */
    DL_I2C_enableTarget(I2C_target_INST);

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

