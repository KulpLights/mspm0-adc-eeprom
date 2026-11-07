/*
 *  adc_input.c
 *
 *  Shared MSPM0 ADC12 access for the ADC emulation modules.
 */

#include <stdint.h>
#include <ti/driverlib/dl_adc12.h>
#include "adc_input.h"

static const uint8_t adc_input_chLUT[ADC_INPUT_CH_CNT] =
{
    DL_ADC12_INPUT_CHAN_1,
    DL_ADC12_INPUT_CHAN_2,
    DL_ADC12_INPUT_CHAN_3,
    DL_ADC12_INPUT_CHAN_4,
    DL_ADC12_INPUT_CHAN_5,
    DL_ADC12_INPUT_CHAN_6,
    DL_ADC12_INPUT_CHAN_8,
    DL_ADC12_INPUT_CHAN_9
};

static uint16_t adc_input_convertChannel(uint8_t channel)
{
    uint16_t result;

    DL_ADC12_configConversionMem(ADC0, DL_ADC12_MEM_IDX_0, channel,
        DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
        DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED,
        DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_enableConversions(ADC0);
    DL_ADC12_startConversion(ADC0);
    while(!DL_ADC12_getRawInterruptStatus(ADC0,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_clearInterruptStatus(ADC0,
        DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    result = DL_ADC12_getMemResult(ADC0, DL_ADC12_MEM_IDX_0);

    return result;
}

void adc_input_init(void)
{
    adc_input_convertChannel(15);
}

uint16_t adc_input_convert(uint8_t input)
{
    if (input >= ADC_INPUT_CH_CNT)
    {
        return 0;
    }
    return adc_input_convertChannel(adc_input_chLUT[input]) & 0x0FFF;
}
