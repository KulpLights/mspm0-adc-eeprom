/*
 *  adc_input.h
 *
 *  Shared MSPM0 ADC12 access for the ADC emulation modules.  The emulated
 *  chips expose eight analog inputs, numbered 0-7, which map onto the
 *  MSPM0 ADC12 input channels listed in adc_input.c.
 */

#ifndef EMULATION_ADC_INPUT_H_
#define EMULATION_ADC_INPUT_H_

#include <stdint.h>

#define ADC_INPUT_CH_CNT 8

/* Power up the ADC with a dummy conversion of the internal supply monitor */
extern void adc_input_init(void);

/* Convert emulated input 0-7 and return the 12 bit result, referenced
 * to VDDA */
extern uint16_t adc_input_convert(uint8_t input);

#endif /* EMULATION_ADC_INPUT_H_ */
