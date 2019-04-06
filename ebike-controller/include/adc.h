/******************************************************************************
 * Filename: adc.h
 ******************************************************************************

Copyright (c) 2019 David Miller

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
// Used resources:
// ADC1, ADC2, ADC3

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"

#define MAXCOUNT		(4096)
#define MAXCOUNTF		(4096.0f)

#define NUM_ADC_CH		8
#define NUM_CUR_CH		3

#define ADC_IA_CH		10
#define ADC_IB_CH		11
#define ADC_IC_CH		12
#define ADC_VBUS_CH		13
#define ADC_TEMP_CH		9
#define ADC_THR1_CH		15
#define ADC_THR2_CH		8
#define VREFINT_CH		17

#define VREFINTDEFAULT	(1.21f) // From the STM32F4 spec sheet
#define VREFINTCAL_MIN	(1400) // Approximately 1.13V. Spec sheet minimum is 1.18V
#define VREFINTCAL_MAX	(1600) // Approximately 1.29V. Spec sheet minimum is 1.24V
#define RSHUNT_INV		(200.0f) // Inverse of 0.001 Ohms
#define INAGAIN_INV		(0.02f) // Inverse of 50x gain (INA213 current amplifier)
#define VBUS_RESISTOR_RATIO	(33.36246f) // Inverse of resistor gain (3.09 / 103.09)

#define TEMP_FIXED_RESISTOR	(10000.0f)
#define THERM_R25			(10000.0f)
#define THERM_B_VALUE		(3435.0f)

//#define ADC_HACKY_OFFSET		(15) // The zero-current points shift by this amount when motor is running
#define ADC_HACKY_OFFSET (0) // Seems to working okay with the .005 resistor...

typedef enum
{
	ADC_IA = 0,
	ADC_IB = 1,
	ADC_IC = 2,
	ADC_VBUS = 3,
	ADC_TEMP = 4,
	ADC_THR1 = 5,
	ADC_THR2 = 6,
	ADC_VREFINT = 7
}ADC_OutputTypeDef;

void adcConvComplete(void);
void adcInit(void);
float adcGetCurrent(uint8_t which_cur);
uint16_t adcRaw(uint8_t which_cur);
float adcConvertToAmps(int32_t rawCurrentReading);
float adcGetThrottle(uint8_t thrnum);
float adcGetVbus(void);
float adcGetVref(void);
void adcSetNull(uint8_t which_cur, uint16_t nullVal);
float adcGetTempDegC(void);

#if defined(USING_OLD_ADC_VER)
void ADC_Init_OldVer(void);
uint16_t ADC_GetData_OldVer(ADC_OutputTypeDef nadc);
void ADC_InjectedConversionCompleteCallBack_OldVer(void);
#endif

#endif /* ADC_H_ */
