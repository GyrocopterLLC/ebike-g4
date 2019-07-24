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

#define MAXCOUNT		(4095)
#define MAXCOUNTF		(4095.0f)

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
#define ADC_FACTORY_CAL_VOLTAGE (3.3f) // Calibration is done at 3.3V
#define ADC_STARTUP_DELAY_MS    (50) // Wait this long for any startup spikes in analog values to die down
#define ADC_NUM_NULLING_SAMPLES (128) // Number of samples to integrate when nulling current sensors at startup

typedef struct _config_adc {
    float Inverse_TIA_Gain;
    float Vbus_Ratio;
    float Thermistor_Fixed_R;
    float Thermistor_R25;
    float Thermistor_Beta;
    float Inverse_Therm_Beta;
} Config_ADC;

typedef enum {
    ADC_IA = 0,
    ADC_IB = 1,
    ADC_IC = 2,
    ADC_VBUS = 3,
    ADC_TEMP = 4,
    ADC_THR1 = 5,
    ADC_THR2 = 6,
    ADC_VREFINT = 7
} ADC_OutputTypeDef;

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

uint8_t adcSetInverseTIAGain(float new_gain);
float adcGetInverseTIAGain(void);
uint8_t adcSetVbusRatio(float new_ratio);
float adcGetVbusRatio(void);
uint8_t adcSetThermFixedR(float new_fixed_r);
float adcGetThermFixedR(void);
uint8_t adcSetThermR25(float new_r25);
float adcGetThermR25(void);
uint8_t adcSetThermBeta(float new_beta);
float adcGetThermBeta(void);

void adcSaveVariables(void);
void adcLoadVariables(void);

#if defined(USING_OLD_ADC_VER)
void ADC_Init_OldVer(void);
uint16_t ADC_GetData_OldVer(ADC_OutputTypeDef nadc);
void ADC_InjectedConversionCompleteCallBack_OldVer(void);
#endif

#endif /* ADC_H_ */
