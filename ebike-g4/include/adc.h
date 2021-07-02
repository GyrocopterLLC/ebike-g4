/******************************************************************************
 * Filename: adc.h
 ******************************************************************************

 Copyright (c) 2020 David Miller

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

#ifndef ADC_H_
#define ADC_H_

#define MAXCOUNT		(4095)
#define MAXCOUNTF		(4095.0f)

#define NUM_ADC_CH		10
#define NUM_CUR_CH		3

// DMA settings
#define ADC1_DMAMUX_REQ     5
#define ADC2_DMAMUX_REQ     36
#define ADC3_DMAMUX_REQ     37
#define ADC4_DMAMUX_REQ     38
#define ADC1_DMACHANNEL     DMA1_Channel1
#define ADC2_DMACHANNEL     DMA1_Channel2
#define ADC3_DMACHANNEL     DMA1_Channel3
#define ADC4_DMACHANNEL     DMA1_Channel4
#define ADC1_DMAMUXCHANNEL  DMAMUX1_Channel0
#define ADC2_DMAMUXCHANNEL  DMAMUX1_Channel1
#define ADC3_DMAMUXCHANNEL  DMAMUX1_Channel2
#define ADC4_DMAMUXCHANNEL  DMAMUX1_Channel3

// Channel definitions
// Two versions are given -
// Version A only uses ADC1 and ADC2, this is compatible with the STM32G4x1 series
// Version B uses all ADC1 through ADC4, this is compatible with STM32G4x3 and x4

#if defined(USE_ADC_1_2_ONLY)
#define ADC_MTEMP_ADC       ADC1
#define ADC_MTEMP_CH        10
#define ADC_FTEMP_ADC       ADC2
#define ADC_FTEMP_CH        10
#define ADC_IA_ADC          ADC1
#define ADC_IA_CH           15
#define ADC_IB_ADC          ADC2
#define ADC_IB_CH           2
#define ADC_IC_ADC          ADC1
#define ADC_IC_CH           1
#define ADC_VA_ADC          ADC2
#define ADC_VA_CH           3
#define ADC_VB_ADC          ADC2
#define ADC_VB_CH           4
#define ADC_VC_ADC          ADC1
#define ADC_VC_CH           12
#define ADC_THR_ADC         ADC2
#define ADC_THR_CH          12
#define ADC_VBUS_ADC        ADC1
#define ADC_VBUS_CH         11

#else

#define ADC_MTEMP_ADC       ADC1
#define ADC_MTEMP_CH        10
#define ADC_FTEMP_ADC       ADC2
#define ADC_FTEMP_CH        10
#define ADC_IA_ADC          ADC3
#define ADC_IA_CH           12
#define ADC_IB_ADC          ADC2
#define ADC_IB_CH           2
#define ADC_IC_ADC          ADC1
#define ADC_IC_CH           1
#define ADC_VA_ADC          ADC2
#define ADC_VA_CH           3
#define ADC_VB_ADC          ADC2
#define ADC_VB_CH           4
#define ADC_VC_ADC          ADC3
#define ADC_VC_CH           1
#define ADC_THR_ADC         ADC2
#define ADC_THR_CH          12
#define ADC_VBUS_ADC        ADC4
#define ADC_VBUS_CH         3

#endif

// Common for all ADCs
#define ADC_VTS_CH          16
#define ADC_VBAT_CH         17
#define ADC_VREFINT_CH      18


#define VREFINTDEFAULT	(1.212f) // From the STM32G4 spec sheet
#define VREFINTCAL_MIN	(1570) // Approximately 1.15V. Spec sheet minimum is 1.182V
#define VREFINTCAL_MAX	(1734) // Approximately 1.27V. Spec sheet maximum is 1.232V
#define ADC_FACTORY_CAL_VOLTAGE (3.0f) // Calibration is done at 3.0V
#define ADC_VREG_STARTUP_DELAY  (1)  // Only needs 20us
#define ADC_STARTUP_DELAY_MS    (50) // Wait this long for any startup spikes in analog values to die down
#define ADC_NUM_INTEG_SAMPLES (128) // Number of samples to integrate when measuring at startup

typedef struct _config_adc {
    float Shunt_Resistance;
    float Inverse_TIA_Gain;
    float Vbus_Ratio;
    float Vphase_Ratio;
    float Thermistor_Fixed_R;
    float Thermistor_R25;
    float Thermistor_Beta;
} Config_ADC;

typedef enum {
    ADC_IA = 0,
    ADC_IB,
    ADC_IC,
    ADC_VA,
    ADC_VB,
    ADC_VC,
    ADC_VBUS,
    ADC_FTEMP,
    ADC_MTEMP,
    ADC_THR,
} ADC_OutputTypeDef;


void ADC_InjSeqComplete(void);
void ADC_RegSeqComplete(void);
void ADC_Init(void);
float ADC_GetCurrent(uint8_t which_cur);
uint16_t ADC_Raw(uint8_t which_cur);
float ADC_ConvertToAmps(int32_t rawCurrentReading);
float ADC_GetThrottle(void);
float ADC_GetVbus(void);
float ADC_GetVref(void);
float ADC_GetPhaseVoltage(uint8_t which_phase);
void ADC_SetNull(uint8_t which_cur, uint16_t nullVal);
float ADC_GetFetTempDegC(void);

uint8_t ADC_SetRShunt(float new_rshunt);
float ADC_GetRShunt(void);
uint8_t ADC_SetVbusRatio(float new_ratio);
float ADC_GetVbusRatio(void);
uint8_t ADC_SetVphaseRatio(float new_ratio);
float ADC_GetVphaseRatio(void);
uint8_t ADC_SetThermFixedR(float new_fixed_r);
float ADC_GetThermFixedR(void);
uint8_t ADC_SetThermR25(float new_r25);
float ADC_GetThermR25(void);
uint8_t ADC_SetThermBeta(float new_beta);
float ADC_GetThermBeta(void);

void ADC_SaveVariables(void);
void ADC_LoadVariables(void);

uint8_t uiADC_SetRShunt(uint8_t* valptr);
uint8_t uiADC_GetRShunt(uint8_t* valptr);
uint8_t uiADC_SetVbusRatio(uint8_t* valptr);
uint8_t uiADC_GetVbusRatio(uint8_t* valptr);
uint8_t uiADC_SetVphaseRatio(uint8_t* valptr);
uint8_t uiADC_GetVphaseRatio(uint8_t* valptr);
uint8_t uiADC_SetThermFixedR(uint8_t* valptr);
uint8_t uiADC_GetThermFixedR(uint8_t* valptr);
uint8_t uiADC_SetThermR25(uint8_t* valptr);
uint8_t uiADC_GetThermR25(uint8_t* valptr);
uint8_t uiADC_SetThermBeta(uint8_t* valptr);
uint8_t uiADC_GetThermBeta(uint8_t* valptr);

#endif /* ADC_H_ */
