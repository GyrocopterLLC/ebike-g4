/*
 * adc.h
 *
 *  Created on: Aug 19, 2015
 *      Author: David
 */

// Used resources:
// ADC1, ADC2, ADC3

#ifndef ADC_H_
#define ADC_H_

#include "stm32f4xx.h"

#define MAXCOUNT		(4096)
#define MAXCOUNTF		(4096.0f)

#define NUM_ADC_CH			7

#define ADC_IA_PIN		0
#define ADC_IB_PIN		1
#define ADC_IC_PIN		2
#define ADC_VBUS_PIN	3
#define ADC_TEMP_PIN	1
#define ADC_THR1_PIN	5
#define ADC_THR2_PIN	0

#define ADC_I_VBUS_THR1_PORT		GPIOC
#define ADC_THR2_AND_TEMP_PORT		GPIOB

#define ADC_I_VBUS_THR1_CLK_ENABLE()		__HAL_RCC_GPIOC_CLK_ENABLE()
#define ADC_THR2_AND_TEMP_CLK_ENABLE()		__HAL_RCC_GPIOB_CLK_ENABLE()

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
#define RSHUNT_INV		(1000.0f) // Inverse of 0.001 Ohms
#define INAGAIN_INV		(0.02f) // Inverse of 50x gain (INA213 current amplifier)
#define VBUS_RESISTOR_RATIO	(33.36246f) // Inverse of resistor gain (3.09 / 103.09)

//
//#define ADC_IA_PIN		GPIO_PIN_0
//#define ADC_IB_PIN		GPIO_PIN_1
//#define ADC_IC_PIN		GPIO_PIN_6
//#define ADC_VBUS_PIN	GPIO_PIN_7
//#define ADC_TEMP_PIN	GPIO_PIN_0
//#define ADC_THR1_PIN	GPIO_PIN_1
//#define ADC_THR2_PIN	GPIO_PIN_5
//
//#define ADC_I_AND_VBUS_PORT		GPIOA
//#define ADC_TEMP_AND_THR1_PORT	GPIOB
//#define ADC_THR2_PORT			GPIOC
//
//#define ADC_IA_CH		0
//#define ADC_IB_CH		1
//#define ADC_IC_CH		6
//#define ADC_VBUS_CH		7
//#define ADC_TEMP_CH		8
//#define ADC_THR1_CH		9
//#define ADC_THR2_CH		15
//
//#define ADC_I_AND_VBUS_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOA_CLK_ENABLE()
//#define ADC_TEMP_AND_THR1_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOB_CLK_ENABLE()
//#define ADC_THR2_GPIO_CLK_ENABLE()			__HAL_RCC_GPIOC_CLK_ENABLE()

typedef enum
{
	ADC_IA = 0,
	ADC_IB = 1,
	ADC_IC = 2,
	ADC_VBUS = 3,
	ADC_TEMP = 4,
	ADC_THR1 = 5,
	ADC_THR2 = 6
}ADC_OutputTypeDef;

void adcConvComplete(void);
void adcInit(void);
float adcGetCurrent(uint8_t which_cur);
uint16_t adcRawCurrent(uint8_t which_cur);
float adcConvertToAmps(int32_t rawCurrentReading);
float adcGetThrottle(void);
float adcGetVbus(void);
float adcGetVref(void);

#if defined(USING_OLD_ADC_VER)
void ADC_Init_OldVer(void);
uint16_t ADC_GetData_OldVer(ADC_OutputTypeDef nadc);
void ADC_InjectedConversionCompleteCallBack_OldVer(void);
#endif

#endif /* ADC_H_ */
