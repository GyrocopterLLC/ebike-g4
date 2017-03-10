/*
 * adc.c
 *
 *  Created on: Aug 19, 2015
 *      Author: David
 */

#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "project_parameters.h"

uint16_t adc_conv[NUM_ADC_CH];
uint16_t adc_current_null[NUM_CUR_CH];
float adc_vref;

static void adcAverageInitialValue(void);

/**
 * Initializes three ADC units for injected conversion mode.
 * All three ADCs are triggered by TIM1 TRGO (should be set to CCR4)
 * ADC1 converts IA, VBUS, and THR2
 * ADC2 converts IB, THR1, and TEMP
 * ADC3 converts IC
 */
void adcInit(void)
{
	// ADC1: IA(10), Vrefint(17), and Vrefint(17) again
	// ADC2: IB(11), Throttle1(15), and Temperature(9)
	// ADC3: IC(12), Vbus(13), and Throttle2(8)

	GPIO_Clk(ADC_I_VBUS_THR1_PORT);
	GPIO_Clk(ADC_THR2_AND_TEMP_PORT);
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

	GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_IA_PIN);
	GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_IB_PIN);
	GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_IC_PIN);
	GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_VBUS_PIN);
	GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
	GPIO_Analog(ADC_THR2_AND_TEMP_PORT, ADC_TEMP_PIN);
	GPIO_Analog(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN);

	ADC1->CR1 = ADC_CR1_SCAN; // Convert all channels each time a trigger occurs
	ADC2->CR1 = ADC_CR1_SCAN;
	ADC3->CR1 = ADC_CR1_SCAN;

	ADC2->CR2 = 0;
	ADC2->CR2 = 0;
	ADC3->CR2 = 0;

	// Sampling time to 15 cycles
	ADC1->SMPR1 = ADC_SMPR1_SMP10_0 | ADC_SMPR1_SMP13_0;
	ADC1->SMPR2 = ADC_SMPR2_SMP8_0;

	ADC2->SMPR1 = ADC_SMPR1_SMP11_0 | ADC_SMPR1_SMP15_0;
	ADC2->SMPR2 = ADC_SMPR2_SMP9_0;

	ADC3->SMPR1 = ADC_SMPR1_SMP12_0;
	ADC3->SMPR2 = 0;

	// Regular sequence
	ADC1->SQR1 = 0;
	ADC1->SQR2 = 0;
	ADC1->SQR3 = ADC_IA_CH; // Just Phase A current in the first slot

	ADC2->SQR1 = 0;
	ADC2->SQR2 = 0;
	ADC2->SQR3 = ADC_IB_CH;

	ADC3->SQR1 = 0;
	ADC3->SQR2 = 0;
	ADC3->SQR3 = ADC_IC_CH;

	// Injected sequence
	ADC1->JSQR = ADC_JSQR_JL_1 | (ADC_IA_CH << 5) | (VREFINT_CH  << 10) | (VREFINT_CH  << 15);
	ADC2->JSQR = ADC_JSQR_JL_1 | (ADC_IB_CH << 5) | (ADC_THR1_CH << 10) | (ADC_TEMP_CH << 15);
	ADC3->JSQR = ADC_JSQR_JL_1 | (ADC_IC_CH << 5) | (ADC_VBUS_CH << 10) | (ADC_THR2_CH << 15);

	// ADC master controls
	// PCLK divided by 4 (21MHz)
	// Simultaneous regular and injected mode
	//ADC->CCR = ADC_CCR_ADCPRE_0 | ADC_CCR_MULTI_0 | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_4;
	ADC->CCR = ADC_CCR_ADCPRE_0 | ADC_CCR_MULTI_0 | ADC_CCR_MULTI_4;
	// Turn on the Vrefint and Temp sensor channels
	ADC->CCR |= ADC_CCR_TSVREFE;

	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	adcAverageInitialValue();

	ADC1->CR2 |= ADC_CR2_JEXTEN_1 | ADC_CR2_JEXTSEL_0; // Rising edge, TIM1 TRGO

	ADC1->SR = 0;
	ADC1->CR1 |= ADC_CR1_JEOCIE;

	NVIC_SetPriority(ADC_IRQn,PRIO_ADC);
	NVIC_EnableIRQ(ADC_IRQn);

#if defined(USING_OLD_VER)


	ADC_InjectionConfTypeDef iconf;
	ADC_MultiModeTypeDef mm;
	GPIO_InitTypeDef gi;

	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();
	__HAL_RCC_ADC3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	gi.Pin = ADC_IA_PIN | ADC_IB_PIN | ADC_IC_PIN | ADC_VBUS_PIN | ADC_THR1_PIN;
	gi.Mode = GPIO_MODE_ANALOG;
	gi.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_I_VBUS_THR1_PORT, &gi);
	gi.Pin = ADC_THR2_PIN | ADC_TEMP_PIN;
	HAL_GPIO_Init(ADC_THR2_AND_TEMP_PORT, &gi);

	adc1.Instance = ADC1;
	adc2.Instance = ADC2;
	adc3.Instance = ADC3;

	adc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	adc1.Init.Resolution = ADC_RESOLUTION_12B;
	adc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	adc1.Init.ScanConvMode = ENABLE;
	adc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	adc1.Init.ContinuousConvMode = DISABLE;
	adc1.Init.DMAContinuousRequests = DISABLE;
	adc1.Init.NbrOfConversion = 1;
	adc1.Init.DiscontinuousConvMode = DISABLE;
	adc1.Init.NbrOfDiscConversion = 1;
	adc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	adc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;

	adc2.Init = adc1.Init;
	adc3.Init = adc1.Init;

	HAL_ADC_Init(&adc1);
	HAL_ADC_Init(&adc2);
	HAL_ADC_Init(&adc3);
	iconf.InjectedChannel = ADC_IA_CH;
	iconf.InjectedRank = 1;
	iconf.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	iconf.InjectedOffset = 0;
	iconf.InjectedNbrOfConversion = 3;
	iconf.AutoInjectedConv = DISABLE;
	iconf.InjectedDiscontinuousConvMode = DISABLE;
	iconf.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
	iconf.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
	iconf.AutoInjectedConv = DISABLE;
	HAL_ADCEx_InjectedConfigChannel(&adc1,&iconf);
	iconf.InjectedChannel = ADC_VBUS_CH;
	iconf.InjectedRank = 2;
	HAL_ADCEx_InjectedConfigChannel(&adc1,&iconf);
	iconf.InjectedChannel = ADC_THR2_CH;
	iconf.InjectedRank = 3;
	HAL_ADCEx_InjectedConfigChannel(&adc1,&iconf);

	iconf.InjectedChannel = ADC_IB_CH;
	iconf.InjectedRank = 1;
	HAL_ADCEx_InjectedConfigChannel(&adc2,&iconf);
	iconf.InjectedChannel = ADC_THR1_CH;
	iconf.InjectedRank = 2;
	HAL_ADCEx_InjectedConfigChannel(&adc2,&iconf);
	iconf.InjectedChannel = ADC_TEMP_CH;
	iconf.InjectedRank = 3;
	HAL_ADCEx_InjectedConfigChannel(&adc2,&iconf);

	iconf.InjectedChannel = ADC_IC_CH;
	iconf.InjectedRank = 1;
	iconf.InjectedNbrOfConversion = 1;
	HAL_ADCEx_InjectedConfigChannel(&adc3,&iconf);

	mm.Mode = ADC_TRIPLEMODE_INJECSIMULT;
	mm.DMAAccessMode = ADC_DMAACCESSMODE_DISABLED;
	mm.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
	HAL_ADCEx_MultiModeConfigChannel(&adc1, &mm);

	HAL_NVIC_SetPriority(ADC_IRQn,2,0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	HAL_ADCEx_InjectedStart(&adc2);
	HAL_ADCEx_InjectedStart(&adc3);
	HAL_ADCEx_InjectedStart_IT(&adc1);
#endif
}

void adcConvComplete(void)
{
	adc_conv[ADC_IA] = ADC1->JDR1;
	adc_conv[ADC_IB] = ADC2->JDR1;
	adc_conv[ADC_IC] = ADC3->JDR1;
	adc_conv[ADC_VBUS] = ADC3->JDR2;
	adc_conv[ADC_THR1] = ADC2->JDR2;
	adc_conv[ADC_THR2] = ADC3->JDR3;
	adc_conv[ADC_TEMP] = ADC2->JDR3;
	adc_conv[ADC_VREFINT] = (ADC1->JDR2 + ADC1->JDR3) >> 1;
}

static void adcAverageInitialValue(void)
{
	uint32_t ia_sum = 0;
	uint32_t ib_sum = 0;
	uint32_t ic_sum = 0;

	float vrefint = 0.0f;
	uint16_t* vrefcal = ((uint16_t*)0x1FFF7A2A); // From STM32F4 datasheet

	if((*vrefcal >= VREFINTCAL_MIN) && (*vrefcal <= VREFINTCAL_MAX)) // Between 1.13V and 1.29V (should be around 1.21V)
	{
		vrefint = (3.30f)*((float)(*vrefcal))/MAXCOUNTF;
	}
	else // Default to the regular value of 1.21V
	{
		vrefint = VREFINTDEFAULT;
	}

	Delay(50); // Wait for all analog voltages to stablize

	// Loop and average
	for(uint8_t i = 0; i < 128; i++)
	{
		// Trigger ADC for Ia, Ib, Ic
		ADC1->CR2 |= ADC_CR2_SWSTART;

		// Wait until all conversions are complete
		while((ADC1->SR & ADC_SR_EOC) == 0);

		ADC1->SR &= ~(ADC_SR_EOC);
		ia_sum += ADC1->DR;
		ib_sum += ADC2->DR;
		ic_sum += ADC3->DR;
	}
	adc_current_null[ADC_IA] = ia_sum / 128 - ADC_HACKY_OFFSET;
	adc_current_null[ADC_IB] = ib_sum / 128 - ADC_HACKY_OFFSET;
	adc_current_null[ADC_IC] = ic_sum / 128 - ADC_HACKY_OFFSET;

	// Switch ADC1 to read the Vrefint channel
	ADC1->SQR3 = VREFINT_CH;
	ia_sum = 0;
	for(uint8_t i = 0; i < 128; i++)
	{
		ADC1->CR2 |= ADC_CR2_SWSTART;
		while((ADC1->SR & ADC_SR_EOC) == 0);
		ADC1->SR &= ~(ADC_SR_EOC);
		ia_sum += ADC1->DR;
	}

	adc_vref = vrefint / ((float)(ia_sum/128) / MAXCOUNTF);
	// Switch it back to Ia channel
	ADC1->SQR3 = ADC_IA_CH;
}

float adcConvertToAmps(int32_t rawCurrentReading)
{
	// Assume null point has already been subtracted.
	// The raw reading is a 12-bit number
	//float temp_current = ((float)rawCurrentReading)*0.000244140625f; // inverse of 4096
	float temp_current = ((float)rawCurrentReading)/MAXCOUNTF;
	// Convert to volts
	temp_current *= adc_vref;
	// Convert to amps
	temp_current *= RSHUNT_INV * INAGAIN_INV;

	return temp_current;
}

float adcGetCurrent(uint8_t which_cur)
{
	return adcConvertToAmps((int32_t)(adc_conv[which_cur]) - (int32_t)(adc_current_null[which_cur]));
}

uint16_t adcRaw(uint8_t which_cur)
{
	return adc_conv[which_cur];
}

float adcGetThrottle(void)
{
	// Convert 12-bit adc result to floating point
	float temp_throttle = ((float)adc_conv[ADC_THR1])/MAXCOUNTF;
	// Convert to volts using reference measurement
	temp_throttle *= adc_vref;
	return temp_throttle;
}

float adcGetVbus(void)
{
	// Convert 12-bit to float
	float temp_vbus = ((float)adc_conv[ADC_VBUS])/MAXCOUNTF;
	// Convert to volts from reference
	temp_vbus *= adc_vref;
	// Convert to real measurement from resistor divider ratio
	temp_vbus *= VBUS_RESISTOR_RATIO;
	return temp_vbus;
}

float adcGetVref(void)
{
	return adc_vref;
}

void adcSetNull(uint8_t which_cur, uint16_t nullVal)
{
	adc_current_null[which_cur] = nullVal;
}

#if defined(USING_OLD_ADC_VER)

/**
 * Initializes the hardware resources for the ADC module(s)
 * PA0 -> ADC123_IN0 (IA)
 * PA1 -> ADC123_IN1 (IB)
 * PA6 -> ADC12_IN6 (IC)
 * PA7 -> ADC12_IN7 (Vbus)
 * PB0 -> ADC12_IN8 (Temp Sensor)
 * PB1 -> ADC12_IN9 (Throttle1)
 * PC5 -> ADC12_IN15 (Throttle2)
 *
 * Enables the three ADC units for triple simultaneous injected mode.
 * Conversions are triggered by TIM1 TRGO (which should be connected to CC4)
 *
 * ADC1 converts IC, Vbus, and Throttle1
 * ADC2 converts IB, Temp sensor, and Throttle2
 * ADC3 converts IA
 *
 * ADC1 flags the injected complete interrupt after conversions are done
 */
void ADC_Init_OldVer(void)
{
	GPIO_InitTypeDef gpioInit;
	// Start by enabling the GPIOs
	ADC_I_VBUS_THR1_CLK_ENABLE();
	ADC_THR2_AND_TEMP_CLK_ENABLE();

	gpioInit.Pin = ADC_IA_PIN | ADC_IB_PIN | ADC_IC_PIN | ADC_VBUS_PIN | ADC_THR1_PIN;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_I_VBUS_THR1_PORT, &gpioInit);

	gpioInit.Pin = ADC_TEMP_PIN | ADC_THR2_PIN;
	HAL_GPIO_Init(ADC_THR2_AND_TEMP_PORT, &gpioInit);

	// Clock the ADCs
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC2_CLK_ENABLE();
	__HAL_RCC_ADC3_CLK_ENABLE();

	// CR1 settings
	ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_JEOCIE;
	ADC2->CR1 = ADC_CR1_SCAN;
	ADC3->CR1 = ADC_CR1_SCAN;

	// CR2 settings
	ADC1->CR2 = ADC_CR2_JEXTEN_0 | ADC_CR2_JEXTSEL_0; // Rising edge on TIM1_TRGO
	ADC2->CR2 = 0;
	ADC3->CR2 = 0;

	// JSQR settings
	ADC1->JSQR = (ADC_IC_CH<<5) + (ADC_VBUS_CH<<10) + (ADC_THR1_CH<<15) + ADC_JSQR_JL_1;
	ADC2->JSQR = (ADC_IB_CH<<5) + (ADC_TEMP_CH<<10) + (ADC_THR2_CH<<15) + ADC_JSQR_JL_1;
	ADC3->JSQR = (ADC_IA_CH<<15);

	// Common settings
	ADC->CCR = ADC_CCR_MULTI_4 | ADC_CCR_MULTI_2 | ADC_CCR_MULTI_0; // Triple injected only
	ADC->CCR |= ADC_CCR_ADCPRE_0; // Prescaler DivideBy4 (84MHz / 4 = 21MHz)
	//ADC->CCR |= ADC_CCR_TSVREFE; // Temp sensor and voltage reference on

	// Enable the interrupt
	HAL_NVIC_SetPriority(ADC_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	// Turn on the ADCs
	ADC1->CR2 |= ADC_CR2_ADON;
	ADC2->CR2 |= ADC_CR2_ADON;
	ADC3->CR2 |= ADC_CR2_ADON;

	/*
	hadc1.Instance = ADC1;
	hadc2.Instance = ADC2;
	hadc3.Instance = ADC3;

	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; // 84MHz / 4 = 21MHz
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 2;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	HAL_ADC_Init(&hadc1);

	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; // 84MHz / 4 = 21MHz
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.ScanConvMode = ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.NbrOfConversion = 2;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.NbrOfDiscConversion = 2;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	HAL_ADC_Init(&hadc2);

	hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4; // 84MHz / 4 = 21MHz
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.ScanConvMode = ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.NbrOfDiscConversion = 1;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	HAL_ADC_Init(&hadc3);
	*/

	// Doing this manually.

}

uint16_t ADC_GetData_OldVer(ADC_OutputTypeDef nadc)
{
	if(nadc < NUM_ADC_CH)
		return adc_conv[nadc];
	return 0;
}

void ADC_InjectedConversionCompleteCallBack_OldVer(void)
{
	// Move data from injected data registers to local variable storage

	// First conversion results
	adc_conv[ADC_IA] = ADC3->JDR1;
	adc_conv[ADC_IB] = ADC2->JDR1;
	adc_conv[ADC_IC] = ADC1->JDR1;

	// Second conversion results
	adc_conv[ADC_VBUS] = ADC1->JDR2;
	adc_conv[ADC_TEMP] = ADC2->JDR2;

	// Third conversion results
	adc_conv[ADC_THR1] = ADC1->JDR3;
	adc_conv[ADC_THR2] = ADC2->JDR3;
}
#endif
