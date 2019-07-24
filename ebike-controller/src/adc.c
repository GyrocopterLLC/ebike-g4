/******************************************************************************
 * Filename: adc.c
 * Description: Performs analog-to-digital converter initialization and data
 *              capture.
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

#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "project_parameters.h"
#include <math.h>

uint16_t adc_conv[NUM_ADC_CH];
uint16_t adc_current_null[NUM_CUR_CH];
float adc_vref;

Config_ADC config_adc;

static void adcAverageInitialValue(void);

/**
 * Initializes three ADC units for injected conversion mode.
 * All three ADCs are triggered by TIM1 TRGO (should be set to CCR4)
 * ADC1 converts IA, VBUS, and THR2
 * ADC2 converts IB, THR1, and TEMP
 * ADC3 converts IC
 */
void adcInit(void) {
    // ADC1: IA(10), Vrefint(17), and Vrefint(17) again
    // ADC2: IB(11), Throttle1(15), and Temperature(9)
    // ADC3: IC(12), Vbus(13), and Throttle2(8)

    // Load from eeprom
    config_adc.Inverse_TIA_Gain = EE_ReadFloatWithDefault(CONFIG_ADC_INV_TIA_GAIN, DFLT_ADC_INV_TIA_GAIN);
    config_adc.Vbus_Ratio = EE_ReadFloatWithDefault(CONFIG_ADC_VBUS_RATIO, DFLT_ADC_VBUS_RATIO);
    config_adc.Thermistor_Fixed_R = EE_ReadFloatWithDefault(CONFIG_ADC_THERM_FIXED_R, DFLT_ADC_THERM_FIXED_R);
    config_adc.Thermistor_R25 = EE_ReadFloatWithDefault(CONFIG_ADC_THERM_R25, DFLT_ADC_THERM_R25);
    config_adc.Thermistor_Beta = EE_ReadFloatWithDefault(CONFIG_ADC_THERM_B, DFLT_ADC_THERM_B);
    // For convenience
    config_adc.Inverse_Therm_Beta = 1.0f / config_adc.Thermistor_Beta;

    GPIO_Clk(ADC_I_VBUS_THR1_PORT);
    GPIO_Clk(ADC_THR2_AND_TEMP_PORT);
    RCC->APB2ENR |=
            RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN | RCC_APB2ENR_ADC3EN;

    GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_IA_PIN);
    GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_IB_PIN);
    GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_IC_PIN);
    GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_VBUS_PIN);
    // GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN); // Done in throttle.c
    GPIO_Analog(ADC_THR2_AND_TEMP_PORT, ADC_TEMP_PIN);
    // GPIO_Analog(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN); // Done in throttle.c

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
    ADC1->JSQR = ADC_JSQR_JL_1 | (ADC_IA_CH << 5) | (VREFINT_CH << 10)
            | (VREFINT_CH << 15);
    ADC2->JSQR = ADC_JSQR_JL_1 | (ADC_IB_CH << 5) | (ADC_THR1_CH << 10)
            | (ADC_TEMP_CH << 15);
    ADC3->JSQR = ADC_JSQR_JL_1 | (ADC_IC_CH << 5) | (ADC_VBUS_CH << 10)
            | (ADC_THR2_CH << 15);

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

    NVIC_SetPriority(ADC_IRQn, PRIO_ADC);
    NVIC_EnableIRQ(ADC_IRQn);
}

void adcConvComplete(void) {
    adc_conv[ADC_IA] = ADC1->JDR1;
    adc_conv[ADC_IB] = ADC2->JDR1;
    adc_conv[ADC_IC] = ADC3->JDR1;
    adc_conv[ADC_VBUS] = ADC3->JDR2;
    adc_conv[ADC_THR1] = ADC2->JDR2;
    adc_conv[ADC_THR2] = ADC3->JDR3;
    adc_conv[ADC_TEMP] = ADC2->JDR3;
    adc_conv[ADC_VREFINT] = (ADC1->JDR2 + ADC1->JDR3) >> 1;
}

static void adcAverageInitialValue(void) {
    uint32_t ia_sum = 0;
    uint32_t ib_sum = 0;
    uint32_t ic_sum = 0;

    float vrefint = 0.0f;
    uint16_t* vrefcal = ((uint16_t*) 0x1FFF7A2A); // From STM32F4 datasheet

    if ((*vrefcal >= VREFINTCAL_MIN) && (*vrefcal <= VREFINTCAL_MAX)) // Between 1.13V and 1.29V (should be around 1.21V)
            {
        vrefint = (ADC_FACTORY_CAL_VOLTAGE) * ((float) (*vrefcal)) / MAXCOUNTF;
    } else // Default to the regular value of 1.21V
    {
        vrefint = VREFINTDEFAULT;
    }

    Delay(ADC_STARTUP_DELAY_MS); // Wait for all analog voltages to stablize

    // Loop and average
    for (uint8_t i = 0; i < ADC_NUM_NULLING_SAMPLES; i++) {
        // Trigger ADC for Ia, Ib, Ic
        ADC1->CR2 |= ADC_CR2_SWSTART;

        // Wait until all conversions are complete
        while ((ADC1->SR & ADC_SR_EOC) == 0)
            ;

        ADC1->SR &= ~(ADC_SR_EOC);
        ia_sum += ADC1->DR;
        ib_sum += ADC2->DR;
        ic_sum += ADC3->DR;
    }
    adc_current_null[ADC_IA] = ia_sum / ADC_NUM_NULLING_SAMPLES;
    adc_current_null[ADC_IB] = ib_sum / ADC_NUM_NULLING_SAMPLES;
    adc_current_null[ADC_IC] = ic_sum / ADC_NUM_NULLING_SAMPLES;

    // Switch ADC1 to read the Vrefint channel
    ADC1->SQR3 = VREFINT_CH;
    ia_sum = 0;
    for (uint8_t i = 0; i < ADC_NUM_NULLING_SAMPLES; i++) {
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while ((ADC1->SR & ADC_SR_EOC) == 0)
            ;
        ADC1->SR &= ~(ADC_SR_EOC);
        ia_sum += ADC1->DR;
    }

    adc_vref = vrefint
            / ((float) (ia_sum / ADC_NUM_NULLING_SAMPLES) / MAXCOUNTF);
    // Switch it back to Ia channel
    ADC1->SQR3 = ADC_IA_CH;
}

float adcConvertToAmps(int32_t rawCurrentReading) {
    // Assume null point has already been subtracted.
    // The raw reading is a 12-bit number
    //float temp_current = ((float)rawCurrentReading)*0.000244140625f; // inverse of 4096
    float temp_current = ((float) rawCurrentReading) / MAXCOUNTF;
    // Convert to volts
    temp_current *= adc_vref;
    // Convert to amps
//    temp_current *= CURRENT_AMP_INV;
    temp_current *= config_adc.Inverse_TIA_Gain;

    return temp_current;
}

float adcGetCurrent(uint8_t which_cur) {
    return adcConvertToAmps(
            (int32_t) (adc_conv[which_cur])
                    - (int32_t) (adc_current_null[which_cur]));
}

uint16_t adcRaw(uint8_t which_cur) {
    return adc_conv[which_cur];
}

float adcGetThrottle(uint8_t thrnum) {
    float temp_throttle = 0.0f;
    if (thrnum == 1) {
        // Convert 12-bit adc result to floating point
        temp_throttle = ((float) adc_conv[ADC_THR1]) / MAXCOUNTF;
    } else if (thrnum == 2) {
        temp_throttle = ((float) adc_conv[ADC_THR2]) / MAXCOUNTF;
    }
    // Convert to volts using reference measurement
    temp_throttle *= adc_vref;
    return temp_throttle;
}

float adcGetVbus(void) {
    // Convert 12-bit to float
    float temp_vbus = ((float) adc_conv[ADC_VBUS]) / MAXCOUNTF;
    // Convert to volts from reference
    temp_vbus *= adc_vref;
    // Convert to real measurement from resistor divider ratio
//    temp_vbus *= VBUS_RESISTOR_RATIO;
    temp_vbus *= config_adc.Vbus_Ratio;
    return temp_vbus;
}

float adcGetVref(void) {
    return adc_vref;
}

void adcSetNull(uint8_t which_cur, uint16_t nullVal) {
    adc_current_null[which_cur] = nullVal;
}

float adcGetTempDegC(void) {
    
    // Step 1: Calculate thermistor resistance right now
    // Fixed resistor is at the bottom of the voltage divider,
    // thermistor is on top.
    // (Vout/Vin) = Rf / (Rt + Rf)
    // ADC / 4095 = Vout/Vin = Rf / (Rt + Rf)
    // let's call ADC/4095 = "adc"
    // adc = Rf/(Rt+Rf)
    // Rt*adc + Rf*adc = Rf
    // Rt*adc = Rf - Rf*adc
    // Rt = Rf*(1-adc)/adc, which simplifies to Rf*(1/adc - 1)
    
    // Convert 12-bit to float
    float temp = ((float) adc_conv[ADC_TEMP]) / MAXCOUNTF;
    // Calculate resistance
//    temp = TEMP_FIXED_RESISTOR * (1.0f/temp - 1.0f);
    temp = config_adc.Thermistor_Fixed_R * (1.0f/temp - 1.0f);
    // Step 2: Convert to Kelvins using thermistor equation
    // beta = log(Rt1/Rt2) / (1/T1 - 1/T2)
    // 1/T1 - 1/T2 = log(Rt1 / Rt2) / beta
    // 1/T1 - log(Rt1 / Rt2)/beta = 1/T2
    // T2 = 1/(1/T1 - log(Rt1 / Rt2)/beta
    // Where T1 = 25degC = 298.15K, Rt1 = THERM_R25, and beta = THERM_B_VALUE
//    temp = (1.0f / 298.15f) - logf(THERM_R25 / temp) / THERM_B_VALUE;
    temp = (1.0f / 298.15f) - logf(config_adc.Thermistor_R25 / temp) * config_adc.Inverse_Therm_Beta;
    temp = 1.0f / temp;
    temp -= 273.15f; // Convert from K to degC

    return temp;
}

uint8_t adcSetInverseTIAGain(float new_gain) {
    config_adc.Inverse_TIA_Gain = new_gain;
    return DATA_PACKET_SUCCESS;
}

float adcGetInverseTIAGain(void) {
    return config_adc.Inverse_TIA_Gain;
}

uint8_t adcSetVbusRatio(float new_ratio) {
    config_adc.Vbus_Ratio = new_ratio;
    return DATA_PACKET_SUCCESS;
}

float adcGetVbusRatio(void) {
    return config_adc.Vbus_Ratio;
}

uint8_t adcSetThermFixedR(float new_fixed_r) {
    config_adc.Thermistor_Fixed_R = new_fixed_r;
    return DATA_PACKET_SUCCESS;
}

float adcGetThermFixedR(void) {
    return config_adc.Thermistor_Fixed_R;
}

uint8_t adcSetThermR25(float new_r25) {
    config_adc.Thermistor_R25 = new_r25;
    return DATA_PACKET_SUCCESS;
}

float adcGetThermR25(void){
    return config_adc.Thermistor_R25;
}

uint8_t adcSetThermBeta(float new_beta) {
    config_adc.Thermistor_Beta = new_beta;
    config_adc.Inverse_Therm_Beta = 1.0f / new_beta;
    return DATA_PACKET_SUCCESS;
}

float adcGetThermBeta(void) {
    return config_adc.Thermistor_Beta;
}
