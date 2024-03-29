/******************************************************************************
 * Filename: adc.c
 * Description: Performs analog-to-digital converter initialization and data
 *              capture.
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

#include "main.h"
#include <math.h>

uint16_t adc1_raw_regular_results[8];
uint16_t adc2_raw_regular_results[8];
uint16_t adc3_raw_regular_results[8];
uint16_t adc4_raw_regular_results[8];

uint16_t adc_conv[NUM_ADC_CH];
uint16_t adc_current_null[NUM_CUR_CH];
float adc_vref;

// Filters
Biquad_Type adc_fet_filter;

Config_ADC config_adc;

static void ADC_Enable(ADC_TypeDef* adc);
static void ADC_CalcVref(void);
/**
 * @brief Initialize the ADC peripherals
 *
 * Initializes two or four ADC units for regular and injected conversion modes.
 * All ADCs are triggered by TIM1 TRGO (should be set to CCR4)
 * In two channel mode:
 * ADC1 converts IA, IC, VC, VBUS, and Motor Temp
 * ADC2 converts IB, VA, VB, Throttle, and FET Temp
 * In four channel mode:
 * ADC1 converts IC and Motor Temp
 * ADC2 converts IB, VA, VB, Throttle, and FET Temp
 * ADC3 converts IA and VC
 * ADC4 converts VBUS
 *
 * Currents are converted in injected mode. All others are done
 * in regular sequence.
 *
 * According to the errata, we should always perform two conversions
 * and throw out the second one due to instability when switching
 * inputs. The sampling switch changes to the next input in the
 * middle of a conversion, but if the next conversion is the same
 * as the current conversion, no switching will take place. Therefore,
 * we should use the first conversion of two of the same channel as the
 * best conversion.
 *
 * ADCs are all triggered together, which should help solve some of the
 * sampling switching issues. All the sampling times should be the same
 * for either injected or regular mode.
 */
void ADC_Init(void) {

    // Load from eeprom
    ADC_LoadVariables();

    // Enable all the GPIO clocks
    GPIO_Clk(ADC_MTEMP_PORT); GPIO_Clk(ADC_FTEMP_PORT);
    GPIO_Clk(ADC_IA_PORT); GPIO_Clk(ADC_IB_PORT); GPIO_Clk(ADC_IC_PORT);
    GPIO_Clk(ADC_VA_PORT); GPIO_Clk(ADC_VB_PORT); GPIO_Clk(ADC_VC_PORT);
    GPIO_Clk(ADC_THR_PORT); GPIO_Clk(ADC_VBUS_PORT);
    // Change GPIOs to analog input
    GPIO_Analog(ADC_MTEMP_PORT, ADC_MTEMP_PIN);
    GPIO_Analog(ADC_FTEMP_PORT, ADC_FTEMP_PIN);
    GPIO_Analog(ADC_IA_PORT, ADC_IA_PIN);
    GPIO_Analog(ADC_IB_PORT, ADC_IB_PIN);
    GPIO_Analog(ADC_IC_PORT, ADC_IC_PIN);
    GPIO_Analog(ADC_VA_PORT, ADC_VA_PIN);
    GPIO_Analog(ADC_VB_PORT, ADC_VB_PIN);
    GPIO_Analog(ADC_VC_PORT, ADC_VC_PIN);
    GPIO_Analog(ADC_THR_PORT, ADC_THR_PIN);
    GPIO_Analog(ADC_VBUS_PORT, ADC_VBUS_PIN);

#if defined(USE_ADC_1_2_ONLY)

#else


#endif
    // Enable the ADC clocks
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN | RCC_AHB2ENR_ADC345EN;
    // Disable the deep power down
    ADC1->CR &= (~ADC_CR_DEEPPWD);
    ADC2->CR &= (~ADC_CR_DEEPPWD);
    ADC3->CR &= (~ADC_CR_DEEPPWD);
    ADC4->CR &= (~ADC_CR_DEEPPWD);

    // Turn on Vrefint, also this register controls
    // clocking. ADC is not prescaled, input clock is
    // the "kernel clock" set to PLL_P.
    ADC12_COMMON->CCR = ADC_CCR_VREFEN;
    ADC345_COMMON->CCR = ADC_CCR_VREFEN;

    // Turn on internal voltage regulators
    ADC1->CR |= ADC_CR_ADVREGEN;
    ADC2->CR |= ADC_CR_ADVREGEN;
    ADC3->CR |= ADC_CR_ADVREGEN;
    ADC4->CR |= ADC_CR_ADVREGEN;
    // Wait for it to stabilize
    Delay(ADC_VREG_STARTUP_DELAY);

    // Perform calibrations
    ADC1->CR |= ADC_CR_ADCAL;
    while((ADC1->CR & ADC_CR_ADCAL) != 0) {
        // pass
    }
    ADC2->CR |= ADC_CR_ADCAL;
    while((ADC2->CR & ADC_CR_ADCAL) != 0) {
        // pass
    }
    ADC3->CR |= ADC_CR_ADCAL;
    while((ADC3->CR & ADC_CR_ADCAL) != 0) {
        // pass
    }
    ADC4->CR |= ADC_CR_ADCAL;
    while((ADC4->CR & ADC_CR_ADCAL) != 0) {
        // pass
    }

    // Measure and store VREF by converting Vrefint
    ADC_CalcVref();

    // Initialize filters
    FOC_BiquadLPF(&adc_fet_filter, ((float)DFLT_FOC_PWM_FREQ), 20.0f, 0.707f);

    // CFGR settings:   JQDIS = 1 (queue is disabled for injected conversions)
    //                  DMAEN = 1 (DMA is enabled to generate DMA requests)
    //                  DMACFG = 1 (DMA in circular mode)
    ADC1->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC2->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC3->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC4->CFGR = ADC_CFGR_JQDIS | ADC_CFGR_DMAEN | ADC_CFGR_DMACFG;
    ADC1->CFGR2 = 0;
    ADC2->CFGR2 = 0;
    ADC3->CFGR2 = 0;
    ADC4->CFGR2 = 0;

    // Sampling time is 2.5 cycles for currents (42.5MHz / 2.5 -> ~59ns)
    // 47.5 cycles (~576ns) for all others
    // MTEMP gets 47.5 cycles
    ADC1->SMPR2 = (0x03) << ((ADC_MTEMP_CH-10U) * 3U);
    // FTEMP, VA, and THR get 47.5 cycles
    ADC2->SMPR1 = (0x03) << (ADC_VB_CH * 3U);
    ADC2->SMPR2 = (0x03) << ((ADC_FTEMP_CH-10U) * 3U)
                  | (0x03) << ((ADC_THR_CH-10U) * 3U);
    // VC gets 47.5 cycles
    ADC3->SMPR1 = (0x03) << (ADC_VC_CH * 3U);
    // VBUS gets 47.5 cycles
    ADC4->SMPR1 = (0x03) << (ADC_VBUS_CH * 3U);

    // Regular sequence - 8 conversions for everybody since ADC2 has 4 channels
    // ADC1 converts motor temp 8 times
    ADC1->SQR1 = 7U | (ADC_MTEMP_CH << 6U) | (ADC_MTEMP_CH << 12U)
            | (ADC_MTEMP_CH << 18U) | (ADC_MTEMP_CH << 24U);
    ADC1->SQR2 = (ADC_MTEMP_CH) | (ADC_MTEMP_CH << 6U)
                | (ADC_MTEMP_CH << 12U) | (ADC_MTEMP_CH << 18U);

    // ADC2 converts VA, VB, throttle, and FET temp 2 times each
    ADC2->SQR1 = 7U | (ADC_VA_CH << 6U) | (ADC_VA_CH << 12U)
            | (ADC_VB_CH << 18U) | (ADC_VB_CH << 24U);
    ADC2->SQR2 = (ADC_THR_CH) | (ADC_THR_CH << 6U)
                | (ADC_FTEMP_CH << 12U) | (ADC_FTEMP_CH << 18U);

    // ADC3 converts VC 8 times
    ADC3->SQR1 = 7U | (ADC_VC_CH << 6U) | (ADC_VC_CH << 12U)
            | (ADC_VC_CH << 18U) | (ADC_VC_CH << 24U);
    ADC3->SQR2 = (ADC_VC_CH) | (ADC_VC_CH << 6U)
                | (ADC_VC_CH << 12U) | (ADC_VC_CH << 18U);

    // and ADC4 converts VBUS 8 times
    ADC4->SQR1 = 7U | (ADC_VBUS_CH << 6U) | (ADC_VBUS_CH << 12U)
            | (ADC_VBUS_CH << 18U) | (ADC_VBUS_CH << 24U);
    ADC4->SQR2 = (ADC_VBUS_CH) | (ADC_VBUS_CH << 6U)
                | (ADC_VBUS_CH << 12U) | (ADC_VBUS_CH << 18U);

    // Injected sequences
    // ADC1, 2, and 3 convert the current sensor 2 times each
    // ADC4 doesn't have an injected sequence
    // External trigger is TIM1_TRGO, rising edge (set to oc4ref in TIM1)
    ADC1->JSQR = 1 | ADC_JSQR_JEXTEN_0 | (ADC_IC_CH << 9U) | (ADC_IC_CH << 15U);
    ADC2->JSQR = 1 | ADC_JSQR_JEXTEN_0 | (ADC_IB_CH << 9U) | (ADC_IB_CH << 15U);
    ADC3->JSQR = 1 | ADC_JSQR_JEXTEN_0 | (ADC_IA_CH << 9U) | (ADC_IA_CH << 15U);

    // External trigger selection (regular sequence): hardware trigger on rising edge
    // of TIM1_TRGO2 (set to oc5ref in TIM1)
    // Also ignore overruns due to weird DMA crap.
    ADC1->CFGR |= ADC_CFGR_OVRMOD | (10U << 5U) | (1U << 10U); // EXTSEL = 01010, EXTEN = 01
    ADC2->CFGR |= ADC_CFGR_OVRMOD | (10U << 5U) | (1U << 10U); // EXTSEL = 01010, EXTEN = 01
    ADC3->CFGR |= ADC_CFGR_OVRMOD | (10U << 5U) | (1U << 10U); // EXTSEL = 01010, EXTEN = 01
    ADC4->CFGR |= ADC_CFGR_OVRMOD | (10U << 5U) | (1U << 10U); // EXTSEL = 01010, EXTEN = 01
    /*
     * Dual ADC mode settings
     * ADC1&2 is set in injected simultaneous + regular simultaneous modes. That means that the
     * triggers are really only read by ADC1. When the injected channels are finished, the master
     * JEOS interrupt will read both sets of JDR data registers. Since the conversions are the
     * same length, they will be finished at the same time. For regular channels, DMA is enabled
     * individually. The DMA request is triggered by each ADC when its EOC flag is set. They will
     * probably be set around the same time as each other since they were triggered together.
     * The DMA is configured to pull from the data register after each conversion is complete
     * and save the result to an array in memory.
     *
     * ADC3&4 are set in regular simultaneous only. The injected channels on ADC3 are run solo,
     * since ADC4 doesn't have any injected channels assigned. The regular sequence for ADC3&4
     * works the same way as ADC1&2, including individual DMA channels for saving data to memory.
     */
    // Enable DMA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN;
    // Configure settings on DMA channels, except for enabling the channel. That's later.
    //  --- 16-bit transfers (both MSIZE and PSIZE), memory increment mode, but not peripheral increment
    //  --- and circular mode. Direction is read from peripheral (bit is zero)
    ADC1_DMACHANNEL->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
    ADC2_DMACHANNEL->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
    ADC3_DMACHANNEL->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
    ADC4_DMACHANNEL->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
    ADC1_DMACHANNEL->CNDTR = 8; // each regular sequence is 8 conversions
    ADC2_DMACHANNEL->CNDTR = 8;
    ADC3_DMACHANNEL->CNDTR = 8;
    ADC4_DMACHANNEL->CNDTR = 8;
    ADC1_DMACHANNEL->CMAR = (uint32_t)(&adc1_raw_regular_results);
    ADC2_DMACHANNEL->CMAR = (uint32_t)(&adc2_raw_regular_results);
    ADC3_DMACHANNEL->CMAR = (uint32_t)(&adc3_raw_regular_results);
    ADC4_DMACHANNEL->CMAR = (uint32_t)(&adc4_raw_regular_results);
    ADC1_DMACHANNEL->CPAR = (uint32_t)(&(ADC1->DR));
    ADC2_DMACHANNEL->CPAR = (uint32_t)(&(ADC2->DR));
    ADC3_DMACHANNEL->CPAR = (uint32_t)(&(ADC3->DR));
    ADC4_DMACHANNEL->CPAR = (uint32_t)(&(ADC4->DR));
    // Configure settings on DMAMUX. Just set the channel selection for channel 1 to ADC1, channel 2 to ADC3
    // None of the other special features (like synchronization) are needed
    ADC1_DMAMUXCHANNEL->CCR = ADC1_DMAMUX_REQ;
    ADC2_DMAMUXCHANNEL->CCR = ADC2_DMAMUX_REQ;
    ADC3_DMAMUXCHANNEL->CCR = ADC3_DMAMUX_REQ;
    ADC4_DMAMUXCHANNEL->CCR = ADC4_DMAMUX_REQ;
    // Setup the ADC common register to enable dual mode.
    ADC12_COMMON->CCR |= ADC_CCR_DUAL_0;
    ADC345_COMMON->CCR |= ADC_CCR_DUAL_2 | ADC_CCR_DUAL_1;

    // Apply current sensing offsets default value
    // We don't know what the calibrated zero point is yet, but it should be close
    // to the middle of the range
    ADC_SetNull(ADC_IA, (MAXCOUNT+1)/2);
    ADC_SetNull(ADC_IB, (MAXCOUNT+1)/2);
    ADC_SetNull(ADC_IC, (MAXCOUNT+1)/2);

    // Interrupts - injected end of queue enabled
    ADC1->IER = ADC_IER_JEOSIE;
    NVIC_SetPriority(ADC1_2_IRQn, PRIO_ADC);
    NVIC_EnableIRQ(ADC1_2_IRQn);
    ADC3->IER = ADC_IER_JEOSIE;
    NVIC_SetPriority(ADC3_IRQn, PRIO_ADC);
    NVIC_EnableIRQ(ADC3_IRQn);

    // DMA interrupts - end of transfer enabled. Also enable the DMA now.
    ADC1_DMACHANNEL->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;
    ADC2_DMACHANNEL->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;
    ADC3_DMACHANNEL->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;
    ADC4_DMACHANNEL->CCR |= DMA_CCR_TCIE | DMA_CCR_EN;
    NVIC_SetPriority(DMA1_Channel1_IRQn, PRIO_ADC);
    NVIC_SetPriority(DMA1_Channel2_IRQn, PRIO_ADC);
    NVIC_SetPriority(DMA1_Channel3_IRQn, PRIO_ADC);
    NVIC_SetPriority(DMA1_Channel4_IRQn, PRIO_ADC);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    // enable the ADCs
    ADC_Enable(ADC1);
    ADC_Enable(ADC2);
    ADC_Enable(ADC3);
    ADC_Enable(ADC4);

    // Start conversions. Note that no conversions will take place until the hardware triggers
    // start coming in from the PWM timer.
    ADC1->CR |= ADC_CR_ADSTART | ADC_CR_JADSTART;
    ADC2->CR |= ADC_CR_ADSTART | ADC_CR_JADSTART;
    ADC3->CR |= ADC_CR_ADSTART | ADC_CR_JADSTART;
    ADC4->CR |= ADC_CR_ADSTART;
}

/**
 * @brief  Converts Vrefint to determine value of VDDA/VREF+
 */
static void ADC_CalcVref(void) {
    // Save current values of things we're gonna change.
    uint32_t temp_smpr2 = ADC1->SMPR2;
    uint32_t temp_sqr1 = ADC1->SQR1;
    uint32_t temp_cfgr = ADC1->CFGR;
    uint32_t temp_cfgr2 = ADC1->CFGR2;
    uint32_t temp_ier = ADC1->IER;

    // Clear 'em.
    ADC1->CFGR = 0;
    ADC1->CFGR2 = 0;
    ADC1->SQR1 = 0;
    ADC1->SMPR2 = 0;
    ADC1->IER = 0;

    float vrefint = 0.0f;
    uint16_t* vrefcal = ((uint16_t*) 0x1FFF75AA); // From STM32G4 datasheet

    if ((*vrefcal >= VREFINTCAL_MIN) && (*vrefcal <= VREFINTCAL_MAX)) { // Between 1.15V and 1.27V (should be around 1.21V)
        vrefint = (ADC_FACTORY_CAL_VOLTAGE) * ((float) (*vrefcal)) / MAXCOUNTF;
    } else { // Default to the regular value of 1.212V

        vrefint = VREFINTDEFAULT;
    }

    // Check if we need to power up
    if(((ADC1->CR & ADC_CR_DEEPPWD) != 0) || ((ADC1->CR & ADC_CR_ADVREGEN) == 0)) {
        ADC1->CR &= (~ADC_CR_DEEPPWD);
        ADC1->CR |= ADC_CR_ADVREGEN;
        Delay(ADC_VREG_STARTUP_DELAY);
    }

    // Turn on ADC1, it will be used alone for this
    ADC_Enable(ADC1);

    // Sampling time for Vrefint has to be at least 4us
    // 247.5 cycles at 42.5MHz = 5.82us
    ADC1->SMPR2 = (0x06) << ((ADC_VREFINT_CH-10U) * 3U);

    // Perform single conversions of the Vrefint channel
    ADC1->SQR1 = ((ADC_VREFINT_CH) << 6U);

    uint32_t vref_sum = 0;
    for (uint8_t i = 0; i < ADC_NUM_INTEG_SAMPLES; i++) {
        ADC1->CR |= ADC_CR_ADSTART;
        while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
            // pass
        }
        ADC1->ISR &= ~(ADC_ISR_EOC);
        vref_sum += ADC1->DR;
    }

    adc_vref = vrefint
            / ((float) (vref_sum / ADC_NUM_INTEG_SAMPLES) / MAXCOUNTF);

    // Reset all the stuff that was changed
    ADC1->SMPR2 = temp_smpr2;
    ADC1->SQR1 = temp_sqr1;
    ADC1->CFGR = temp_cfgr;
    ADC1->CFGR2 = temp_cfgr2;
    ADC1->IER = temp_ier;

}

void ADC_InjSeqComplete(void) {
    // Error on schematic, Ia and Ic current sensors need to be swapped
    adc_conv[ADC_IC] = ADC3->JDR1;
    adc_conv[ADC_IB] = ADC2->JDR1;
    adc_conv[ADC_IA] = ADC1->JDR1;

}

void ADC_RegSeqComplete(void) {
    adc_conv[ADC_VBUS] = adc4_raw_regular_results[0];
    adc_conv[ADC_THR] = adc2_raw_regular_results[4];
    adc_conv[ADC_FTEMP] = adc2_raw_regular_results[6];
    adc_conv[ADC_MTEMP] = adc1_raw_regular_results[0];
    adc_conv[ADC_VA] = adc2_raw_regular_results[0];
    adc_conv[ADC_VB] = adc2_raw_regular_results[2];
    adc_conv[ADC_VC] = adc3_raw_regular_results[0];

    // Apply filters
    adc_fet_filter.X = (float)adc_conv[ADC_FTEMP];
    FOC_BiquadCalc(&adc_fet_filter);
}

/**
 * @brief Enables an ADC, checking that the ready bit shows it's good to go.
 * @param adc The ADC peripheral to enable.
 */
static void ADC_Enable(ADC_TypeDef* adc) {
    if((adc->CR & ADC_CR_ADEN) == 0) {
        adc->ISR |= ADC_ISR_ADRDY;
        adc->CR |= ADC_CR_ADEN;
        while((adc->ISR & ADC_ISR_ADRDY) == 0){
            // pass
        }
    }
}

void ADC_AverageInitialValue(void) {
    // TODO: do I need this? Can I do it after already initializing the ADC?
}

float ADC_ConvertToAmps(int32_t rawCurrentReading) {
    // Assume null point has already been subtracted.
    // The raw reading is a 12-bit number
    float temp_current = ((float) rawCurrentReading) / MAXCOUNTF;
    // Convert to volts
    temp_current *= adc_vref;
    // Convert to amps
    temp_current *= config_adc.Inverse_TIA_Gain;

    return temp_current;
}

float ADC_GetCurrent(uint8_t which_cur) {
    return ADC_ConvertToAmps(
            (int32_t) (adc_conv[which_cur])
                    - (int32_t) (adc_current_null[which_cur]));
}

uint16_t ADC_Raw(uint8_t which_cur) {
    return adc_conv[which_cur];
}

float ADC_GetThrottle(void) {
    float temp_throttle;
    // Convert 12-bit adc result to floating point
    temp_throttle = ((float) adc_conv[ADC_THR]) / MAXCOUNTF;
    // Convert to volts using reference measurement
    temp_throttle *= adc_vref;
    return temp_throttle;
}

float ADC_GetVbus(void) {
    // Convert 12-bit to float
    float temp_vbus = ((float) adc_conv[ADC_VBUS]) / MAXCOUNTF;
    // Convert to volts from reference
    temp_vbus *= adc_vref;
    // Convert to real measurement from resistor divider ratio
//    temp_vbus *= VBUS_RESISTOR_RATIO;
    temp_vbus *= config_adc.Vbus_Ratio;
    return temp_vbus;
}

float ADC_GetVref(void) {
    return adc_vref;
}

float ADC_GetPhaseVoltage(uint8_t which_phase) {
    // Convert 12-bit to float
    float temp_vphase = ((float) adc_conv[which_phase]) / MAXCOUNTF;
    // Convert to volts from reference
    temp_vphase *= adc_vref;
    // Convert to real measurement from resistor divider ratio
    temp_vphase *= config_adc.Vphase_Ratio;
    return temp_vphase;
}

void ADC_SetNull(uint8_t which_cur, uint16_t nullVal) {
    adc_current_null[which_cur] = nullVal;
}

float ADC_GetFetTempDegC(void) {
    
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
    float temp = (adc_fet_filter.Y) / MAXCOUNTF;
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
    temp = (1.0f / 298.15f) - logf(config_adc.Thermistor_R25 / temp) / config_adc.Thermistor_Beta;
    temp = 1.0f / temp;
    temp -= 273.15f; // Convert from K to degC

    return temp;
}

uint8_t ADC_SetRShunt(float new_rshunt) {
    config_adc.Shunt_Resistance = new_rshunt;
    return RETVAL_OK;
}

float ADC_GetRShunt(void) {
    return config_adc.Shunt_Resistance;
}

uint8_t ADC_SetVbusRatio(float new_ratio) {
    config_adc.Vbus_Ratio = new_ratio;
    return RETVAL_OK;
}

float ADC_GetVbusRatio(void) {
    return config_adc.Vbus_Ratio;
}

uint8_t ADC_SetVphaseRatio(float new_ratio) {
    config_adc.Vphase_Ratio = new_ratio;
    return RETVAL_OK;
}

float ADC_GetVphaseRatio(void) {
    return config_adc.Vphase_Ratio;
}

uint8_t ADC_SetThermFixedR(float new_fixed_r) {
    config_adc.Thermistor_Fixed_R = new_fixed_r;
    return RETVAL_OK;
}

float ADC_GetThermFixedR(void) {
    return config_adc.Thermistor_Fixed_R;
}

uint8_t ADC_SetThermR25(float new_r25) {
    config_adc.Thermistor_R25 = new_r25;
    return RETVAL_OK;
}

float ADC_GetThermR25(void){
    return config_adc.Thermistor_R25;
}

uint8_t ADC_SetThermBeta(float new_beta) {
    config_adc.Thermistor_Beta = new_beta;
    return RETVAL_OK;
}

float ADC_GetThermBeta(void) {
    return config_adc.Thermistor_Beta;
}

void ADC_SaveVariables(void) {
    EE_SaveFloat(CONFIG_ADC_RSHUNT, config_adc.Shunt_Resistance);
    EE_SaveFloat(CONFIG_ADC_VBUS_RATIO, config_adc.Vbus_Ratio);
    EE_SaveFloat(CONFIG_ADC_THERM_FIXED_R, config_adc.Thermistor_Fixed_R);
    EE_SaveFloat(CONFIG_ADC_THERM_R25, config_adc.Thermistor_R25);
    EE_SaveFloat(CONFIG_ADC_THERM_B, config_adc.Thermistor_Beta);
}

void ADC_LoadVariables(void) {
    config_adc.Shunt_Resistance = EE_ReadFloatWithDefault(CONFIG_ADC_RSHUNT, DFLT_ADC_RSHUNT);
    config_adc.Vbus_Ratio = EE_ReadFloatWithDefault(CONFIG_ADC_VBUS_RATIO, DFLT_ADC_VBUS_RATIO);
    config_adc.Vphase_Ratio = EE_ReadFloatWithDefault(CONFIG_ADC_VPHASE_RATIO, DFLT_ADC_VPHASE_RATIO);
    config_adc.Thermistor_Fixed_R = EE_ReadFloatWithDefault(CONFIG_ADC_THERM_FIXED_R, DFLT_ADC_THERM_FIXED_R);
    config_adc.Thermistor_R25 = EE_ReadFloatWithDefault(CONFIG_ADC_THERM_R25, DFLT_ADC_THERM_R25);
    config_adc.Thermistor_Beta = EE_ReadFloatWithDefault(CONFIG_ADC_THERM_B, DFLT_ADC_THERM_B);
    // For convenience
    config_adc.Inverse_TIA_Gain = 1.0f / (DEFAULT_CSA_GAIN * config_adc.Shunt_Resistance);
}

// ui versions of commands. all parameters use uint8_t pointers
uint8_t uiADC_SetRShunt(uint8_t* valptr) {
    config_adc.Shunt_Resistance = data_packet_extract_float(valptr);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_GetRShunt(uint8_t* valptr) {
    data_packet_pack_float(valptr, config_adc.Shunt_Resistance);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_SetVbusRatio(uint8_t* valptr) {
    config_adc.Vbus_Ratio = data_packet_extract_float(valptr);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_GetVbusRatio(uint8_t* valptr) {
    data_packet_pack_float(valptr, config_adc.Vbus_Ratio);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_SetVphaseRatio(uint8_t* valptr) {
    config_adc.Vphase_Ratio = data_packet_extract_float(valptr);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_GetVphaseRatio(uint8_t* valptr) {
    data_packet_pack_float(valptr, config_adc.Vphase_Ratio);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_SetThermFixedR(uint8_t* valptr) {
    config_adc.Thermistor_Fixed_R = data_packet_extract_float(valptr);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_GetThermFixedR(uint8_t* valptr) {
    data_packet_pack_float(valptr, config_adc.Thermistor_Fixed_R);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_SetThermR25(uint8_t* valptr) {
    config_adc.Thermistor_R25 = data_packet_extract_float(valptr);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_GetThermR25(uint8_t* valptr) {
    data_packet_pack_float(valptr, config_adc.Thermistor_R25);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_SetThermBeta(uint8_t* valptr) {
    config_adc.Thermistor_Beta = data_packet_extract_float(valptr);
    return DATA_PACKET_SUCCESS;
}

uint8_t uiADC_GetThermBeta(uint8_t* valptr) {
    data_packet_pack_float(valptr, config_adc.Thermistor_Beta);
    return DATA_PACKET_SUCCESS;
}
