/******************************************************************************
 * Filename: pwm.c
 * Description: Initializes timer hardware on the STM32G4 for pulse width
 *              modulation (PWM) output. This PWM is configured specifically
 *              for motor control: 6 outputs in 3 complementary (high-side and
 *              low-side) channels.
 *
 * Configuration details:
 *              The timer is center aligned, so the clock alternately counts
 *              up and down. When counting up, it flips direction at the
 *              ARR value. When counting down, it counts to zero and flips.
 *
 *              The PWM outputs are set in Mode 1, active when counter value
 *              is low, and inactive when it's high.
 *
 *       ARR --- |     /\          /\
 *               |    /  \        /  \
 *       CCR --- |   /    \      /    \
 *               |  /:    :\    /      \
 *               | / :    : \  /        \
 *                /  :    :  \/          \
 *                   :    :
 *                ___:    :______      ___
 *       ocref       |____|      |____|
 *
 *              The ADC trigger (injected channels) is oc4ref. It is triggered
 *              right about thetop of the count (CCR4 = ARR-1), so the rising
 *              edge is just after the very center of the low period of all the
 *              other PWM outputs. This means that the ADC is triggered when the
 *              low-side FET is turned on, which is necessary for low-side
 *              current shunt sensing.
 *
 *              For regular channels, it's oc5ref. To minimize interference with
 *              the injected channels, the conversion is done at the center of
 *              the pwm high period (counter near zero).
 *
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

float PWM_TIM_arr_f = PWM_PERIOD_F;

uint16_t PWM_DT_ns_to_reg(uint32_t dtns);
uint32_t PWM_DT_reg_to_ns(uint16_t dtreg);

uint16_t PWM_DT_ns_to_reg(uint32_t dtns) {
    uint16_t temp_bdtr = 0x00;
    if (dtns < DT_RANGE1_MAX) {
        temp_bdtr = (uint32_t) (((float) dtns) * 0.170f);
        temp_bdtr &= 0x7F;
    } else if (dtns < DT_RANGE2_MAX) {
        temp_bdtr = (uint32_t) (((float) dtns) * 0.085f);
        temp_bdtr = (temp_bdtr > 64) ? (temp_bdtr - 64) : 0;
        if (temp_bdtr > 63)
            temp_bdtr = 63;
        temp_bdtr |= 0x80;
    } else if (dtns < DT_RANGE3_MAX) {
        temp_bdtr = (uint32_t) (((float) dtns) * 0.02125f);
        temp_bdtr = (temp_bdtr > 32) ? (temp_bdtr - 32) : 0;
        if (temp_bdtr > 31)
            temp_bdtr = 31;
        temp_bdtr |= 0xC0;
    } else if (dtns < DT_RANGE4_MAX) {
        temp_bdtr = (uint32_t) (((float) dtns) * 0.010625f);
        temp_bdtr = (temp_bdtr > 32) ? (temp_bdtr - 32) : 0;
        if (temp_bdtr > 31)
            temp_bdtr = 31;
        temp_bdtr |= 0xE0;
    }

    return temp_bdtr;
}

uint32_t PWM_DT_reg_to_ns(uint16_t dtreg) {
    float temp;
    if ((dtreg & 0x80) == 0) {
        temp = ((float) (dtreg)) / 0.170f;
    } else if ((dtreg & 0xC0) == 0x80) {
        temp = ((float) ((0x3F & dtreg) + 64)) / 0.085f;
    } else if ((dtreg & 0xE0) == 0xC0) {
        temp = ((float) ((0x1F & dtreg) + 32)) / 0.02125f;
    } else {
        temp = ((float) ((0x1F & dtreg) + 32)) / 0.010625f;
    }

    return ((uint32_t) (temp));
}

void PWM_Init(int32_t freq) {
    GPIO_Clk(PWM_HI_PORT);
    GPIO_Clk(PWM_LO_PORT);

    // Force GPIO outputs low. If the Timer releases the output pins, this ensures that the
    // FETs will not be turned on inadvertently.
    PWM_HI_PORT->ODR &= ~((1 << PWM_AHI_PIN) | (1 << PWM_BHI_PIN)
            | (1 << PWM_CHI_PIN));
    PWM_LO_PORT->ODR &= ~((1 << PWM_ALO_PIN) | (1 << PWM_BLO_PIN)
            | (1 << PWM_CLO_PIN));

    GPIO_AF(PWM_HI_PORT, PWM_AHI_PIN, PWM_AHI_AF);
    GPIO_AF(PWM_HI_PORT, PWM_BHI_PIN, PWM_BHI_AF);
    GPIO_AF(PWM_HI_PORT, PWM_CHI_PIN, PWM_CHI_AF);
    GPIO_AF(PWM_LO_PORT, PWM_ALO_PIN, PWM_ALO_AF);
    GPIO_AF(PWM_LO_PORT, PWM_BLO_PIN, PWM_BLO_AF);
    GPIO_AF(PWM_LO_PORT, PWM_CLO_PIN, PWM_CLO_AF);
    GPIO_AF(PWM_BKIN_PORT, PWM_BKIN_PIN, PWM_BKIN_AF);
    // Also enable pullup on brake input
    GPIO_SetPUPD(PWM_BKIN_PORT, PWM_BKIN_PIN, PuPd_PullUp);

    PWM_TIM_CLK_ENABLE();

    PWM_SetFreq(freq);
    PWM_TIM->PSC = 0; // No prescaler
    PWM_TIM->RCR = 1; // Update occurs every full cycle of the PWM timer
    PWM_TIM->CR1 = TIM_CR1_CMS_0; // Center aligned 1 mode
    PWM_TIM->CR2 = (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0); // OC4REF is trigger out
    PWM_TIM->CR2 |= (TIM_CR2_MMS2_3); // OC5REF is trigger out 2
    // Set Capture/Compare mode to PWM mode 1 (active from zero to CCRx, inactive from CCRx to ARR)
    PWM_TIM->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1
            | TIM_CCMR1_OC2M_2;
    // Preload enable, CCR1/CCR2 take effect at update events
    PWM_TIM->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    // Set Capture/Compare mode to PWM mode 1
    PWM_TIM->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1
            | TIM_CCMR2_OC4M_2;
    // Preload enable, CCR3/CCR4 take effect at update events
    PWM_TIM->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
    // Set Capture/Compare mode to PWM mode 1
    PWM_TIM->CCMR3 = TIM_CCMR3_OC5M_1 | TIM_CCMR3_OC5M_2;
    // Preload enable, CCR5 takes effect at update events
    PWM_TIM->CCMR3 |= TIM_CCMR3_OC5PE;
    // Enable Channels 1, 2, and 3 both complementary outputs
    PWM_TIM->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E
            | TIM_CCER_CC2NE |
            TIM_CCER_CC3E | TIM_CCER_CC3NE;
    // Dead time selection, and drive outputs low when Motor Enable (MOE) bit is zero
    PWM_TIM->BDTR = PWM_DEFAULT_DT_REG | TIM_BDTR_OSSI;
    // Break input 1 enable
    PWM_TIM->BDTR |= TIM_BDTR_BKE | TIM_BDTR_BKP;
    PWM_TIM->AF1 = TIM1_AF1_BKINE | TIM1_AF1_BKINP;

    PWM_TIM->CCR1 = PWM_PERIOD / 2 + 1;
    PWM_TIM->CCR2 = PWM_PERIOD / 2 + 1;
    PWM_TIM->CCR3 = PWM_PERIOD / 2 + 1;
    PWM_TIM->CCR4 = PWM_PERIOD - 1; // Triggers during downcounting, just after reload
    PWM_TIM->CCR5 = 1; // Triggers during downcounting, just before hitting zero

    NVIC_SetPriority(PWM_IRQn, PRIO_PWM); // Highest priority
    NVIC_EnableIRQ(PWM_IRQn);

    // For odd values of RCR in center aligned mode, the update is either on overflows
    // or underflows depending on when RCR was written and counter was launched.
    // If RCR written before launching the timer, UEV happens on underflow.
    // Otherwise, UEV happens on overflow.

    PWM_TIM->RCR = 1;
    PWM_TIM->EGR |= TIM_EGR_UG; // Generate an update event to latch in all the settings
    PWM_TIM->CR1 |= TIM_CR1_CEN; // Start the timer

    PWM_TIM->SR = ~(TIM_SR_UIF); // Clear update interrupt (if it was triggered)
    PWM_TIM->DIER = TIM_DIER_UIE | TIM_DIER_BIE; // Enable update interrupt and break interrupt

    /* NOTE: MOE bit is still zero, so outputs are at their inactive level. */
}

uint8_t PWM_SetDeadTime(int32_t newDT) {
    uint32_t temp_bdtr = PWM_TIM->BDTR;
    temp_bdtr &= 0xFFFFFF00U; // Clear DTG[7:0] bits
    temp_bdtr |= PWM_DT_ns_to_reg(newDT);
    PWM_TIM->BDTR = temp_bdtr;
    return RETVAL_OK;
}
int32_t PWM_GetDeadTime(void) {
    uint32_t temp_bdtr = PWM_TIM->BDTR & (0x000000FF);
    return PWM_DT_reg_to_ns(temp_bdtr);
}

uint8_t PWM_SetFreq(int32_t newFreq) {
    int32_t temp = PWM_CLK;
    int32_t tempcr1;
    int32_t tempbdtr;
    if((newFreq >= PWM_MIN_FREQ) && (newFreq <= PWM_MAX_FREQ)) {
        temp = temp / newFreq;
        temp = temp / 2;
        temp = temp - 1;
        tempbdtr = PWM_TIM->BDTR;
        PWM_MotorOFF();
        tempcr1 = PWM_TIM->CR1; // Save current CR1 value
        PWM_TIM->CR1 &= ~TIM_CR1_CEN; // Stop the timer if it's running
        PWM_TIM->ARR = temp;
        PWM_TIM->EGR |= TIM_EGR_UG; // Generate an update event to latch in all the settings
        PWM_TIM->CR1 = tempcr1; // Restart the timer if it was running
        PWM_TIM->BDTR = tempbdtr; // Turn on outputs if they were on

        return RETVAL_OK;
    }

    return RETVAL_FAIL;
}

int32_t PWM_GetFreq(void) {
    int32_t temp = PWM_CLK;
    temp = temp / ((PWM_TIM->ARR) + 1);
    return temp;
}

/* Changed so that tA->CCR3, tC->CCR1. tB unchanged (tB->CCR2)
 * The implemented pinout on the STM32G4xx is TIM1_CH1 -> PA8,
 * TIM1_CH2 -> PA9, and TIM1_CH3 -> PA10. However, the board
 * design has (PWMA+) -> PA10, (PWMB+) -> PA9, and (PWMC+) -> PA8.
 * Since the 1st and 3rd channels are swapped in hardware, here
 * I'm swapping them in software to match.
 *
 * The negative versions of these signals are likewise swapped.
 * E.G., TIM1_CH1N -> PB13, TIM1_CH2N -> PB14, TIM1_CH3N -> PB15
 * but... (PWMA-) -> PB15, (PWMB-) -> PB14, (PWMC-) -> PB13
 */
void PWM_SetDuty(uint16_t tA, uint16_t tB, uint16_t tC) {
    // scale from 65536 to the maximum counter value, PWM_PERIOD
    uint32_t temp;
    temp = tC * PWM_TIM->ARR / 65536;
    PWM_TIM->CCR1 = temp;
    temp = tB * PWM_TIM->ARR / 65536;
    PWM_TIM->CCR2 = temp;
    temp = tA * PWM_TIM->ARR / 65536;
    PWM_TIM->CCR3 = temp;
}

void PWM_SetDutyF(float tA, float tB, float tC) {
    PWM_TIM->CCR1 = (uint16_t) (tC * PWM_TIM_arr_f);
    PWM_TIM->CCR2 = (uint16_t) (tB * PWM_TIM_arr_f);
    PWM_TIM->CCR3 = (uint16_t) (tA * PWM_TIM_arr_f);
}

