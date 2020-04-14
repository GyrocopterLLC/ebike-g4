/******************************************************************************
 * Filename: pwm.h
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

#ifndef PWM_H_
#define PWM_H_

/********** Timer defines ***********/

#define PWM_TIMER           TIM1

#define PWM_TIMER_FREQ      (170000000L)
#define PWM_MIN_FREQ        (5000)
#define PWM_MAX_FREQ        (40000)

#define PWM_PERIOD          (4249) // 170MHz / (4249+1) = 40kHz -> 20kHz due to up/down counting
#define PWM_PERIOD_F        (4249.0f)

/** Deadtime register settings **
 * DTG[7:5]=0xx => DT=DTG[7:0]x Tdtg with tdtg=Tdts.
 * DTG[7:5]=10x => DT=(64+DTG[5:0])xTdtg with Tdtg=2xTdts.
 * DTG[7:5]=110 => DT=(32+DTG[4:0])xTdtg with Tdtg=8xTdts.
 * DTG[7:5]=111 => DT=(32+DTG[4:0])xTdtg with Tdtg=16xTdts.
 *
 * Tdts = 1/170MHz, since TIM_CR1_CKD = 2'b00 (Tdts = Ttim_ker_clk = 170MHz)
 *
 * If Tdts = 1/170MHz, then:
 * DTG[7:5]=0xx => DT=DTG[7:0]x 5.882ns       (range: 0 -> 747.1ns)
 * DTG[7:5]=10x => DT=(64+DTG[5:0])x 11.765ns (range: 752.9 -> 1494.1ns)
 * DTG[7:5]=110 => DT=(32+DTG[4:0])x 47.059ns (range: 1505.9 -> 2964.7ns)
 * DTG[7:5]=111 => DT=(32+DTG[4:0])x 94.118ns (range: 3011.8 -> 5929.4ns)
 */



#define DT_RANGE1_MAX     747
#define DT_RANGE2_MAX     1494
#define DT_RANGE3_MAX     2964
#define DT_RANGE4_MAX     5929

#define PWM_DEFAULT_DT_NS   500
// Default setting is 500ns ... 170MHz / 85 = 2MHz -> 0.5us
// Setting is therefore 85 -> DTG[7:0] = '01010101' (bits 6, 4, 2, and 0)
#define PWM_DEFAULT_DT_REG  (TIM_BDTR_DTG_6 | TIM_BDTR_DTG_4 | TIM_BDTR_DTG_2 | TIM_BDTR_DTG_0)

/********** Functions **************/

void PWM_Init(int32_t freq);
void PWM_SetDuty(uint16_t tA, uint16_t tB, uint16_t tC);
void PWM_SetDutyF(float tA, float tB, float tC);
uint8_t PWM_SetDeadTime(int32_t newDT);
int32_t PWM_GetDeadTime(void);
uint8_t PWM_SetFreq(int32_t freq);
int32_t PWM_GetFreq(void);

/* Defines for directly changing PWM outputs */

#define PWM_MotorON()       PWM_TIMER->BDTR |= (TIM_BDTR_MOE)
#define PWM_MotorOFF()      PWM_TIMER->BDTR &= ~(TIM_BDTR_MOE)

// PWM: OC mode is "PWM Mode 1", outputs enabled
inline void PHASE_C_PWM(void) {
    PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M);
    PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
    PWM_TIMER->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
}
// Low side on: OC mode is "Force inactive level", outputs enabled
inline void PHASE_C_LOW(void) {
    PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M);
    PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_2);
    PWM_TIMER->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
}
// Phase off: OC mode is "Force inactive", high-side output disabled, low-side enabled (will be outputting low)
inline void PHASE_C_OFF(void) {
    PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M);
    PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_2);
    PWM_TIMER->CCER &= ~(TIM_CCER_CC1E);
    PWM_TIMER->CCER |= (TIM_CCER_CC1NE);
}

inline void PHASE_B_PWM(void) {
    PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M);
    PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);
    PWM_TIMER->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
}
inline void PHASE_B_LOW(void) {
    PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M);
    PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_2);
    PWM_TIMER->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);
}

inline void PHASE_B_OFF(void) {
    PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M);
    PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_2);
    PWM_TIMER->CCER &= ~(TIM_CCER_CC2E);
    PWM_TIMER->CCER |= (TIM_CCER_CC2NE);
}

inline void PHASE_A_PWM(void) {
    PWM_TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M);
    PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);
    PWM_TIMER->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

inline void PHASE_A_LOW(void) {
    PWM_TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M);
    PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_2);
    PWM_TIMER->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);
}

inline void PHASE_A_OFF(void) {
    PWM_TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M);
    PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_2);
    PWM_TIMER->CCER &= ~(TIM_CCER_CC3E);
    PWM_TIMER->CCER |= (TIM_CCER_CC3NE);
}

#endif /* PWM_H_ */
