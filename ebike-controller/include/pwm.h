/******************************************************************************
 * Filename: pwm.h
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
// TIM1
#ifndef PWM_H_
#define PWM_H_

#include "stm32f4xx.h"
#include "pinconfig.h"
#include "periphconfig.h"

/********** Timer defines ***********/

#define PWM_TIMER_FREQ      (168000000L)
#define PWM_PERIOD					(4199) // 168MHz / (4199+1) = 40kHz -> 20kHz due to up/down counting
#define PWM_PERIOD_F				(4199.0f)

/** Deadtime register settings **
 * DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.
 * DTG[7:5]=10x => DT=(64+DTG[5:0])xtdtg with Tdtg=2xtDTS.
 * DTG[7:5]=110 => DT=(32+DTG[4:0])xtdtg with Tdtg=8xtDTS.
 * DTG[7:5]=111 => DT=(32+DTG[4:0])xtdtg with Tdtg=16xtDTS.
 *
 * If tDTS = 1/168MHz, then:
 * DTG[7:5]=0xx => DT=DTG[7:0]x 5.952ns       (range: 0 -> 755.952ns)
 * DTG[7:5]=10x => DT=(64+DTG[5:0])x 11.905ns (range: 761.905 -> 1511.905ns)
 * DTG[7:5]=110 => DT=(32+DTG[4:0])x 47.619ns (range: 1523.809 -> 3000ns)
 * DTG[7:5]=111 => DT=(32+DTG[4:0])x 95.238ns (range: 3047.619 -> 6000ns)
 */

#define DT_RANGE1_MAX     756
#define DT_RANGE2_MAX     1511
#define DT_RANGE3_MAX     3000
#define DT_RANGE4_MAX     6000

#define PWM_DEFAULT_DT_NS   500
#define PWM_DEFAULT_DT_REG  (TIM_BDTR_DTG_2 | TIM_BDTR_DTG_4 | TIM_BDTR_DTG_6) // ~500ns (168MHz / 84 = 2MHz -> 0.5us)

/********** Functions **************/

void PWM_Init(void);
/*
 inline void PWM_MotorOFF(void)
 {
 PWM_TIMER->BDTR &= ~(TIM_BDTR_MOE);
 }
 inline void PWM_MotorON(void)
 {
 PWM_TIMER->BDTR |= (TIM_BDTR_MOE);
 }
 */
void PWM_SetDuty(uint16_t tA, uint16_t tB, uint16_t tC);
void PWM_SetDutyF(float tA, float tB, float tC);
uint8_t PWM_SetDeadTime(int32_t newDT);
int32_t PWM_GetDeadTime(void);
int32_t PWM_GetDeadTime_EEPROM(void);
uint8_t PWM_SetFreq(int32_t freq);
int32_t PWM_GetFreq(void);
int32_t PWM_GetFreq_EEPROM(void);

/* Defines for directly changing PWM outputs */

#define PWM_MotorON()		PWM_TIMER->BDTR |= (TIM_BDTR_MOE)
#define PWM_MotorOFF()		PWM_TIMER->BDTR &= ~(TIM_BDTR_MOE)

// PWM: OC mode is "PWM Mode 1", outputs enabled
#define PHASE_C_PWM()		do{		PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M);	\
									PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1); \
									PWM_TIMER->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);	\
							}while(0)
// Low side on: OC mode is "Force inactive level", outputs enabled
#define PHASE_C_LOW()		do{		PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M);	\
									PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_2); \
									PWM_TIMER->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);	\
							}while(0)
// Phase off: OC mode is "Force inactive", high-side output disabled, low-side enabled (will be outputting low)
#define PHASE_C_OFF()		do{		PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M);	\
									PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC1M_2); \
									PWM_TIMER->CCER &= ~(TIM_CCER_CC1E);	\
									PWM_TIMER->CCER |= (TIM_CCER_CC1NE);	\
							}while(0)

#define PHASE_B_PWM()		do{		PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M);	\
									PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1); \
									PWM_TIMER->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);	\
							}while(0)
#define PHASE_B_LOW()		do{		PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M);	\
									PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_2); \
									PWM_TIMER->CCER |= (TIM_CCER_CC2E | TIM_CCER_CC2NE);	\
							}while(0)
#define PHASE_B_OFF()		do{		PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC2M);	\
									PWM_TIMER->CCMR1 |= (TIM_CCMR1_OC2M_2); \
									PWM_TIMER->CCER &= ~(TIM_CCER_CC2E);	\
									PWM_TIMER->CCER |= (TIM_CCER_CC2NE);	\
							}while(0)

#define PHASE_A_PWM()		do{		PWM_TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M);	\
									PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); \
									PWM_TIMER->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);	\
							}while(0)
#define PHASE_A_LOW()		do{		PWM_TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M);	\
									PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_2); \
									PWM_TIMER->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC3NE);	\
							}while(0)
#define PHASE_A_OFF()		do{		PWM_TIMER->CCMR2 &= ~(TIM_CCMR2_OC3M);	\
									PWM_TIMER->CCMR2 |= (TIM_CCMR2_OC3M_2); \
									PWM_TIMER->CCER &= ~(TIM_CCER_CC3E);	\
									PWM_TIMER->CCER |= (TIM_CCER_CC3NE);	\
							}while(0)
#endif /* PWM_H_ */
