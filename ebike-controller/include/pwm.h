/*
 * pwm.h
 *
 *  Created on: Aug 19, 2015
 *      Author: David
 */

// Used resources:
// TIM1

#ifndef PWM_H_
#define PWM_H_

#include "stm32f4xx.h"
#include "pinconfig.h"
#include "periphconfig.h"


/********** Timer defines ***********/

#define PWM_PERIOD					4199 // 168MHz / (4199+1) = 40kHz -> 20kHz due to up/down counting
#define PWM_PERIOD_F				4199.0f
// Dead time calculation rules:
// If DTG[7] = 0:
// --- Dead time is DTG[6:0]*(t_DTS)
// If DTG[7:6] = 10
// --- Dead time is (DTG[5:0]+64)*(2*t_DTS)
// If DTG[7:5] = 110
// --- Dead time is (DTG[4:0]+32)*(8*t_DTS)
// If DTG[7:5] = 111
// --- Dead time is (DTG[4:0]+32)*(16*t_DTS)

// Since f_DTS is 168MHz (same as clock input), setting DTG = 01010100 gives us
// Dead time = t_DTS * DTG[6:0] = (1/168MHz) * 84 = 5.95ns * 84 = 500ns
#define PWM_DT_NS           500
#define PWM_DEAD_TIME				(TIM_BDTR_DTG_2 | TIM_BDTR_DTG_4 | TIM_BDTR_DTG_6) // ~500ns (168MHz / 84 = 2MHz -> 0.5us)

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
