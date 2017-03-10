/*
 * pwm.c
 *
 *  Created on: Aug 19, 2015
 *      Author: David
 */

#include "pwm.h"
#include "gpio.h"
#include "pinconfig.h"
#include "project_parameters.h"

/***** Not using this version, using the non-HAL version below.

TIM_HandleTypeDef hpwm;

// Enables the three-phase complementary PWM channels
void PWM_Init(void)
{
	TIM_OC_InitTypeDef ocInit;
	TIM_BreakDeadTimeConfigTypeDef brkInit;
	TIM_MasterConfigTypeDef masterInit;

	// Enable the time base
	hpwm.Instance = PWM_TIMER;
	hpwm.Init.Period = PWM_PERIOD;
	hpwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	hpwm.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	hpwm.Init.Prescaler = 0;
	hpwm.Init.RepetitionCounter = 1;
	HAL_TIM_PWM_Init(&hpwm);

	// Configure the PWM channels
	ocInit.OCMode = TIM_OCMODE_PWM1;
	ocInit.OCIdleState = TIM_OCIDLESTATE_RESET;
	ocInit.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	ocInit.OCPolarity = TIM_OCPOLARITY_HIGH;
	ocInit.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	ocInit.OCFastMode = TIM_OCFAST_DISABLE;
	ocInit.Pulse = PWM_PERIOD/2 + 1;
	HAL_TIM_PWM_ConfigChannel(&hpwm, &ocInit, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&hpwm, &ocInit, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&hpwm, &ocInit, TIM_CHANNEL_3);
	ocInit.OCMode = TIM_OCMODE_PWM1;
	ocInit.Pulse = PWM_PERIOD - 1; // Triggers during downcounting, just after reload
	HAL_TIM_PWM_ConfigChannel(&hpwm, &ocInit, TIM_CHANNEL_4);

	// Configure the break and deadtime
	brkInit.OffStateRunMode = TIM_OSSR_DISABLE;
	brkInit.OffStateIDLEMode = TIM_OSSI_DISABLE;
	brkInit.LockLevel = TIM_LOCKLEVEL_OFF;
	brkInit.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	brkInit.BreakPolarity = TIM_BREAKPOLARITY_LOW;
	brkInit.BreakState = TIM_BREAK_DISABLE;
	brkInit.DeadTime = PWM_DEAD_TIME;
	HAL_TIMEx_ConfigBreakDeadTime(&hpwm, &brkInit);

	// Enable the PWM output positive and negative channels
	PWM_TIMER->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;

	// Configure the master mode (triggers the ADC injected channels with CC4 output)
	masterInit.MasterOutputTrigger = TIM_TRGO_OC4REF;
	masterInit.MasterSlaveMode = TIM_SLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&hpwm, &masterInit);

	// Outputs disabled
	PWM_TIMER->BDTR &= ~(TIM_BDTR_MOE);

	// Enable the update IRQ
	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn,0,0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	// Enable the update interrupt
	PWM_TIMER->DIER |= TIM_DIER_UIE;
	// Start the timer
	PWM_TIMER->CR1 |= TIM_CR1_CEN;
	// Write to the repetition counter so updates occur at underflow events
	PWM_TIMER->RCR = 1;
	// Generate an update to get that immediately latched in
	PWM_TIMER->EGR |= TIM_EGR_UG;
}

 */

void PWM_Init_NoHal(void)
{
	GPIO_Clk(PWM_HI_PORT);
	GPIO_Clk(PWM_LO_PORT);

	PWM_HI_PORT->ODR &= ~((1 << PWM_AHI_PIN) | (1 << PWM_BHI_PIN) | (1 << PWM_CHI_PIN));
	PWM_LO_PORT->ODR &= ~((1 << PWM_ALO_PIN) | (1 << PWM_BLO_PIN) | (1 << PWM_CLO_PIN));

	GPIO_AF(PWM_HI_PORT, PWM_AHI_PIN, PWM_AF);
	GPIO_AF(PWM_HI_PORT, PWM_BHI_PIN, PWM_AF);
	GPIO_AF(PWM_HI_PORT, PWM_CHI_PIN, PWM_AF);
	GPIO_AF(PWM_LO_PORT, PWM_ALO_PIN, PWM_AF);
	GPIO_AF(PWM_LO_PORT, PWM_BLO_PIN, PWM_AF);
	GPIO_AF(PWM_LO_PORT, PWM_CLO_PIN, PWM_AF);

	PWM_TIM_CLK_ENABLE();

	PWM_TIMER->ARR = PWM_PERIOD; // 2*84MHz / 4200 / 2 = 20kHz
	PWM_TIMER->PSC = 0; // No prescaler
	PWM_TIMER->RCR = 1; // Update occurs every full cycle of the PWM timer
	PWM_TIMER->CR1 = TIM_CR1_CMS_0; // Center aligned 1 mode
	PWM_TIMER->CR2 = (TIM_CR2_MMS_2 | TIM_CR2_MMS_1 | TIM_CR2_MMS_0); // OC4REF is trigger out
	PWM_TIMER->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
	PWM_TIMER->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
	PWM_TIMER->CCMR2 = TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;
	PWM_TIMER->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
	PWM_TIMER->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
			TIM_CCER_CC3E | TIM_CCER_CC3NE;
	PWM_TIMER->BDTR = PWM_DEAD_TIME | TIM_BDTR_OSSI; // Dead time selection, and drive outputs low when
													// Motor Enable (MOE) bit is zero

	PWM_TIMER->CCR1 = PWM_PERIOD/2 + 1;
	PWM_TIMER->CCR2 = PWM_PERIOD/2 + 1;
	PWM_TIMER->CCR3 = PWM_PERIOD/2 + 1;
	PWM_TIMER->CCR4 = PWM_PERIOD - 1; // Triggers during downcounting, just after reload

	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, PRIO_PWM); // Highest priority
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

	TIM1->SR = ~(TIM_SR_UIF); // Clear update interrupt (if it was triggered)
	PWM_TIMER->DIER = TIM_DIER_UIE; // Enable update interrupt

	PWM_TIMER->CR1 |= TIM_CR1_CEN; // Start the timer
	PWM_TIMER->RCR = 1; // This picks the underflow as the update event
	PWM_TIMER->EGR |= TIM_EGR_UG; // Generate an update event to latch in all the settings

	/* NOTE: MOE bit is still zero, so outputs are at their inactive level. */
}

/*
inline void PWM_MotorOFF(void)
{
	PWM_TIMER->BDTR &= ~(TIM_BDTR_MOE);
}
inline void PWM_MotorON(void)
{
	PWM_TIMER->BDTR |= TIM_BDTR_MOE;
}
*/

/* Changed so that tA->CCR3, tC->CCR1. tB unchanged (tB->CCR2)
 * The implemented pinout on the STM32F4x5 is TIM1_CH1 -> PA8,
 * TIM1_CH2 -> PA9, and TIM1_CH3 -> PA10. However, the board
 * design has (PWMA+) -> PA10, (PWMB+) -> PA9, and (PWMC+) -> PA8.
 * Since the 1st and 3rd channels are swapped in hardware, here
 * I'm swapping them in software to match.
 *
 * The negative versions of these signals are likewise swapped.
 * E.G., TIM1_CH1N -> PB13, TIM1_CH2N -> PB14, TIM1_CH3N -> PB15
 * but... (PWMA-) -> PB15, (PWMB-) -> PB14, (PWMC-) -> PB13
 */
void PWM_SetDuty(uint16_t tA, uint16_t tB, uint16_t tC)
{
	// scale from 65536 to the maximum counter value, PWM_PERIOD
	uint32_t temp;
	temp = tA * PWM_PERIOD / 65536;
	//PWM_TIMER->CCR1 = temp;
	PWM_TIMER->CCR3 = temp;
	temp = tB * PWM_PERIOD / 65536;
	PWM_TIMER->CCR2 = temp;
	temp = tC * PWM_PERIOD / 65536;
	//PWM_TIMER->CCR3 = temp;
	PWM_TIMER->CCR1 = temp;
}

void PWM_SetDutyF(float tA, float tB, float tC)
{
	PWM_TIMER->CCR1 = (uint16_t)(tA * PWM_PERIOD_F);
	PWM_TIMER->CCR2 = (uint16_t)(tB * PWM_PERIOD_F);
	PWM_TIMER->CCR3 = (uint16_t)(tC * PWM_PERIOD_F);
}
