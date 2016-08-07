/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"


/** DAC MSP Init
 * Enables the DAC clock and GPIO pins
 *
 * GPIO A4 -> DAC1
 * GPIO A5 -> DAC2
 *
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
	GPIO_InitTypeDef gpioInit;
	if(hdac->Instance == DAC)
	{
		__HAL_RCC_DAC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		gpioInit.Pin = GPIO_PIN_4 | GPIO_PIN_5;
		gpioInit.Mode = GPIO_MODE_ANALOG;
		gpioInit.Pull = GPIO_NOPULL;
		gpioInit.Speed = GPIO_SPEED_HIGH;

		HAL_GPIO_Init(GPIOA, &gpioInit);
	}
}

/** Timer Base MSP Init
 * Enables timer clocks and GPIO pins
 *
 * TIM12: Used as a basic application timer. No GPIO.
 *
 * TIM3: Hall Sensor Interface
 * GPIO C6 -> TIM3_CH1
 * GPIO C7 -> TIM3_CH2
 * GPIO C8 -> TIM3_CH3
 *
 */

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim)
{
	GPIO_InitTypeDef gpioInit;

	if(htim->Instance == TIM12)
	{
		__HAL_RCC_TIM12_CLK_ENABLE();
	}
	if(htim->Instance == TIM3)
	{
		// The HallSensor version will probably be called, not this one
		HALL_TIM_CLK_ENABLE();
		//HALL_PORT_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();

		gpioInit.Pin = (HALL_PIN_A | HALL_PIN_B | HALL_PIN_C);
		gpioInit.Pull = GPIO_NOPULL;
		gpioInit.Speed = GPIO_SPEED_HIGH;
		gpioInit.Mode = GPIO_MODE_AF_PP;
		gpioInit.Alternate = HALL_PINS_AF;
		HAL_GPIO_Init(HALL_PORT,&gpioInit);
	}
}

/** Initializes hardware for the three phase bridge PWM
 * PA8 -> TIM1_CH1 (PWM C High)
 * PA9 -> TIM1_CH2 (PWM B High)
 * PA10 -> TIM1_CH3 (PWM A High)
 * PB13 -> TIM1_CH1N (PWM C Low)
 * PB14 -> TIM1_CH2N (PWM B Low)
 * PB15 -> TIM1_CH3N (PWM A Low)
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef gpioInit;

	if(htim->Instance == TIM1)
	{
		PWM_TIM_CLK_ENABLE();

		//PWM_HI_GPIO_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		//PWM_LO_GPIO_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		gpioInit.Pin = PWM_AHI_PIN | PWM_BHI_PIN | PWM_CHI_PIN;
		gpioInit.Mode = GPIO_MODE_AF_PP;
		gpioInit.Pull = GPIO_NOPULL;
		gpioInit.Speed = GPIO_SPEED_HIGH;
		gpioInit.Alternate = PWM_AF;
		HAL_GPIO_Init(PWM_HI_PORT, &gpioInit);
		PWM_HI_PORT->ODR &= ~(PWM_AHI_PIN | PWM_BHI_PIN | PWM_CHI_PIN);

		gpioInit.Pin = PWM_ALO_PIN | PWM_BLO_PIN | PWM_CLO_PIN;
		HAL_GPIO_Init(PWM_LO_PORT, &gpioInit);
		PWM_LO_PORT->ODR &= ~(PWM_ALO_PIN | PWM_BLO_PIN | PWM_CLO_PIN);
	}
}

/** Initializes the hardware resources for the Hall sensor timer
 * PC6 -> TIM3_CH1 (Hall A)
 * PC7 -> TIM3_CH2 (Hall B)
 * PC8 -> TIM3_CH3 (Hall C)
 */
void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef gpioInit;

	if(htim->Instance == HALL_TIMER)
	{
		HALL_TIM_CLK_ENABLE();
		//HALL_PORT_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();

		gpioInit.Pin = (HALL_PIN_A | HALL_PIN_B | HALL_PIN_C);
		gpioInit.Pull = GPIO_PULLUP;
		gpioInit.Speed = GPIO_SPEED_HIGH;
		gpioInit.Mode = GPIO_MODE_AF_PP;
		gpioInit.Alternate = HALL_PINS_AF;
		HAL_GPIO_Init(HALL_PORT,&gpioInit);
	}
}
