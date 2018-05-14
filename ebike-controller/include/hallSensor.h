/*
 * hallSensor.h
 *
 *  Created on: Aug 14, 2015
 *      Author: David
 */

// Used resources:
// TIM3, TIM4
// DMAx Stream x

#ifndef HALLSENSOR_H_
#define HALLSENSOR_H_

#include "stm32f4xx.h"

#define USE_FLOATING_POINT

/*
#define HALL_PIN_A						GPIO_PIN_6
#define HALL_PIN_A_POS					6	// Used for bit-shifting the Hall state
#define HALL_PIN_B						GPIO_PIN_7
#define HALL_PIN_C						GPIO_PIN_8

#define HALL_PORT						GPIOC
#define HALL_PORT_CLK_ENABLE()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define HALL_PINS_AF					GPIO_AF2_TIM3
*/

#define HALL_TIMER						TIM3
#define HALL_TIM_CLK_ENABLE()			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#define HALL_IRQn						TIM3_IRQn
#define HALL_SAMPLE_TIMER				TIM8
#define HALL_SAMPLE_TIMER_CLK_ENABLE()	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN
#define HALL_DMA						DMA2_Stream1
#define HALL_DMA_CLK_ENABLE()			RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN

#define HALL_TIMER_INPUT_CLOCK			84000000 // APB1 clock * 2
#define HALL_TIMER_INPUT_CLOCK_MHZ		84 // APB1 clock * 2 / 1000000

#define HALL_SAMPLE_PERIOD				1176	// 7us on the APB2 clock (NOT a multiple of 50us!)
#define HALL_NUM_SAMPLES				32 // Number of samples taken of the GPIO

#define HALL_PSC_MIN					15  // 84MHz clock / 16  = 5.25MHz -> 12.5millisec total period
#define HALL_PSC_MAX					511 // 84MHz clock / 512 = 164kHz -> 0.4sec total period
#define HALL_PSC_CHG_AMT				16
#define HALL_MIN_CAPTURE				16384 // First 1/4 of the period
#define HALL_MAX_OVERFLOWS				3

#define HALL_PSC_CHANGED_UP				1
#define HALL_PSC_CHANGED_DOWN			2
#define HALL_PSC_CHANGED				(HALL_PSC_CHANGED_UP | HALL_PSC_CHANGED_DOWN)
#define HALL_STOPPED					4

#define	HALL_ROT_UNKNOWN				0
#define HALL_ROT_FORWARD				1
#define HALL_ROT_REVERSE				2

typedef struct
{
#if defined(USE_FLOATING_POINT)
	float Speed;
	uint32_t CallingFrequency;
	float AngleIncrement;
	float Angle;
	uint32_t CaptureValue;
	uint16_t Prescaler;
	uint8_t Status;
	uint8_t OverflowCount;
	uint8_t RotationDirection;
	uint8_t CurrentState;
#else
	uint32_t Speed; // Expressed in Hz * 2^16 (aka Q16 number)
	uint32_t CallingFrequency; // How rapidly the speed calculation will be updated
	uint32_t CaptureValue;
	uint16_t AngleIncrement;
	uint16_t Angle;
	uint16_t Prescaler;
	uint8_t Status;
	uint8_t OverflowCount;
	uint8_t RotationDirection;
	uint8_t CurrentState;
#endif

}HallSensor_HandleTypeDef;

/************ Functions ************/

uint8_t HallSensor_Get_State(void);
void HallSensor_Inc_Angle(void);
uint16_t HallSensor_Get_Angle(void);
float HallSensor_Get_Anglef(void);
uint32_t HallSensor_Get_Speed(void);
float HallSensor_Get_Speedf(void);
uint8_t HallSensor_Get_Direction(void);
void HallSensor_Init(uint32_t callingFrequency);
void HallSensor_Init_NoHal(uint32_t callingFrequency);
void HallSensor_UpdateCallback(void);
void HallSensor_CaptureCallback(void);

#endif /* HALLSENSOR_H_ */
