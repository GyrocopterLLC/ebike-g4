/*
 * hallSensor.c
 *
 *  Created on: Aug 14, 2015
 *      Author: David
 */

#include "hallSensor.h"
#include "gpio.h"
#include "pinconfig.h"

/*################### Private variables #####################################*/
TIM_HandleTypeDef hHallTim;
HallSensor_HandleTypeDef HallSensor;
#if defined(HALL_DEBUG_SOURCE)
uint8_t debug_state = 1;
uint8_t debug_direction = 0;
uint8_t debug_freq = 4;
#endif

uint16_t HallStateAngles[8] = { 0, // State 0 -> Invalid
								0, // State 1 -> 0°
								43691, // State 2 -> 240°
								54613, // State 3 -> 300°
								21845, // State 4 -> 120°
								10923, // State 5 -> 60°
								32768, // State 6 -> 180°
								0}; // State 7 -> Invalid

float HallStateAnglesFloat[8] = { 0.0f, // State 0 -> Invalid
							  	  0.0f, // State 1 -> 0
								  0.666666667f, // State 2 -> 240
								  0.833333333f, // State 3 -> 300
								  0.333333333f, // State 4 -> 120
								  0.166666667f, // State 5 -> 60
								  0.5f, // State 6 -> 180
								  0.0f}; // State 7 -> Invalid

// Motor rotation order -> 1, 5, 4, 6, 2, 3...
uint8_t HallStateOrder[8] = { 0, // State 0 -> Invalid
							  1, // State 1 -> First
							  5, // State 2 -> Fifth
							  6, // State 3 -> Sixth
							  3, // State 4 -> Third
							  2, // State 5 -> Second
							  4, // State 6 -> Fourth
							  0}; // State 7 -> Invalid
uint8_t DebugOrder[8] = {0, 1, 5, 4, 6, 2, 3, 0};

/*################### Private function declarations #########################*/
static void HallSensor_CalcSpeed(void);

/*################### Public functions ######################################*/

/** HallSensor_Get_State
 * Retrieves the state (number 0-7) corresponding to the Hall Sensor
 * inputs. States 0 and 7 are invalid, since the Hall Sensors should
 * never be all low or all high. States 1-6 are valid states.
 */

uint8_t HallSensor_Get_State(void)
{
#if defined(HALL_DEBUG_SOURCE)
	debug_state++;
	if((debug_state == 0) || (debug_state >= 7))
		debug_state = 1;
	return DebugOrder[debug_state];
#else
	uint32_t temp;
	uint8_t state = 0;
	// Pull out only the Hall pins A, B, and C
	temp = HALL_PORT->IDR;
	state += (temp & (1<<HALL_PIN_A)) ? 1 : 0;
	state += (temp & (1<<HALL_PIN_B)) ? 2 : 0;
	state += (temp & (1<<HALL_PIN_C)) ? 4 : 0;

	return state;
#endif
}

/** HallSensor_Inc_Angle
 * Speed and timing info, plus hall state at the last captured edge,
 * are used to interpolate the angle within a 60° sector.
 */
void HallSensor_Inc_Angle(void)
{
	// Increment the angle by the pre-calculated increment amount
	HallSensor.Angle += HallSensor.AngleIncrement;
#if defined(USE_FLOATING_POINT)
	// Wraparound for floating point. Fixed point simply overflows the 16 bit variable.
	if(HallSensor.Angle > 1.0f)
		HallSensor.Angle -= 1.0f;
	if(HallSensor.Angle < 0.0f)
		HallSensor.Angle += 1.0f;
#endif
}

/** HallSensor_Get_Angle
 * Retrieves the motor electrical angle as a function of the Hall state.
 */
uint16_t HallSensor_Get_Angle(void)
{

#if defined(USE_FLOATING_POINT)
	return (uint16_t)(HallSensor.Angle * 65536.0f);
#else
	return HallSensor.Angle;
#endif
}
/** HallSensor_Get_Speed
 * Returns the electrical speed in Hz in the form Q16.16 (32 bit number, where the
 * least significant 16 bits are fractional).
 */
uint32_t HallSensor_Get_Speed(void)
{
#if defined(USE_FLOATING_POINT)
	return (uint32_t)(HallSensor.Speed * 65536.0f);
#else
	return HallSensor.Speed;
#endif
}

/** HallSensor_Init
 * Starts the time base for the Hall Sensor Timer and (through the MSP init function)
 * the GPIOs associated with the Hall Sensors. The timer is started in the UP counting
 * mode, with a prescaler that results in about a 10KHz clock. The period is always at the
 * max value, 0xFFFF (=65535 decimal). The first overflow will occur at about 6 and a
 * half seconds.
 * While the timer is running and the Hall effect sensor inputs are switching, the prescaler
 * is adjusted to keep the interrupts around half of the maximum period. This results in better
 * granularity and more accurate measurement of the motor speed.
 */

void HallSensor_Init(uint32_t callingFrequency)
{
	TIM_HallSensor_InitTypeDef hallInit;

	hHallTim.Instance = HALL_TIMER;
	hHallTim.Init.Prescaler = HALL_PSC_MAX; // 84MHz timer clock / (511+1) = 164kHz
	hHallTim.Init.CounterMode = TIM_COUNTERMODE_UP;
	hHallTim.Init.Period = 0xFFFF;
	hHallTim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Filter clock Fdts is 84MHz

	hallInit.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
	hallInit.IC1Prescaler = TIM_ICPSC_DIV1;
	hallInit.IC1Filter = 0x8; // Filter is 6 samples at Fdts/8 (1.75MHz ~= 571ns)
	hallInit.Commutation_Delay = 0; // Not using this anyway

	HAL_TIMEx_HallSensor_Init(&hHallTim, &hallInit);

	HALL_TIMER->CR1 |= TIM_CR1_URS; // Reset through slave controller does not cause an Update interrupt
	HALL_TIMER->DIER |= TIM_DIER_UIE; // Update interrupt enabled ... only occurs when timer rolls over without hall sensor change

	HAL_NVIC_SetPriority(HALL_IRQn,2,0);
	HAL_NVIC_EnableIRQ(HALL_IRQn);

	HAL_TIMEx_HallSensor_Start_IT(&hHallTim);

	HallSensor.Prescaler = HALL_PSC_MAX;
	HallSensor.Status = 0;
#if defined(USE_FLOATING_POINT)
	HallSensor.Speed = 0.0f;
#else
	HallSensor.Speed = 0;
#endif
	HallSensor.CallingFrequency = callingFrequency;
	HallSensor.OverflowCount = 0;
	HallSensor.Status |= HALL_STOPPED;
}

void HallSensor_Init_NoHal(uint32_t callingFrequency)
{

#if defined(HALL_DEBUG_SOURCE)

	HALL_TIM_CLK_ENABLE();

	HALL_TIMER->PSC = HALL_PSC_MAX; // Set the prescaler as high as possible to start
	HALL_TIMER->ARR = 0xFFFF; // Auto reload always at max

	HALL_TIMER->CCMR1 = TIM_CCMR1_CC1S; // Channel 1 is input
	HALL_TIMER->CCMR1 |= TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_0; // Filter set to 8 samples
															// at Fdts/8 (2.625MHz)
	HALL_TIMER->SMCR = TIM_SMCR_TS_0 | TIM_SMCR_SMS_2; // Reset mode, input is ITR1 (TIM2 TRGO)
	//HALL_TIMER->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_SMS_2; // Reset mode, input is TI1F_ED (Channel 1
														// input, filtered, edge detector)
	HALL_TIMER->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // Input 1 enabled, both
																	// edges captured
	HALL_TIMER->CR1 = TIM_CR1_URS; // The slave mode resets (capture interrupts) won't trigger
									// an update interrupt, only a timer overflow will.
	HALL_TIMER->CR1 |= TIM_CR1_CKD_1; // Input filter clock = timer clock / 4
	HALL_TIMER->CR2 = TIM_CR2_TI1S; // Channels 1, 2, and 3 are XOR'd together into Channel 1

	HALL_TIMER->EGR |= TIM_EGR_UG; // Trigger an update to get all those shadow registers set

	NVIC_SetPriority(HALL_IRQn,2);
	NVIC_EnableIRQ(HALL_IRQn);

	HALL_TIMER->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE; // Enable channel 1 and update interrupts
	HALL_TIMER->CR1 |= TIM_CR1_CEN; // Start the timer

	HallSensor.Prescaler = HALL_PSC_MAX;
	HallSensor.Status = 0;
	HallSensor.Speed = 0;
	HallSensor.CallingFrequency = callingFrequency;
	HallSensor.OverflowCount = 0;
	HallSensor.Status |= HALL_STOPPED;


	/**** TIM2 Settings, used to trigger Hall sensor captures ****/
	__HAL_RCC_TIM2_CLK_ENABLE();
	TIM2->CR1 = 0;
	TIM2->CR2 = TIM_CR2_MMS_2; // OC1REF is TRGO
	TIM2->PSC = 399; // 84MHz / 400 = 210kHz
	TIM2->ARR = 70000; // 210kHz / 70000 = 3.0 Hz
	TIM2->CCR1 = 20;
	TIM2->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode 1
	TIM2->CR1 |= TIM_CR1_CEN;

#else
	//HALL_PORT_CLK_ENABLE();
	GPIO_Clk(HALL_PORT);
	HALL_TIM_CLK_ENABLE();

	// Enable GPIOs!
	GPIO_AF(HALL_PORT, HALL_PIN_A, HALL_PINS_AF);
	GPIO_AF(HALL_PORT, HALL_PIN_B, HALL_PINS_AF);
	GPIO_AF(HALL_PORT, HALL_PIN_C, HALL_PINS_AF);

	HALL_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << HALL_PIN_A);
	HALL_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << HALL_PIN_B);
	HALL_PORT->PUPDR |= (GPIO_PUPDR_PUPDR0_0 << HALL_PIN_C);

	HALL_TIMER->PSC = HALL_PSC_MAX; // Set the prescaler as high as possible to start
	HALL_TIMER->ARR = 0xFFFF; // Auto reload always at max

	HALL_TIMER->CCMR1 = TIM_CCMR1_CC1S; // Channel 1 is input
	HALL_TIMER->CCMR1 |= TIM_CCMR1_IC1F_3 | TIM_CCMR1_IC1F_0; // Filter set to 8 samples
															// at Fdts/8 (2.625MHz)
	HALL_TIMER->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_SMS_2; // Reset mode, input is TI1F_ED (Channel 1
														// input, filtered, edge detector)
	HALL_TIMER->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP; // Input 1 enabled, both
																	// edges captured
	HALL_TIMER->CR1 = TIM_CR1_URS; // The slave mode resets (capture interrupts) won't trigger
									// an update interrupt, only a timer overflow will.
	HALL_TIMER->CR1 |= TIM_CR1_CKD_1; // Input filter clock = timer clock / 4
	HALL_TIMER->CR2 = TIM_CR2_TI1S; // Channels 1, 2, and 3 are XOR'd together into Channel 1

	HALL_TIMER->EGR |= TIM_EGR_UG; // Trigger an update to get all those shadow registers set

	NVIC_SetPriority(HALL_IRQn,2);
	NVIC_EnableIRQ(HALL_IRQn);

	HALL_TIMER->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE; // Enable channel 1 and update interrupts
	HALL_TIMER->CR1 |= TIM_CR1_CEN; // Start the timer

	HallSensor.Prescaler = HALL_PSC_MAX;
	HallSensor.Status = 0;
	HallSensor.Speed = 0;
	HallSensor.CallingFrequency = callingFrequency;
	HallSensor.OverflowCount = 0;
	HallSensor.Status |= HALL_STOPPED;


#endif
}

/** HallSensor_CalcSpeed
 * Called from the capture interrupt. The capture value is the period of time between Hall Sensor
 * state changes. This function needs to (1) determine the timebase as a function of the timer
 * clock and prescaler, and (2) determine the motor electrical speed as the inverse of the period
 * between state changes.
 */
static void HallSensor_CalcSpeed(void)
{
	// Timer input clock / prescaler = actual timer clock
	// Actual timer clock / capture counts = Hall state transition frequency
	// Hall state frequency / 6 = motor electrical frequency

#if defined(USE_FLOATING_POINT)
	HallSensor.Speed = ((float)HALL_TIMER_INPUT_CLOCK) /(6.0f * ((float)(HallSensor.Prescaler + 1)) * ((float)HallSensor.CaptureValue) );
	HallSensor.AngleIncrement = HallSensor.Speed / ((float)HallSensor.CallingFrequency);
#else

	uint32_t freq = ((uint32_t)HALL_TIMER_INPUT_CLOCK) << 4; // Clock input frequency in (Hz * 2^4)
	freq = freq / (HallSensor.Prescaler + 1); // Actual timer frequency in (Hz * 2^4)
	if(HallSensor.CaptureValue != 0) // No divides by zero please
	{
		freq = freq / HallSensor.CaptureValue; // Frequency of Hall state change in (Hz * 2^4)
		HallSensor.Speed = ((uint32_t)(freq<<12)) / 6; // Speed in Hz*2^16
	}
	else
	{
		HallSensor.Speed = 0;
	}

	//HallSensor.AngleIncrement = ((uint32_t)(HallSensor.Speed * 6) / HallSensor.CallingFrequency) / 6;
	// Cancel out the 6's
	/** The angle increment is the amount of angle to add each time the angle is updated **/
	HallSensor.AngleIncrement = (uint32_t)(HallSensor.Speed) / HallSensor.CallingFrequency;

#endif
}

/** HallSensor_UpdateCallback
 * Triggered when the Hall Sensor timer overflows (counts past 65535)
 * This means that no Hall change occurred for the entire counter duration.
 * So in this case, we set the speed to zero and try to lengthen the counter.
 * If the prescaler is already at the maximum value, it can't be lengthened any more.
 */
void HallSensor_UpdateCallback(void)
{
	HallSensor.OverflowCount++;
	if(HallSensor.OverflowCount >= HALL_MAX_OVERFLOWS)
	{
		// Limit overflow counter
		HallSensor.OverflowCount = HALL_MAX_OVERFLOWS;
		// Set speed to zero - stopped motor
#if defined(USE_FLOATING_POINT)
		HallSensor.Speed = 0.0f;
		HallSensor.AngleIncrement = 0.0f;
#else
		HallSensor.Speed = 0;
		HallSensor.AngleIncrement = 0;
#endif
		HallSensor.Status |= HALL_STOPPED;
		HallSensor.Prescaler = HALL_PSC_MAX;
		HALL_TIMER->PSC = HALL_PSC_MAX;
	}
}

/**
 * HallSensor_CaptureCallback
 * Triggered when any of the three Hall Sensor switches change state.
 * This function stores the most recent speed information. If the switch change
 * occurred in the first 1/8 of the timer period, the prescaler is reduced to
 * shorten the timer period. Likewise, if it occurred after 7/8 of the period,
 * the timer period is extended.
 */
void HallSensor_CaptureCallback(void)
{
#if defined(HALL_DEBUG_SOURCE)
	if(debug_state == 6)
	{
		if(debug_direction == 0)
		{
			if(debug_freq >= 100)
			{
				debug_direction = 1;
			}
			else
			{
				debug_freq += 1;
				(TIM2->ARR) = 210000 / debug_freq;
			}
		}
		else if(debug_direction == 1)
		{
			if(debug_freq <= 4)
			{
				debug_direction = 0;
			}
			else
			{
				debug_freq -= 1;
				(TIM2->ARR) = 210000 / debug_freq;
			}
		}
	}

#endif

	HallSensor.CaptureValue = HALL_TIMER->CCR1;

	// Update the angle - just encountered a 60deg marker (the Hall state change)
#if defined(USE_FLOATING_POINT)
	HallSensor.Angle = HallStateAnglesFloat[HallSensor_Get_State()];
	//HallSensor.Angle = ((float)HallStateAngles[HallSensor_Get_State()]) / 65536.0f;
#else
	HallSensor.Angle = HallStateAngles[HallSensor_Get_State()];
#endif

	if(HallSensor.OverflowCount > 0)
	{
		// Fix the capture value for the speed calculation
		// Include the duration of the timer for each overflow that occurred
		HallSensor.CaptureValue += ((HallSensor.OverflowCount)*0xFFFF);
	}

	// Only calculate speed if there have been two consecutive captures without stopping
	if((HallSensor.Status & HALL_STOPPED) == 0)
		HallSensor_CalcSpeed();
	else
		HallSensor.Status &= ~(HALL_STOPPED);

	// Update prescaler if needed - can't change if it was just adjusted in the last capture
	if((HallSensor.Status & HALL_PSC_CHANGED) == 0)
	{
		if(HallSensor.CaptureValue <= HALL_MIN_CAPTURE)
		{
			if(HallSensor.Prescaler > HALL_PSC_MIN)
			{
				HALL_TIMER->PSC = HallSensor.Prescaler - HALL_PSC_CHG_AMT;
				HallSensor.Status |= HALL_PSC_CHANGED_DOWN;
			}
		}
		if(HallSensor.OverflowCount > 0)
		{
			if(HallSensor.Prescaler < HALL_PSC_MAX)
			{
				HALL_TIMER->PSC = HallSensor.Prescaler + HALL_PSC_CHG_AMT;
				HallSensor.Status |= HALL_PSC_CHANGED_UP;
			}
		}
	}
	else	// It was previously changed, time to take it into effect
			// This is now safe to do since the speed calculation is already done
	{
		HallSensor.Prescaler = HALL_TIMER->PSC;
		HallSensor.Status &= ~(HALL_PSC_CHANGED);
	}
	// Now it's safe to clear overflow counts
	HallSensor.OverflowCount = 0;
}
