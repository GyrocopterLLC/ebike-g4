#ifndef _THROTTLE_H_
#define _THROTTLE_H_

#include "stm32f4xx.h"
#include "DavidsFOCLib.h"
#include "adc.h"
#include "project_parameters.h"

#define PAS1_TIM                  TIM13
#define PAS2_TIM                  TIM14
#define PAS1_TIMER_CLK_ENABLE()   do{RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;}while(0)
#define PAS2_TIMER_CLK_ENABLE()   do{RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;}while(0)
#define PAS_TIMER_INPUT_CLOCK     84000000 // APB1 clock * 2
#define PAS_TIM_PSC               (8399) // 0.1ms per tick - 10kHz clock


#define THROTTLE_START_TIME			1000
#define THROTTLE_START_DEADTIME		500
#define THROTTLE_RANGE_LIMIT		0.05f
#define THROTTLE_MIN_DEFAULT		0.85f
#define THROTTLE_MAX_DEFAULT		2.20f
#define THROTTLE_HYST_LOW			0.025f
#define THROTTLE_HYST_HIGH			0.030f
#define THROTTLE_DROPOUT			0.72f

#define THROTTLE_OUTPUT_MIN   (0.00f)
#define THROTTLE_OUTPUT_MAX   (0.99f)
// Limit the throttle climb rate to 50% / second
// The update rate is 1000Hz, so the rate limit is actually .125% per update
#define THROTTLE_SLEW_RATE  (0.00125f)

#define THROTTLE_TYPE_ANALOG        0
#define THROTTLE_TYPE_PAS           1

typedef struct
{
	uint8_t state;
	uint32_t startup_count;
	float min;
	float max;
	float scale_factor;
}Throttle_Analog_Type;

typedef struct
{
  uint8_t last_reading;
  uint32_t time_counter;
  float filtered_speed;
  float scale_factor;
} Throttle_PAS_Type;

#define THROTTLE_ANALOG_DEFAULTS	{0, 0, 0.0f, 0.0f, 1.0f}
#define THROTTLE_PAS_DEFAULTS     {0, 0, 0.0f, 0.0f}
#define PAS_FILTER                (0.125f) // LPF of 1/8

// Biquad filter: Fs = 1kHz, f0 = 2Hz, Q = 0.45
// Little bit sluggish response. Maybe feels safer?
#define THROTTLE_LPF_DEFAULTS  { \
                -1.972304f, \
                0.9724600f, \
                0.00003893429f, \
                0.00007786857f, \
                0.00003893429f, \
                0.0f, \
                0.0f, \
                0.0f, \
                0.0f }


float throttle_process(float raw_voltage);

#endif /* _THROTTLE_H_ */
