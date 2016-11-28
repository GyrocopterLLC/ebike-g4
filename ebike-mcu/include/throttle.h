#ifndef _THROTTLE_H_
#define _THROTTLE_H_

#include "stm32f4xx_hal.h"
#include "DavidsFOCLib.h"
#include "adc.h"

#define THROTTLE_START_TIME			1000
#define THROTTLE_START_DEADTIME		500
#define THROTTLE_RANGE_LIMIT		0.05f
#define THROTTLE_MIN_DEFAULT		0.85f
#define THROTTLE_MAX_DEFAULT		2.20f
#define THROTTLE_HYST_LOW			0.03f
#define THROTTLE_HYST_HIGH			0.08f
#define THROTTLE_DROPOUT			0.72f

typedef struct
{
	uint8_t throttle_state;
	uint32_t throttle_startup_count;
	float throttle_min;
	float throttle_max;
	float throttle_scale_factor;
}Throttle_Type;
#define THROTTLE_DEFAULTS	{0, 0, 0.0f, 0.0f, 1.0f}

float throttle_process(float raw_voltage);

#endif /* _THROTTLE_H_ */
