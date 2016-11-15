#ifndef _THROTTLE_H_
#define _THROTTLE_H_

#include "stm32f4xx_hal.h"
#include "DavidsFOCLib.h"

#define THROTTLE_START_TIME			1000
#define THROTTLE_START_DEADTIME		500
#define THROTTLE_MIN_HYST			0.05f
#define THROTTLE_SCALE_FACTOR		0.47619f // (2.8 - 0.7)^(-1)
#define THROTTLE_HYST_LOW			0.03f
#define THROTTLE_HYST_HIGH			0.08f

typedef struct
{
	uint8_t throttle_state;
	uint32_t throttle_startup_count;
	float throttle_min;
}Throttle_Type;
#define THROTTLE_DEFAULTS	{0, 0, 0.0f}

float throttle_process(float raw_voltage);

#endif /* _THROTTLE_H_ */
