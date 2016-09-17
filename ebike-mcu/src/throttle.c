/*
 * throttle.c
 *
 *  Created on: Sep 17, 2016
 *      Author: David
 */

#include "throttle.h"

uint8_t throttle_prev_on = 0;
uint32_t throttle_startup_count = 0;
float throttle_min = 0.0f;

float throttle_process(float raw_voltage)
{
	float temp_cmd;
	if(throttle_startup_count < THROTTLE_START_TIME)
	{
		// Throttle startup routine.
		// No effect for about a short duration ("Deadtime")
		// Then, average the throttle position for the remainder of the startup
		// This averaged value becomes the throttle minimum position
		throttle_startup_count++;
		if(throttle_startup_count >= THROTTLE_START_DEADTIME)
		{
			throttle_min = throttle_min + raw_voltage;
		}
		return 0.0f;
	}
	else
	{
		// Regular throttle processing
		if(throttle_startup_count == THROTTLE_START_TIME)
		{
			// If this is the first time, compute the average for the minimum position
			throttle_startup_count++;
			throttle_min = throttle_min / (THROTTLE_START_TIME - THROTTLE_START_DEADTIME);
		}
		if(raw_voltage < ( throttle_min - THROTTLE_MIN_HYST))
		{
			// If incoming throttle position is less than the recorded minimum,
			// Redo the startup routine
			throttle_startup_count = 0;
			return 0.0f;
		}
		temp_cmd = (raw_voltage - throttle_min) * THROTTLE_SCALE_FACTOR;
		if(throttle_prev_on)
		{
			// Hysteresis. If throttle was on but passes below hysteresis level, turn it off
			if(temp_cmd <= THROTTLE_HYST_LOW)
			{
				temp_cmd = 0.0f;
				throttle_prev_on = 0;
			}
		}
		else
		{
			// If throttle was off but passes above hysteresis level, turn it on
			if(temp_cmd >= THROTTLE_HYST_HIGH)
			{
				throttle_prev_on = 1;
			}
			else
			{
				temp_cmd = 0.0f;
			}
		}
		return temp_cmd;
	}
}
