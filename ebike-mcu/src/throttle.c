/*
 * throttle.c
 *
 *  Created on: Sep 17, 2016
 *      Author: David
 */

#include "throttle.h"

Biquad_Float_Type Throttle_filt=BIQ_LPF_DEFAULTS;
volatile Throttle_Type sThrottle=THROTTLE_DEFAULTS;

float throttle_process(float raw_voltage)
{
	// Filter the raw throttle voltage
	Throttle_filt.X = raw_voltage;
	dfsl_biquadf(&Throttle_filt);

	// *****STARTUP SEQUENCE*****
	// For the first Throttle Startup period, input is ignored
	// This allows the Biquad filter to stabilize
	// After this period, the value of the biquad filter output is
	// selected as the minimum throttle position.

	float temp_cmd;
	if(sThrottle.throttle_startup_count < THROTTLE_START_TIME)
	{
		// Throttle startup routine.
		// No effect for about a short duration ("Deadtime")
		// Then, average the throttle position for the remainder of the startup
		// This averaged value becomes the throttle minimum position
		sThrottle.throttle_startup_count++;
		if(sThrottle.throttle_startup_count >= THROTTLE_START_DEADTIME)
		{
			sThrottle.throttle_min = sThrottle.throttle_min + Throttle_filt.Y;
		}
		return 0.0f;
	}
	else
	{
		// Regular throttle processing
		if(sThrottle.throttle_startup_count == THROTTLE_START_TIME)
		{
			// If this is the first time, compute the average for the minimum position
			sThrottle.throttle_startup_count++;
			sThrottle.throttle_min = sThrottle.throttle_min / (THROTTLE_START_TIME - THROTTLE_START_DEADTIME);
		}
		if(Throttle_filt.Y < ( sThrottle.throttle_min - THROTTLE_MIN_HYST))
		{
			// If incoming throttle position is less than the recorded minimum,
			// Redo the startup routine
			sThrottle.throttle_startup_count = 0;
			return 0.0f;
		}
		temp_cmd = (Throttle_filt.Y - sThrottle.throttle_min) * THROTTLE_SCALE_FACTOR;
		if(sThrottle.throttle_state)
		{
			// Hysteresis. If throttle was on but passes below hysteresis level, turn it off
			if(temp_cmd <= THROTTLE_HYST_LOW)
			{
				temp_cmd = 0.0f;
				sThrottle.throttle_state = 0;
			}
		}
		else
		{
			// If throttle was off but passes above hysteresis level, turn it on
			if(temp_cmd >= THROTTLE_HYST_HIGH)
			{
				sThrottle.throttle_state = 1;
			}
			else
			{
				temp_cmd = 0.0f;
			}
		}
		return temp_cmd;
	}
}
