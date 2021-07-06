/******************************************************************************
 * Filename: throttle.c
 * Description: Interfaces with an analog throttle (e.g. a thumb throttle).
 ******************************************************************************

 Copyright (c) 2020 David Miller

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

#include "main.h"

Biquad_Type Throttle_filt = THROTTLE_LPF_DEFAULTS;
Throttle_Type sThrottle = THROTTLE_DEFAULTS;
Throttle_Analog_Type sAnalogThrottle = THROTTLE_ANALOG_DEFAULTS;

static void THROTTLE_HystAndRateLimiting(Throttle_Type *thr);
static void THROTTLE_SetScale(Throttle_Analog_Type* thr);
static uint8_t THROTTLE_DetectStuck(Throttle_Analog_Type* thr, float thr_in);

void THROTTLE_Init(void) {
    // Reads from EEPROM and initializes throttle settings
    // This function can also be used to refresh the RAM settings from the EEPROM
    THROTTLE_LoadVariables();

}

void THROTTLE_Process(void) {
    float temp_cmd = 0.0f;
    sThrottle.raw_voltage = ADC_GetThrottle();
    sThrottle.raw_voltage *= sThrottle.resistor_ratio; // The value at the throttle connector, not at the MCU pin
    // Perform the startup check
    if(THROTTLE_DetectStuck(&sAnalogThrottle, sThrottle.raw_voltage)) {
        // Throttle is okay. Or at least it started up okay.
        Throttle_filt.X = sThrottle.raw_voltage;
    } else {
        // Throttle might be stuck on
        // Set input to zero
        Throttle_filt.X = 0.0f;
    }

    // Filter the raw throttle voltage
    FOC_BiquadCalc(&Throttle_filt);

    // Regular throttle processing
    temp_cmd = (Throttle_filt.Y
            - sAnalogThrottle.min)
            * (sAnalogThrottle.scale_factor);
    // Clip at 0% and 100%
    if (temp_cmd < THROTTLE_OUTPUT_MIN)
        temp_cmd = THROTTLE_OUTPUT_MIN;
    if (temp_cmd > THROTTLE_OUTPUT_MAX)
        temp_cmd = THROTTLE_OUTPUT_MAX;
    sThrottle.throttle_command = temp_cmd;
    THROTTLE_HystAndRateLimiting(&sThrottle);
}

static void THROTTLE_HystAndRateLimiting(Throttle_Type *thr) {
    if (thr->state) {
        // Hysteresis. If throttle was on but passes below hysteresis level, turn it off
        if (thr->throttle_command <= (thr->hyst)) {
            thr->throttle_command = 0.0f;
            thr->state = 0;
        }
    } else {
        // If throttle was off but passes above hysteresis level, turn it on
        if (thr->throttle_command >= (2.0f * (thr->hyst))) {
            thr->state = 1;
        } else {
            thr->throttle_command = 0.0f;
        }
    }
    // Rate limit (upward only! no limit on how fast the throttle can fall)
    float delta = thr->throttle_command - thr->prev_output;
    if (delta > thr->rise) {
        thr->throttle_command = thr->prev_output + thr->rise;
    }
    thr->prev_output = thr->throttle_command;
}

static void THROTTLE_SetScale(Throttle_Analog_Type* thr) {
    thr->scale_factor = (1.0f) / ((thr->max) - (thr->min));
}

/**
 * @brief  Determines if the throttle position is stuck at startup.
 *
 * If the throttle becomes disconnected, the motor controller might
 * instead see a fixed throttle command and start ramping up the motor
 * uncontrollably. This is difficult to detect while running,
 * but at startup we can check for it easily.
 * When the throttle module starts up, check if the throttle position
 * is above the minimum value. If it is, wait for it to go below the
 * minimum cutoff before actually applying any throttle.
 * The throttle needs to stay below the minimum for some duration
 * before we trust it. If it passes above the minimum before the
 * end of that duration, we reset the counter.
 *
 * @param  thr - Analog throttle structure containing the throttle to
 *               validate.
 * @param  thr_in - float that holds the present analog value of the
 *                  throttle command.
 * @retval uint8_t - If throttle should be zeroed due to a stuck throttle,
 *                   returns 0. Otherwise returns 1.
 */
static uint8_t THROTTLE_DetectStuck(Throttle_Analog_Type* thr, float thr_in) {
    // Check startup flags. If we've already finished startup, don't perform
    // any checks.
    if((thr->flags & (THR_FLAG_STARTUP_COMPLETE)) == 0) {
        if(thr_in > (thr->min)) {
            // We are stuck on. Need to wait for throttle to drop below min for
            // some duration.
            // Reset the counter
            thr->startup_counter = 0;

        } else {
            // Increment the timer
            thr->startup_counter = thr->startup_counter + 1;
            if(thr->startup_counter >= THR_STARTUP_TIMER_DURATION) {
                thr->flags |= THR_FLAG_STARTUP_COMPLETE;
                return 1;
            }
        }
        return 0;
    }
    return 1;
}

float THROTTLE_GetCommand(void) {
    return sThrottle.throttle_command;
}

float THROTTLE_GetRaw(void) {
    return sThrottle.raw_voltage;
}

/**** Interfacing with UI ****/
uint8_t THROTTLE_SetMin(float thrmin) {
    if (thrmin >= 0.0f) {
        sAnalogThrottle.min = thrmin;
        THROTTLE_SetScale(&sAnalogThrottle);
        return RETVAL_OK;
    }
    return RETVAL_FAIL;
}

float THROTTLE_GetMin(void) {
    return sAnalogThrottle.min;

}

uint8_t THROTTLE_SetMax(float thrmax) {
    if (thrmax >= 0.0f) {
        sAnalogThrottle.max = thrmax;
        THROTTLE_SetScale(&sAnalogThrottle);
        return RETVAL_OK;
    }
    return RETVAL_FAIL;
}

float THROTTLE_GetMax(void) {
    return sAnalogThrottle.max;
}

uint8_t THROTTLE_SetHyst(float thrhyst) {
        if ((thrhyst >= THROTTLE_HYST_MIN) && (thrhyst < THROTTLE_HYST_MAX)) {
            sThrottle.hyst = thrhyst;
            return RETVAL_OK;
        }
    return RETVAL_FAIL;
}

float THROTTLE_GetHyst(void) {
    return sThrottle.hyst;
}
uint8_t THROTTLE_SetFilt(float thrfilt) {
    if ((thrfilt >= THROTTLE_FILT_MIN) && (thrfilt < THROTTLE_FILT_MAX)) {
        sThrottle.filt = thrfilt;
        // And do the filter calculation
        FOC_BiquadLPF(&Throttle_filt,
                THROTTLE_SAMPLING_RATE, thrfilt,
                THROTTLE_FILT_Q_DEFAULT);
        return RETVAL_OK;
    }

    return RETVAL_FAIL;
}

float THROTTLE_GetFilt(void) {
    return sThrottle.filt;
}

uint8_t THROTTLE_SetRise(float thrrise) {
    if ((thrrise < THROTTLE_RISE_MAX) && (thrrise >= THROTTLE_RISE_MIN)) {
        sThrottle.rise = thrrise;
        return RETVAL_OK;
    }

    return RETVAL_FAIL;
}

float THROTTLE_GetRise(void) {
    return sThrottle.rise;
}


uint8_t THROTTLE_SetRatio(float thrratio) {
    if((thrratio < THROTTLE_RATIO_MAX) && (thrratio >= THROTTLE_RATIO_MIN)) {
        sThrottle.resistor_ratio = thrratio;
        return RETVAL_OK;
    }
    return RETVAL_FAIL;
}

float THROTTLE_GetRatio(void) {
    return sThrottle.resistor_ratio;
}

void THROTTLE_SaveVariables(void) {
    EE_SaveFloat(CONFIG_THRT_MIN, sAnalogThrottle.min);
    EE_SaveFloat(CONFIG_THRT_MAX, sAnalogThrottle.max);
    EE_SaveFloat(CONFIG_THRT_HYST, sThrottle.hyst);
    EE_SaveFloat(CONFIG_THRT_RISE, sThrottle.rise);
    EE_SaveFloat(CONFIG_THRT_FILT, sThrottle.filt);
    EE_SaveFloat(CONFIG_THRT_RATIO, sThrottle.resistor_ratio);
}

void THROTTLE_LoadVariables(void) {

    sThrottle.hyst = EE_ReadFloatWithDefault(CONFIG_THRT_HYST,
            DFLT_THRT_HYST);
    sThrottle.filt = EE_ReadFloatWithDefault(CONFIG_THRT_FILT,
            DFLT_THRT_FILT);
    FOC_BiquadLPF(&Throttle_filt, THROTTLE_SAMPLING_RATE,
            sThrottle.filt,
            THROTTLE_FILT_Q_DEFAULT);
    sThrottle.rise = EE_ReadFloatWithDefault(CONFIG_THRT_RISE,
            DFLT_THRT_RISE);
    sAnalogThrottle.min = EE_ReadFloatWithDefault(CONFIG_THRT_MIN,
            DFLT_THRT_MIN);
    sAnalogThrottle.max = EE_ReadFloatWithDefault(CONFIG_THRT_MAX,
            DFLT_THRT_MAX);
    sThrottle.resistor_ratio = EE_ReadFloatWithDefault(CONFIG_THRT_RATIO,
            DFLT_THRT_RATIO);
    THROTTLE_SetScale(&sAnalogThrottle);
}

uint8_t uiTHRT_GetMin(uint8_t* valptr) {
    data_packet_pack_float(valptr, THROTTLE_GetMin());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiTHRT_SetMin(uint8_t* valptr) {
    return THROTTLE_SetMin(data_packet_extract_float(valptr));
}

uint8_t uiTHRT_GetMax(uint8_t* valptr) {
    data_packet_pack_float(valptr, THROTTLE_GetMax());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiTHRT_SetMax(uint8_t* valptr) {
    return THROTTLE_SetMax(data_packet_extract_float(valptr));
}

uint8_t uiTHRT_GetHyst(uint8_t* valptr) {
    data_packet_pack_float(valptr, THROTTLE_GetHyst());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiTHRT_SetHyst(uint8_t* valptr) {
    return THROTTLE_SetHyst(data_packet_extract_float(valptr));
}

uint8_t uiTHRT_GetFilt(uint8_t* valptr) {
    data_packet_pack_float(valptr, THROTTLE_GetFilt());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiTHRT_SetFilt(uint8_t* valptr) {
    return THROTTLE_SetFilt(data_packet_extract_float(valptr));
}

uint8_t uiTHRT_GetRise(uint8_t* valptr) {
    data_packet_pack_float(valptr, THROTTLE_GetRise());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiTHRT_SetRise(uint8_t* valptr) {
    return THROTTLE_SetRise(data_packet_extract_float(valptr));
}

uint8_t uiTHRT_GetRatio(uint8_t* valptr) {
    data_packet_pack_float(valptr, THROTTLE_GetRatio());
    return DATA_PACKET_SUCCESS;
}

uint8_t uiTHRT_SetRatio(uint8_t* valptr) {
    return THROTTLE_SetRatio(data_packet_extract_float(valptr));
}

