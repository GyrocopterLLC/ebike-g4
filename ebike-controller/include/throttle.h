/******************************************************************************
 * Filename: throttle.h
 ******************************************************************************

 Copyright (c) 2019 David Miller

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

#ifndef _THROTTLE_H_
#define _THROTTLE_H_

#include "stm32f4xx.h"
#include "DavidsFOCLib.h"
#include "project_parameters.h"
#include "pinconfig.h"
#include "periphconfig.h"

#define PAS_TIMER_INPUT_CLOCK     84000000 // APB1 clock * 2
#define PAS_TIM_PSC               8399 // 0.1ms per tick - 10kHz clock
#define PAS_TIM_ARR               9999 // Reset at 1 second (0->9999)
#define PAS_CLK                   10000

#define THROTTLE_TYPE_NONE        0
#define THROTTLE_TYPE_ANALOG      1
#define THROTTLE_TYPE_PAS         2

typedef struct _throttle_type {
    uint8_t throttle_type;
    uint8_t state;
    float throttle_command;
    float prev_output;
    float hyst;
    float filt;
    float rise;
} Throttle_Type;

typedef struct _throttle_analog {
    uint16_t flags;
    uint16_t startup_counter;
    float min;
    float max;
    float scale_factor;
} Throttle_Analog_Type;

// Definition of flags for analog throttle
#define THR_FLAG_STARTUP_COMPLETE       (0x0001)
// Duration of time (in throttle ticks - usually 1ms) to wait for throttle to steady out
#define THR_STARTUP_TIMER_DURATION      (300)

typedef struct _throttle_pas {
    float filtered_speed;
    float scale_factor;
} Throttle_PAS_Type;

#define THROTTLE_DEFAULTS         {THROTTLE_TYPE_ANALOG, 0, 0.0f, 0.0f, \
                                    THROTTLE_HYST_DEFAULT, THROTTLE_FILT_DEFAULT, \
                                    THROTTLE_RISE_DEFAULT}
#define THROTTLE_ANALOG_DEFAULTS	{0, 0, 0.0f, 0.0f, 1.0f}
#define THROTTLE_PAS_DEFAULTS     {0.0f, 0.015f}
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

void throttle_init(void);
void throttle_switch_type(uint8_t thrnum, uint8_t thrtype);
void throttle_process(uint8_t thrnum);
void throttle_pas_process(uint8_t thrnum);
void throttle_pas_timer_overflow(uint8_t thrnum);
float throttle_get_command(uint8_t thrnum);

uint8_t throttle_set_type(uint8_t thrnum, uint8_t thrtype);
uint8_t throttle_get_type(uint8_t thrnum);
uint8_t throttle_set_min(uint8_t thrnum, float thrmin);
float throttle_get_min(uint8_t thrnum);
uint8_t throttle_set_max(uint8_t thrnum, float thrmax);
float throttle_get_max(uint8_t thrnum);
uint8_t throttle_set_hyst(uint8_t thrnum, float thrhyst);
float throttle_get_hyst(uint8_t thrnum);
uint8_t throttle_set_filt(uint8_t thrnum, float thrfilt);
float throttle_get_filt(uint8_t thrnum);
uint8_t throttle_set_rise(uint8_t thrnum, float thrrise);
float throttle_get_rise(uint8_t thrnum);

void throttle_save_variables(void);
void throttle_load_variables(void);

#endif /* _THROTTLE_H_ */

