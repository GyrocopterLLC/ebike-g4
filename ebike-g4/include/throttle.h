/******************************************************************************
 * Filename: throttle.h
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

#ifndef _THROTTLE_H_
#define _THROTTLE_H_

typedef struct _throttle_type {
    uint8_t state;
    float raw_voltage;
    float throttle_command;
    float prev_output;
    float hyst;
    float filt;
    float rise;
    float resistor_ratio;
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

#define THROTTLE_DEFAULTS         { 0, 0.0f, 0.0f, 0.0f, \
                                    DFLT_THRT_HYST, DFLT_THRT_FILT, \
                                    DFLT_THRT_RISE, DFLT_THRT_RATIO}
#define THROTTLE_ANALOG_DEFAULTS	{0, 0, 0.0f, 0.0f, 1.0f}
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

#define THROTTLE_OUTPUT_MIN         (0.00f)
#define THROTTLE_OUTPUT_MAX         (0.99f)
#define THROTTLE_HYST_MIN           (0.005f)
#define THROTTLE_HYST_MAX           (0.1f)
#define THROTTLE_FILT_MIN           (0.1f)
#define THROTTLE_FILT_MAX           (499.9f)
#define THROTTLE_FILT_Q_DEFAULT     (0.707f)
#define THROTTLE_SAMPLING_RATE      (1000.0f)
#define THROTTLE_RISE_MIN           (0.00005f) // Minimum of 5% / sec
#define THROTTLE_RISE_MAX           (0.01f) // Maximum of 1000% / sec
#define THROTTLE_RATIO_MIN          (1.0f) // Minimum of 1.0 - a resistor divider cannot increase the voltage
#define THROTTLE_RATIO_MAX          (6.0f) // I don't know what kind of crazy throttle needs to have range up to 20V, but whatever.

void THROTTLE_Init(void);
void THROTTLE_Process(void);
float THROTTLE_GetCommand(void);
float THROTTLE_GetRaw(void);

uint8_t THROTTLE_SetMin(float thrmin);
float THROTTLE_GetMin(void);
uint8_t THROTTLE_SetMax(float thrmax);
float THROTTLE_GetMax(void);
uint8_t THROTTLE_SetHyst(float thrhyst);
float THROTTLE_GetHyst(void);
uint8_t THROTTLE_SetFilt(float thrfilt);
float THROTTLE_GetFilt(void);
uint8_t THROTTLE_SetRise(float thrrise);
float THROTTLE_GetRise(void);
uint8_t THROTTLE_SetRatio(float thrratio);
float THROTTLE_GetRatio(void);

void THROTTLE_SaveVariables(void);
void THROTTLE_LoadVariables(void);

#endif /* _THROTTLE_H_ */

