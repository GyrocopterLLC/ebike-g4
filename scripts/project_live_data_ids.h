
/******************************************************************************
 * Filename: project_live_data_ids.h
 ******************************************************************************

 Copyright (c) 2022 David Miller

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
 

// Auto-generated file. Do not edit!
// If changes are needed, edit the variable settings in 'datavars.xml' and
// re-run 'datavar_codegen.py' to recreate this file.

#ifndef PROJECT_LIVE_DATA_IDS_H_
#define PROJECT_LIVE_DATA_IDS_H_


// Debugging output id definitions
#define MAX_LIVE_DATA_CHOICES           (22)
#define LIVE_CHOICE_UNUSED              (0)
#define LIVE_CHOICE_IA                  (1) // Phase A current (amps)
#define LIVE_CHOICE_IB                  (2) // Phase B current (amps)
#define LIVE_CHOICE_IC                  (3) // Phase C current (amps)
#define LIVE_CHOICE_TA                  (4) // Phase A PWM duty cycle (percent)
#define LIVE_CHOICE_TB                  (5) // Phase B PWM duty cycle (percent)
#define LIVE_CHOICE_TC                  (6) // Phase C PWM duty cycle (percent)
#define LIVE_CHOICE_THROTTLE            (7) // Throttle position (percent)
#define LIVE_CHOICE_HALLANGLE           (8) // Hall sensor angle (zero to one)
#define LIVE_CHOICE_HALLSPEED           (9) // Hall sensor speed (Hz)
#define LIVE_CHOICE_HALLACCEL           (10) // Hall sensor acceleration (Hz/s)
#define LIVE_CHOICE_HALLSTATE           (11) // Hall sensor state (1 to 6)
#define LIVE_CHOICE_VBUS                (12) // Battery voltage
#define LIVE_CHOICE_ID                  (13) // D phase current (amps)
#define LIVE_CHOICE_IQ                  (14) // Q phase current (amps)
#define LIVE_CHOICE_TD                  (15) // D phase duty cycle (percent)
#define LIVE_CHOICE_TQ                  (16) // Q phase duty cycle (percent)
#define LIVE_CHOICE_VA                  (17) // Phase A voltage
#define LIVE_CHOICE_VB                  (18) // Phase B voltage
#define LIVE_CHOICE_VC                  (19) // Phase C voltage
#define LIVE_CHOICE_FTEMP               (20) // Motor controller (MOSFET) temperature (degC)
#define LIVE_CHOICE_ERRORCODE           (21) // Error code (packed binary)
#define LIVE_CHOICE_RAWTHROTTLE         (22) // Raw throttle voltage

#endif // PROJECT_LIVE_DATA_IDS_H_
