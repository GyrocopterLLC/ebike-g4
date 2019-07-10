/******************************************************************************
 * Filename: motor_loop.h
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

#ifndef MOTOR_LOOP_H_
#define MOTOR_LOOP_H_

#include "stm32f4xx.h"
#include "DavidsFOCLib.h"
#include "pwm.h"
#include "hallSensor.h"

typedef enum _Motor_RunState {
    Motor_Off,		// All switching disabled.
    Motor_SixStep,  // Running in trapezoidal mode
    Motor_Startup,	// In FOC, but with untrustworthy angle
    Motor_AtSpeed,	// Switched to FOC full control
    Motor_OpenLoop, // Open-loop with fixed frequency rotation
    Motor_Fault		// Something fd up
} Motor_RunState;

typedef struct _Motor_Controls {
    Motor_RunState state;
    float ThrottleCommand;
    float BusVoltage;
    float RampAngle;
    uint32_t speed_cycle_integrator;
} Motor_Controls;

typedef struct _Motor_Observations {
    float iA;
    float iB;
    float iC;
    float RotorAngle;
    uint8_t HallState;
} Motor_Observations;

typedef struct _Motor_PWMDuties {
    float tA;
    float tB;
    float tC;
} Motor_PWMDuties;

typedef struct _FOC_StateVariables {
    float Clarke_Alpha;
    float Clarke_Beta;
    float Park_D;
    float Park_Q;
    PID_Float_Type* Id_PID;
    PID_Float_Type* Iq_PID;
} FOC_StateVariables;

void Motor_Loop(Motor_Controls* cntl, Motor_Observations* obv,
        FOC_StateVariables* foc, Motor_PWMDuties* duty);

#endif // MOTOR_LOOP_H_
