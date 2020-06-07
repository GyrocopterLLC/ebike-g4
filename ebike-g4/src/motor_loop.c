/******************************************************************************
 * Filename: motor_loop.c
 * Description: The inner loop of the PWM calculation. These functions are
 *              called at a high rate to control the power and speed of
 *              the brushless DC motor.
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

uint8_t lastHallState = 0;
Motor_Run_State lastRunState = Motor_Off;

void Motor_Loop(Main_Variables* mvar) {
    float throttle = mvar->Ctrl->ThrottleCommand;
    FOC_StateVariables* foc = mvar->Foc;
    Motor_PWMDuties* pwm = mvar->Pwm;


    if(mvar->Ctrl->state == Motor_Sine) {
        if(lastRunState != Motor_Sine) {
            PHASE_A_PWM();
            PHASE_B_PWM();
            PHASE_C_PWM();
        }
        FOC_Ipark(0.0f, throttle, foc->Sin, foc->Cos, &(foc->Clarke_Alpha), &(foc->Clarke_Beta));
        FOC_SVM(foc->Clarke_Alpha, foc->Clarke_Beta, &(pwm->tA), &(pwm->tB), &(pwm->tC));
    }
    else if(mvar->Ctrl->state == Motor_Debug) {
        if(lastRunState != Motor_Debug) {
            PHASE_A_PWM();
            PHASE_B_PWM();
            PHASE_C_PWM();
        }
        pwm->tA = throttle;
        pwm->tB = throttle;
        pwm->tC = throttle;
    }
    else if(mvar->Ctrl->state == Motor_SixStep) {
        switch(mvar->Obv->HallState) {
        case 2: // rotor = 0 to 60
            // B+, A-
            pwm->tB = throttle;
            pwm->tA = 0.0f;
            pwm->tC = 0.5f;
            if((lastRunState != Motor_SixStep) || (lastHallState != 2)) {
                PHASE_B_PWM();
                PHASE_A_LOW();
                PHASE_C_OFF();
            }
            break;
        case 6: // rotor = 60 to 120
            // C+, A-
            pwm->tC = throttle;
            pwm->tA = 0.0f;
            pwm->tB = 0.5f;
            if((lastRunState != Motor_SixStep) || (lastHallState != 6)) {
                PHASE_C_PWM();
                PHASE_A_LOW();
                PHASE_B_OFF();
            }
            break;
        case 4: // rotor = 120 to 180
            // C+, B-
            pwm->tC = throttle;
            pwm->tB = 0.0f;
            pwm->tA = 0.5f;
            if((lastRunState != Motor_SixStep) || (lastHallState != 4)) {
                PHASE_C_PWM();
                PHASE_B_LOW();
                PHASE_A_OFF();
            }
            break;
        case 5: // rotor = 180 to 240
            // A+, B-
            pwm->tA = throttle;
            pwm->tB = 0.0f;
            pwm->tC = 0.5f;
            if((lastRunState != Motor_SixStep) || (lastHallState != 5)) {
                PHASE_A_PWM();
                PHASE_B_LOW();
                PHASE_C_OFF();
            }
            break;
        case 1: // rotor = 240 to 300
            // A+, C-
            pwm->tA = throttle;
            pwm->tC = 0.0f;
            pwm->tB = 0.5f;
            if((lastRunState != Motor_SixStep) || (lastHallState != 1)) {
                PHASE_A_PWM();
                PHASE_C_LOW();
                PHASE_B_OFF();
            }
            break;
        case 3: // rotor = 300 to 360
            // B+, C-
            pwm->tB = throttle;
            pwm->tC = 0.0f;
            pwm->tA = 0.5f;
            if((lastRunState != Motor_SixStep) || (lastHallState != 3)) {
                PHASE_B_PWM();
                PHASE_C_LOW();
                PHASE_A_OFF();
            }
            break;
        }
    }
    lastRunState = mvar->Ctrl->state;
    lastHallState = mvar->Obv->HallState;
}
