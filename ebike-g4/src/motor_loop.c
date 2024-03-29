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
#include <math.h>

uint8_t lastHallState = 0;
Motor_Run_State lastRunState = Motor_Off;

void Motor_Loop(Main_Variables* mvar) {
    float throttle = mvar->Ctrl->ThrottleCommand;
    FOC_StateVariables* foc = mvar->Foc;
    Motor_PWMDuties* pwm = mvar->Pwm;
    Motor_Observations* obv = mvar->Obv;
    Motor_Controls* ctrl = mvar->Ctrl;
    Config_Main* cfg = mvar->Cfg;

    if(mvar->Ctrl->state == Motor_Off) {
        // Reset PIDs so there aren't any weird jumps at the next startup
        FOC_PIDreset(foc->Id_PID);
        FOC_PIDreset(foc->Iq_PID);
        // Set PWM duty cycles to zero
        pwm->tA = 0.0f;
        pwm->tB = 0.0f;
        pwm->tC = 0.0f;
    }
    else if(mvar->Ctrl->state == Motor_Sine) {
        if(lastRunState != Motor_Sine) {
            PHASE_A_PWM();
            PHASE_B_PWM();
            PHASE_C_PWM();
        }
        FOC_Ipark(0.0f, throttle, foc->Sin, foc->Cos, &(foc->Clarke_Alpha), &(foc->Clarke_Beta));
        FOC_SVM(foc->Clarke_Alpha, foc->Clarke_Beta, &(pwm->tA), &(pwm->tB), &(pwm->tC));
    }
    else if(mvar->Ctrl->state == Motor_FOC) {
        if(lastRunState != Motor_FOC) {
            PHASE_A_PWM();
            PHASE_B_PWM();
            PHASE_C_PWM();
        }
        // --- Determine rotational phase current (Phase D/Q) ---
        // The current with the highest on time will be ignored. The current sensors are on the low-side, so
        // when the high side is turned on fully the measured current will be incorrect.
        // But since the three currents always sum to zero (iA + iB + iC = 0), it's easy to calculate one current
        // if you have the other two.
        float mincurrentA, mincurrentB;

        if(((pwm->tA) > (pwm->tB)) && ((pwm->tA) > (pwm->tC))) {
                // Duty cycle on A is the highest, use B and C
                mincurrentA = -(obv->iB) - (obv->iC);
                mincurrentB = obv->iB;
        } else if(((pwm->tB) > (pwm->tA)) && ((pwm->tB) > (pwm->tC))) {
            // Duty cycle B is biggest, use A and C
            mincurrentA = obv->iA;
            mincurrentB = -(obv->iA) - (obv->iC);
        } else {
            // Duty cycle C is biggest, use A and B
            mincurrentA = obv->iA;
            mincurrentB = obv->iB;
        }

        // Perform 3-phase to 2-phase conversion (Clarke)
        FOC_Clarke(mincurrentA, mincurrentB, &(foc->Clarke_Alpha), &(foc->Clarke_Beta));
        // Perform stationary to rotational conversion (Park)
        FOC_Park(foc->Clarke_Alpha, foc->Clarke_Beta, foc->Sin, foc->Cos, &(foc->Park_D), &(foc->Park_Q));
        // --- Proportional-integral-derivative (PID) controller ---
        // Error signals are normalized to 1.0. This allows us to use the same
        // PID gains regardless of current scaling.
        foc->Id_PID->Err = 0.0f
                - ((foc->Park_D) * (cfg->inv_max_phase_current));
        foc->Iq_PID->Err = ctrl->ThrottleCommand
                - ((foc->Park_Q) * (cfg->inv_max_phase_current));
        // Perform the PID control loop
        FOC_PIDcalc(foc->Id_PID);
        FOC_PIDcalc(foc->Iq_PID);
        // Convert back to stationary phase (inverse Park)
        FOC_Ipark(foc->Id_PID->Out, foc->Iq_PID->Out, foc->Sin, foc->Cos, &(foc->T_Alpha), &(foc->T_Beta));
        // Saturate inputs to unit-length vector
        // Is magnitude of ipark greater than 1?
        if ((((foc->T_Alpha) * (foc->T_Alpha)) + ((foc->T_Beta) * (foc->T_Beta))) > 1.0f) {
            // Trim by scaling by 1.0 / mag(ipark)
            float inv_mag_ipark = 1.0f
                    / sqrtf(((foc->T_Alpha) * (foc->T_Alpha)) + ((foc->T_Beta) * (foc->T_Beta)));
            foc->T_Alpha = (foc->T_Alpha) * inv_mag_ipark;
            foc->T_Beta = (foc->T_Beta) * inv_mag_ipark;
        }

        FOC_SVM(foc->T_Alpha, foc->T_Beta, &(pwm->tA), &(pwm->tB), &(pwm->tC));
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
        // Trapezoidal patterns:
        // A+/C- ==> 30
        // B+/C- ==> 90
        // B+/A- ==> 150
        // C+/A- ==> 210
        // C+/B- ==> 270
        // A+/B- ==> 330
        // State 2: 0 to 60 degrees - apply B+/A- (150), leads rotor by 150-0=150 to 150-60=90
        // State 6: 60 to 120 degrees - apply C+/A- (210), leads by 210-60=150 to 210-120=90
        // State 4: 120 to 180 degrees - apply C+/B- (270), leads by 270-120=150 to 270-180=90
        // State 5: 180 to 240 degrees - apply A+/B- (330), leads by 330-180=150 to 330-240=90
        // State 1: 240 to 300 degrees - apply A+/C- (30), leads by (360+30)-240=150 to (360+30)-300=90
        // State 3: 300 to 0/360 degrees - apply B+/C- (90), leads by (360+90)-300=150 to 90-0=90
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
