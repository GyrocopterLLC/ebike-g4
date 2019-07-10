/******************************************************************************
 * Filename: motor_loop.c
 * Description: The inner loop of the PWM calculation. These functions are
 *              called at a high rate to control the power and speed of
 *              the brushless DC motor.
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

#include "main.h"
#include "motor_loop.h"
#include "project_parameters.h"

uint8_t lastHallState = 0;
Motor_RunState lastRunState = Motor_Off;

float HallStateToDriveFloat[8] = HALL_ANGLES_TO_DRIVE_FLOAT;

static void MLoop_Turn_Off_Check(Motor_Controls* cntl) {
    if (cntl->ThrottleCommand <= 0.0f) {
        cntl->state = Motor_Off;
        cntl->speed_cycle_integrator = 0;
        cntl->ThrottleCommand = 0.0f;
        PWM_MotorOFF();
    }
}

void Motor_Loop(Motor_Controls* cntl, Motor_Observations* obv,
        FOC_StateVariables* foc, Motor_PWMDuties* duty) {
    float ipark_a, ipark_b;

    switch (cntl->state) {
    case Motor_Off:
        // There's no command to give, so we can skip all that fancy processing.
        duty->tA = 0.0f;
        duty->tB = 0.0f;
        duty->tC = 0.0f;
        dfsl_pid_resetf(foc->Id_PID);
        dfsl_pid_resetf(foc->Iq_PID);
        PWM_MotorOFF();

        break;
    case Motor_SixStep:
        if (lastRunState != Motor_SixStep) {
            PWM_MotorON();
        }
        MLoop_Turn_Off_Check(cntl);
        // Running the motor in six-step mode.
        // Current monitoring is not used in determining duty cycle, unless
        // a fault condition is met (over-current)
        // State settings:
        //		6 -> +B, -C
        //		2 -> +B, -A
        //		3 -> +C, -A
        //		1 -> +C, -B
        // 		5 -> +A, -B
        //		4 -> +A, -C
        duty->tA = cntl->ThrottleCommand;
        duty->tB = cntl->ThrottleCommand;
        duty->tC = cntl->ThrottleCommand;
        if (lastHallState != obv->HallState) {
            lastHallState = obv->HallState;

            switch (obv->HallState) {
            // Set duty cycles - only one phase is PWM'd. Throttle command is directly
            // sent as power demand (in form of PWM duty cycle)
            // Enable the "participating" phases. One phase pwm, one phase low-side on,
            // and the third phase completely turned off
            case 2:
                PHASE_B_PWM()
                ;
                PHASE_A_LOW()
                ;
                PHASE_C_OFF()
                ;
                break;
            case 6:
                PHASE_C_PWM()
                ;
                PHASE_A_LOW()
                ;
                PHASE_B_OFF()
                ;
                break;
            case 4:
                PHASE_C_PWM()
                ;
                PHASE_B_LOW()
                ;
                PHASE_A_OFF()
                ;
                break;
            case 5:
                PHASE_A_PWM()
                ;
                PHASE_B_LOW()
                ;
                PHASE_C_OFF()
                ;
                break;
            case 1:
                PHASE_A_PWM()
                ;
                PHASE_C_LOW()
                ;
                PHASE_B_OFF()
                ;
                break;
            case 3:
                PHASE_B_PWM()
                ;
                PHASE_C_LOW()
                ;
                PHASE_A_OFF()
                ;
                break;
//            case 6:
//              PHASE_A_OFF();
//              PHASE_C_LOW();
//              PHASE_B_PWM();
//              break;
//            case 2:
//              PHASE_C_OFF();
//              PHASE_A_LOW();
//              PHASE_B_PWM();
//              break;
//            case 3:
//              PHASE_B_OFF();
//              PHASE_A_LOW();
//              PHASE_C_PWM();
//              break;
//            case 1:
//              PHASE_A_OFF();
//              PHASE_B_LOW();
//              PHASE_C_PWM();
//              break;
//            case 5:
//              PHASE_C_OFF();
//              PHASE_B_LOW();
//              PHASE_A_PWM();
//              break;
//            case 4:
//              PHASE_B_OFF();
//              PHASE_C_LOW();
//              PHASE_A_PWM();
//              break;
            default:
                // Oh shit damage control
                cntl->state = Motor_Fault;
                duty->tA = 0.0f;
                duty->tB = 0.0f;
                duty->tC = 0.0f;
                PWM_MotorOFF();
                break;
            }
        }
        break;
    case Motor_Startup:
        // During this startup routine, the FOC is active but the angle
        // of the motor is discontinuous (similar to 6-step)
        if (lastRunState != Motor_Startup) {
            PHASE_A_PWM()
            ;
            PHASE_B_PWM()
            ;
            PHASE_C_PWM()
            ;
            PWM_MotorON();
        }
        MLoop_Turn_Off_Check(cntl);
        if ((obv->HallState > 6) || (obv->HallState < 1)) {
            cntl->state = Motor_Fault;
            duty->tA = 0.0f;
            duty->tB = 0.0f;
            duty->tC = 0.0f;
            PWM_MotorOFF();
        }

        // Clarke transform to 2-phase stationary system
        dfsl_clarkef(obv->iA, obv->iB, &(foc->Clarke_Alpha),
                &(foc->Clarke_Beta));
        dfsl_parkf(foc->Clarke_Alpha, foc->Clarke_Beta,
                HallStateToDriveFloat[obv->HallState], &(foc->Park_D),
                &(foc->Park_Q));
        // Pass filtered current to the PI(D)s
        foc->Id_PID->Err = 0.0f - foc->Park_D;
        foc->Iq_PID->Err = (FULLSCALE_THROTTLE) * (cntl->ThrottleCommand)
                - foc->Park_Q;
        //Id_control.Err = 0.0f - Id_Filt.Y;
        //Iq_control.Err = (3.0f)*Throttle_cmd - Iq_Filt.Y;
        // Don't integrate unless the throttle is active
        if (cntl->ThrottleCommand > 0.0f) {
            dfsl_pidf(foc->Id_PID);
            dfsl_pidf(foc->Iq_PID);
        }

        // **************** FORWARD PATH *****************
        // Feed to inverse Park
        dfsl_iparkf(foc->Id_PID->Out, foc->Iq_PID->Out,
                HallStateToDriveFloat[obv->HallState], &ipark_a, &ipark_b);
        // Saturate inputs to unit-length vector
        // Is magnitude of ipark greater than 1?
        if (((ipark_a * ipark_a) + (ipark_b * ipark_b)) > 1.0f) {
            // Trim by scaling by 1.0 / mag(ipark)
            float mag_ipark = sqrtf((ipark_a * ipark_a) + (ipark_b * ipark_b));
            ipark_a = ipark_a / mag_ipark;
            ipark_b = ipark_b / mag_ipark;
        }
        // Inverse Park outputs to space vector modulation, output three-phase waveforms
        dfsl_svmf(ipark_a, ipark_b, &(duty->tA), &(duty->tB), &(duty->tC));

        // Check if we're ready for regular run mode
        if (HallSensor_Get_Speedf() > MIN_SPEED_TO_FOC) {
            cntl->speed_cycle_integrator = cntl->speed_cycle_integrator + 1;
        } else {
            if (cntl->speed_cycle_integrator > 0) {
                cntl->speed_cycle_integrator = cntl->speed_cycle_integrator - 1;
            }
        }
        if (cntl->speed_cycle_integrator > SPEED_COUNTS_TO_FOC) {
            // Hold off on changing to FOC until the angle is pretty darn close
            // to lining up.
            if (fabsf(
                    HallStateToDriveFloat[obv->HallState]
                            - obv->RotorAngle) < FOC_SWITCH_ANGLE_EPS)
                cntl->state = Motor_AtSpeed;
        }
        break;

    case Motor_AtSpeed:
        if (lastRunState != Motor_AtSpeed) {
            PHASE_A_PWM()
            ;
            PHASE_B_PWM()
            ;
            PHASE_C_PWM()
            ;
            PWM_MotorON();
//	    dfsl_pid_resetf(foc->Id_PID);
//	    dfsl_pid_resetf(foc->Iq_PID);
        }
        MLoop_Turn_Off_Check(cntl);
        // Running full-fledged FOC algorithm now!
        // **************** FEEDBACK PATH *****************
        // Transform sensor readings
        dfsl_clarkef(obv->iA, obv->iB, &(foc->Clarke_Alpha),
                &(foc->Clarke_Beta));
        dfsl_parkf(foc->Clarke_Alpha, foc->Clarke_Beta, obv->RotorAngle,
                &(foc->Park_D), &(foc->Park_Q));
        // Input feedbacks to the Id and Iq controllers
        // Filter the currents
        /*
         Id_Filt.X = foc->Park_D;
         Iq_Filt.X = foc->Park_Q;
         dfsl_biquadf(&Id_Filt);
         dfsl_biquadf(&Iq_Filt);
         */
        // Pass current to the PI(D)s
        foc->Id_PID->Err = 0.0f - foc->Park_D;
        foc->Iq_PID->Err = (FULLSCALE_THROTTLE) * (cntl->ThrottleCommand)
                - foc->Park_Q;
        //Id_control.Err = 0.0f - Id_Filt.Y;
        //Iq_control.Err = (3.0f)*Throttle_cmd - Iq_Filt.Y;
        // Don't integrate unless the throttle is active
        if (cntl->ThrottleCommand > 0.0f) {
            dfsl_pidf(foc->Id_PID);
            dfsl_pidf(foc->Iq_PID);
        }

        // **************** FORWARD PATH *****************
        // Feed to inverse Park
        dfsl_iparkf(foc->Id_PID->Out, foc->Iq_PID->Out, obv->RotorAngle,
                &ipark_a, &ipark_b);
        //dfsl_iparkf(0, cntl->ThrottleCommand, obv->RotorAngle, &ipark_a, &ipark_b);
        // Saturate inputs to unit-length vector
        // Is magnitude of ipark greater than 1?
        if (((ipark_a * ipark_a) + (ipark_b * ipark_b)) > 1.0f) {
            // Trim by scaling by 1.0 / mag(ipark)
            float mag_ipark = sqrtf((ipark_a * ipark_a) + (ipark_b * ipark_b));
            ipark_a = ipark_a / mag_ipark;
            ipark_b = ipark_b / mag_ipark;
        }
        // Inverse Park outputs to space vector modulation, output three-phase waveforms
        dfsl_svmf(ipark_a, ipark_b, &(duty->tA), &(duty->tB), &(duty->tC));
        break;

    case Motor_OpenLoop:
        // Current control is active, but only on the D-phase.
        // The forced ramp angle is used instead of the actual motor angle,
        // which means that the motor is locked to a fixed rotational frequency.
        if (lastRunState != Motor_OpenLoop) {
            PHASE_A_PWM()
            ;
            PHASE_B_PWM()
            ;
            PHASE_C_PWM()
            ;
            PWM_MotorON();
            // Resetting the PID means the motor is gonna jump a little bit
            dfsl_pid_resetf(foc->Id_PID);
            dfsl_pid_resetf(foc->Iq_PID);
        }
        MLoop_Turn_Off_Check(cntl);
        // **************** FEEDBACK PATH *****************
        // Transform sensor readings
        dfsl_clarkef(obv->iA, obv->iB, &(foc->Clarke_Alpha),
                &(foc->Clarke_Beta));
        dfsl_parkf(foc->Clarke_Alpha, foc->Clarke_Beta, cntl->RampAngle,
                &(foc->Park_D), &(foc->Park_Q));
        // Input feedbacks to the Id and Iq controllers
        // Pass current to the PI(D)s
        foc->Id_PID->Err = (FULLSCALE_THROTTLE) * (cntl->ThrottleCommand)
                - foc->Park_D;
        foc->Iq_PID->Err = 0.0f - foc->Park_Q;
        //Id_control.Err = 0.0f - Id_Filt.Y;
        //Iq_control.Err = (3.0f)*Throttle_cmd - Iq_Filt.Y;
        // Don't integrate unless the throttle is active
        if (cntl->ThrottleCommand > 0.0f) {
            dfsl_pidf(foc->Id_PID);
            dfsl_pidf(foc->Iq_PID);
        }

        // **************** FORWARD PATH *****************
        // Feed to inverse Park
        dfsl_iparkf(foc->Id_PID->Out, foc->Iq_PID->Out, cntl->RampAngle,
                &ipark_a, &ipark_b);
        //dfsl_iparkf(0, cntl->ThrottleCommand, obv->RotorAngle, &ipark_a, &ipark_b);
        // Inverse Park outputs to space vector modulation, output three-phase waveforms
        // Saturate inputs to unit-length vector
        // Is magnitude of ipark greater than 1?
        if (((ipark_a * ipark_a) + (ipark_b * ipark_b)) > 1.0f) {
            // Trim by scaling by 1.0 / mag(ipark)
            float mag_ipark = sqrtf((ipark_a * ipark_a) + (ipark_b * ipark_b));
            ipark_a = ipark_a / mag_ipark;
            ipark_b = ipark_b / mag_ipark;
        }
        dfsl_svmf(ipark_a, ipark_b, &(duty->tA), &(duty->tB), &(duty->tC));
        break;
    case Motor_Fault:
        PWM_MotorOFF();
        duty->tA = 0.0f;
        duty->tB = 0.0f;
        duty->tC = 0.0f;
        break;
    }
    lastRunState = cntl->state;
}
