/******************************************************************************
 * Filename: main_data_types.h
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


#ifndef __MAIN_DATA_TYPES_H
#define __MAIN_DATA_TYPES_H

#include "foc_lib.h"

// Variable type definitions

typedef enum _main_limit_type {
    Main_Limit_PhaseCurrent,
    Main_Limit_PhaseRegenCurrent,
    Main_Limit_BattCurrent,
    Main_Limit_BattRegenCurrent,
    Main_Limit_SoftVoltage,
    Main_Limit_HardVoltage,
    Main_Limit_SoftFetTemp,
    Main_Limit_HardFetTemp,
    Main_Limit_SoftMotorTemp,
    Main_Limit_HardMotorTemp,
    Main_Limit_MinVoltFault,
    Main_Limit_MaxVoltFault,
    Main_Limit_CurrentFault
} Main_Limit_Type;

typedef enum _main_control_methods {
    Control_None, // invalid
    Control_BLDC, // six-step (trapezoidal) control
    Control_FOC, // Field oriented control
    Control_Debug // all three PWMs simply follow the throttle
} Main_Control_Methods;

typedef enum _motor_run_state {
    Motor_Off,      // All switching disabled.
    Motor_SixStep,  // Running in trapezoidal mode
    Motor_Sine,  // Ruuning in sinewave (open loop current, continuously updating angle)
    Motor_FOC,  // Switched to FOC full control
    Motor_OpenLoop, // Open-loop with fixed frequency rotation
    Motor_Debug,    // All three PWMs follow the throttle
    Motor_Fault     // Something screwed up
} Motor_Run_State;

typedef struct _main_config {
    // ----- Settings editable by user -----
    float RampSpeed;
    uint32_t CountsToFOC;
    float SpeedToFOC;
    float SwitchEpsilon;
    uint16_t MotorPolePairs;
    float WheelSizeMM;
    float GearRatio;
    float MotorKv;
    int32_t PWMFrequency;
    int32_t PWMDeadTime;
    float MaxPhaseCurrent;
    float MaxPhaseRegenCurrent;
    float MaxBatteryCurrent;
    float MaxBatteryRegenCurrent;
    float VoltageSoftCap;
    float VoltageHardCap;
    float FetTempSoftCap;
    float FetTempHardCap;
    float MotorTempSoftCap;
    float MotorTempHardCap;
    float MinVoltFault;
    float MaxVoltFault;
    float CurrentFault;
    // ----- Generated constants -----
    float inv_max_phase_current;
    float inv_pole_pairs;
    float kv_volts_per_ehz;
    // ----- Local variables -----
    float throttle_limit_scale;
} Config_Main;

typedef struct _Motor_Controls {
    Main_Control_Methods ControlMethod;
    Motor_Run_State state;
    float ThrottleCommand;
} Motor_Controls;

typedef struct _Motor_Observations {
    float iA;
    float iB;
    float iC;
    float iAlpha;
    float iBeta;
    float vA;
    float vB;
    float vC;
    float BusVoltage;
    float RotorAngle;
    float RotorSpeed_eHz;
    float RotorAccel_eHzps;
    float FetTemperature;
    uint8_t HallState;
} Motor_Observations;

typedef struct _Motor_PWMDuties {
    float tA;
    float tB;
    float tC;
} Motor_PWMDuties;

typedef struct _Motor_Settings {
    float inv_max_phase_current;
    float kv_volts_per_ehz;
} Motor_Settings;

typedef struct _FOC_StateVariables {
    float Sin; // sin(motorangle)
    float Cos; // cos(motorangle)
    float Clarke_Alpha; // Stationary 2-phase current, aligned on 0 degrees
    float Clarke_Beta;  // Stationary 2-phase current, aligned on 90 degrees
    float Park_D; // Rotational current, aligned with rotor
    float Park_Q; // Rotational current, aligned 90 degrees ahead of rotor
    float T_Alpha; // Commanded voltage (as percent of battery), aligned on 0 degrees
    float T_Beta;  // Commanded voltage (as percent of battery), aligned on 90 degrees
    PID_Type* Id_PID; // Controller for rotor aligned current
    PID_Type* Iq_PID; // Controller for 90 degree ahead aligned current
} FOC_StateVariables;

typedef struct _main_variables_type {
    uint32_t Timestamp;
    Motor_Controls* Ctrl;
    Motor_Observations* Obv;
    Motor_PWMDuties* Pwm;
    FOC_StateVariables* Foc;
    Motor_Settings* Sett;
} Main_Variables;



#endif
