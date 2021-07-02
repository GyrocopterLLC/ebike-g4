/******************************************************************************
 * Filename: ram_commands.h
 ******************************************************************************

 Copyright (c) 2021 David Miller

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

#ifndef __RAM_COMMANDS_H
#define __RAM_COMMANDS_H

#include "main.h"

//**************  ADC Commands  **************

const Ram_Command_Type ADC_Rshunt = {CONFIG_ADC_RSHUNT,
                                    Data_Type_Float,
                                    1,
                                    uiADC_GetRShunt,
                                    uiADC_SetRShunt};
const Ram_Command_Type ADC_VbusRatio = {CONFIG_ADC_VBUS_RATIO,
                                Data_Type_Float,
                                1,
                                uiADC_GetVbusRatio,
                                uiADC_SetVbusRatio};
const Ram_Command_Type ADC_VphaseRatio = {CONFIG_ADC_VPHASE_RATIO,
                                Data_Type_Float,
                                1,
                                uiADC_GetVphaseRatio,
                                uiADC_SetVphaseRatio};
const Ram_Command_Type ADC_ThermFixedR = {CONFIG_ADC_THERM_FIXED_R,
                                Data_Type_Float,
                                1,
                                uiADC_GetThermFixedR,
                                uiADC_SetThermFixedR};
const Ram_Command_Type ADC_ThermR25 = {CONFIG_ADC_THERM_R25,
                                Data_Type_Float,
                                1,
                                uiADC_GetThermR25,
                                uiADC_SetThermR25};
const Ram_Command_Type ADC_ThermB = {CONFIG_ADC_THERM_B,
                                Data_Type_Float,
                                1,
                                uiADC_GetThermBeta,
                                uiADC_SetThermBeta};

#if 0

//**************  PWM Commands  **************
const Ram_Command_Type FOC_Kp = {CONFIG_FOC_KP,
                                Data_Type_Float,
                                1,
                                uiFOC_GetKp,
                                uiFOC_SetKp};
const Ram_Command_Type FOC_Ki = {CONFIG_FOC_KI,
                                Data_Type_Float,
                                1,
                                uiFOC_GetKi,
                                uiFOC_SetKi};
const Ram_Command_Type FOC_Kd = {CONFIG_FOC_KD,
                                Data_Type_Float,
                                1,
                                uiFOC_GetKd,
                                uiFOC_SetKd};
const Ram_Command_Type FOC_Kc = {CONFIG_FOC_KC,
                                Data_Type_Float,
                                1,
                                uiFOC_GetKc,
                                uiFOC_SetKc};
const Ram_Command_Type FOC_PwmFreq = {CONFIG_FOC_PWM_FREQ,
                                Data_Type_Int32,
                                1,
                                uiFOC_GetPwmFreq,
                                uiFOC_SetPwmFreq};
const Ram_Command_Type FOC_Kc = {CONFIG_FOC_PWM_DEADTIME,
                                Data_Type_Int32,
                                1,
                                uiFOC_GetPwmDeadtime,
                                uiFOC_SetPwmDeadtime};

//**************  MAIN Commands  **************
const Ram_Command_Type MAIN_CountsToFoc = {CONFIG_MAIN_COUNTS_TO_FOC,
                               Data_Type_Int32,
                               1,
                               uiMAIN_GetCountsToFoc,
                               uiMAIN_SetCountsToFoc};
const Ram_Command_Type MAIN_SpeedToFoc = {CONFIG_MAIN_SPEED_TO_FOC,
                               Data_Type_Float,
                               1,
                               uiMAIN_GetSpeedToFoc,
                               uiMAIN_SetSpeedToFoc};
const Ram_Command_Type MAIN_SwitchEps = {CONFIG_MAIN_SWITCH_EPS,
                               Data_Type_Float,
                               1,
                               uiMAIN_GetSwitchEps,
                               uiMAIN_SetSwitchEps};
const Ram_Command_Type MAIN_NumUsbOutputs = {CONFIG_MAIN_NUM_USB_OUTPUTS,
                               Data_Type_Int16,
                               0,
                               uiMAIN_GetNumUsbOutputs,
                               uiMAIN_SetNumUsbOutputs};
const Ram_Command_Type MAIN_UsbSpeed = {CONFIG_MAIN_USB_SPEED,
                               Data_Type_Int16,
                               0,
                               uiMAIN_GetUsbSpeed,
                               uiMAIN_SetUsbSpeed};
const Ram_Command_Type MAIN_UsbChoiceFirst = {CONFIG_MAIN_USB_CHOICE_1ST,
                               Data_Type_Int16,
                               0,
                               uiMAIN_GetUsbChoiceFirst,
                               uiMAIN_SetUsbChoiceFirst};
const Ram_Command_Type MAIN_UsbChoiceNext = {CONFIG_MAIN_USB_CHOICE_NEXT,
                               Data_Type_Int16,
                               0,
                               uiMAIN_GetUsbChoiceNext,
                               uiMAIN_SetUsbChoiceNext};

//**************  Throttle Commands  **************
const Ram_Command_Type THRT_Min = {CONFIG_THRT_MIN,
                               Data_Type_Float,
                               1,
                               uiTHRT_GetMin,
                               uiTHRT_SetMin};
const Ram_Command_Type THRT_Max = {CONFIG_THRT_MAX,
                               Data_Type_Float,
                               1,
                               uiTHRT_GetMax,
                               uiTHRT_SetMax};
const Ram_Command_Type THRT_Hyst = {CONFIG_THRT_HYST,
                               Data_Type_Float,
                               1,
                               uiTHRT_GetHyst,
                               uiTHRT_SetHyst};
const Ram_Command_Type THRT_Filt = {CONFIG_THRT_FILT,
                               Data_Type_Float,
                               1,
                               uiTHRT_GetFilt,
                               uiTHRT_SetFilt};
const Ram_Command_Type THRT_Rise = {CONFIG_THRT_RISE,
                               Data_Type_Float,
                               1,
                               uiTHRT_GetRise,
                               uiTHRT_SetRise};
const Ram_Command_Type THRT_Ratio = {CONFIG_THRT_RATIO,
                               Data_Type_Float,
                               1,
                               uiTHRT_GetRatio,
                               uiTHRT_SetRatio};

//**************  Limit Commands  **************
const Ram_Command_Type LMT_VoltFaultMin = {CONFIG_LMT_VOLT_FAULT_MIN,
                               Data_Type_Float,
                               1,
                               uiLMT_GetVoltFaultMin,
                               uiLMT_SetVoltFaultMin};
const Ram_Command_Type LMT_VoltFaultMax = {CONFIG_LMT_VOLT_FAULT_MAX,
                               Data_Type_Float,
                               1,
                               uiLMT_GetVoltFaultMax,
                               uiLMT_SetVoltFaultMax};
const Ram_Command_Type LMT_CurFaultMax = {CONFIG_LMT_CUR_FAULT_MAX,
                               Data_Type_Float,
                               1,
                               uiLMT_GetCurFaultMax,
                               uiLMT_SetCurFaultMax};
const Ram_Command_Type LMT_VoltSoftCap = {CONFIG_LMT_VOLT_SOFTCAP,
                               Data_Type_Float,
                               1,
                               uiLMT_GetVoltSoftCap,
                               uiLMT_SetVoltSoftCap};
const Ram_Command_Type LMT_VoltHardCap = {CONFIG_LMT_VOLT_HARDCAP,
                               Data_Type_Float,
                               1,
                               uiLMT_GetVoltHardCap,
                               uiLMT_SetVoltHardCap};
const Ram_Command_Type LMT_PhaseCurMax = {CONFIG_LMT_PHASE_CUR_MAX,
                               Data_Type_Float,
                               1,
                               uiLMT_GetPhaseCurMax,
                               uiLMT_SetPhaseCurMax};
const Ram_Command_Type LMT_PhaseRegenMax = {CONFIG_LMT_PHASE_REGEN_MAX,
                               Data_Type_Float,
                               1,
                               uiLMT_GetPhaseRegenMax,
                               uiLMT_SetPhaseRegenMax};
const Ram_Command_Type LMT_BattCurMax = {CONFIG_LMT_BATT_CUR_MAX,
                               Data_Type_Float,
                               1,
                               uiLMT_GetBattCurMax,
                               uiLMT_SetBattCurMax};
const Ram_Command_Type LMT_BattRegenMax = {CONFIG_LMT_BATT_REGEN_MAX,
                               Data_Type_Float,
                               1,
                               uiLMT_GetBattRegenMax,
                               uiLMT_SetBattRegenMax};
const Ram_Command_Type LMT_FetTempSoftCap = {CONFIG_LMT_FET_TEMP_SOFTCAP,
                               Data_Type_Float,
                               1,
                               uiLMT_GetFetTempSoftCap,
                               uiLMT_SetFetTempSoftCap};
const Ram_Command_Type LMT_FetTempHardCap = {CONFIG_LMT_FET_TEMP_HARDCAP,
                               Data_Type_Float,
                               1,
                               uiLMT_GetFetTempHardCap,
                               uiLMT_SetFetTempHardCap};
const Ram_Command_Type LMT_MotorTempSoftCap = {CONFIG_LMT_MOTOR_TEMP_SOFTCAP,
                               Data_Type_Float,
                               1,
                               uiLMT_GetMotorTempSoftCap,
                               uiLMT_SetMotorTempSoftCap};
const Ram_Command_Type LMT_MotorTempHardCap = {CONFIG_LMT_MOTOR_TEMP_HARDCAP,
                               Data_Type_Float,
                               1,
                               uiLMT_GetMotorTempHardCap,
                               uiLMT_SetMotorTempHardCap};

//**************  Motor Commands  **************
const Ram_Command_Type MOTOR_Hall1 = {CONFIG_MOTOR_HALL1,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetHall1,
                               uiMOTOR_SetHall1};
const Ram_Command_Type MOTOR_Hall2 = {CONFIG_MOTOR_HALL2,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetHall2,
                               uiMOTOR_SetHall2};
const Ram_Command_Type MOTOR_Hall3 = {CONFIG_MOTOR_HALL3,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetHall3,
                               uiMOTOR_SetHall3};
const Ram_Command_Type MOTOR_Hall4 = {CONFIG_MOTOR_HALL4,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetHall4,
                               uiMOTOR_SetHall4};
const Ram_Command_Type MOTOR_Hall5 = {CONFIG_MOTOR_HALL5,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetHall5,
                               uiMOTOR_SetHall5};
const Ram_Command_Type MOTOR_Hall6 = {CONFIG_MOTOR_HALL6,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetHall6,
                               uiMOTOR_SetHall6};
const Ram_Command_Type MOTOR_PolePairs = {CONFIG_MOTOR_POLEPAIRS,
                               Data_Type_Int32,
                               1,
                               uiMOTOR_GetPolePairs,
                               uiMOTOR_SetPolePairs};
const Ram_Command_Type MOTOR_GearRatio = {CONFIG_MOTOR_GEAR_RATIO,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetGearRatio,
                               uiMOTOR_SetGearRatio};
const Ram_Command_Type MOTOR_WheelSize = {CONFIG_MOTOR_WHEEL_SIZE,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetWheelSize,
                               uiMOTOR_SetWheelSize};
const Ram_Command_Type MOTOR_Kv = {CONFIG_MOTOR_KV,
                               Data_Type_Float,
                               1,
                               uiMOTOR_GetKv,
                               uiMOTOR_SetKv};

//**************  DRV8353 Commands  **************
const Ram_Command_Type DRV_GateStrength = {CONFIG_DRV_GATE_STRENGTH,
                               Data_Type_Int32,
                               1,
                               uiDRV_GetGateStrength,
                               uiDRV_SetGateStrength};
const Ram_Command_Type DRV_VdsLimit = {CONFIG_DRV_VDS_LIMIT,
                               Data_Type_Int8,
                               1,
                               uiDRV_GetVdsLimit,
                               uiDRV_SetVdsLimit};
const Ram_Command_Type DRV_CsaGain = {CONFIG_DRV_CSA_GAIN,
                               Data_Type_Int8,
                               1,
                               uiDRV_GetCsaGain,
                               uiDRV_SetCsaGain};

const Ram_Command_Type* const Ram_Commands[TOTAL_EE_VARS] = {
        &ADC_Rshunt,
        &ADC_VbusRatio,
        &ADC_VphaseRatio,
        &ADC_ThermFixedR,
        &ADC_ThermR25,
        &ADC_ThermB
};

#endif


#endif
