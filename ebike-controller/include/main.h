/******************************************************************************
 * Filename: main.h
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

// Used resources:
// DAC
// TIM12

#ifndef __MAIN_H
#define __MAIN_H

/* Constants */
/*
 #define THROTTLE_MIN  (0.7f) // Less than 0.7V is zero throttle
 #define THROTTLE_MAX  (2.8f) // Above 2.8V is 100% throttle
 #define THROTTLE_SCALE  (0.47619f)
 #define THROTTLE_STARTUP_COUNT 1500 // Wait 1.5 sec for filter to stabilize
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
//#include "stm324xg_eval.h"
#include "usb.h"
#include "usb_cdc.h"
#include "gpio.h"
#include "DavidsFOCLib.h"
#include "hallSensor.h"
#include "adc.h"
#include "pwm.h"
#include "motor_loop.h"
#include "eeprom_emulation.h"
#include "throttle.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "uart.h"
//#include "ui.h"
#include "wdt.h"
#include "power_calcs.h"
#include "crc32.h"
#include "data_packet.h"
#include "data_commands.h"
#include "usb_data_comm.h"
#include "bms_data_comm.h"
#include "hbd_data_comm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#if defined(TESTING_2X) || defined(TESTING_PLL)
#define MAX_USB_VALS                (19)
#else
#define MAX_USB_VALS                (18)
#endif
#define MAX_USB_OUTPUTS             (10)
#define MAX_USB_SPEED_CHOICES       (6)
#define USB_SPEED_RELOAD_VALS       {400, 200, 100, 40, 20, 4} // 50Hz, 100Hz, 200Hz, 500Hz, 1kHz, 5kHz

typedef enum _pb_type {
    PB_RELEASED, PB_PRESSED
} PB_TypeDef;

typedef enum _control_methods {
    Control_None, // invalid
    Control_BLDC, // six-step (trapezoidal) control
    Control_FOC, // Field oriented control
    Control_Debug // all three PWMs simply follow the throttle
} Control_Methods;

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

typedef struct _main_config {
    // ----- Settings editable by user -----
    float RampSpeed;
    uint32_t CountsToFOC;
    float SpeedToFOC;
    float SwitchEpsilon;
    uint16_t Num_USB_Outputs;
    uint16_t USB_Speed;
    uint16_t USB_Choices[MAX_USB_OUTPUTS];
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
    Control_Methods ControlMethod;
    // ----- Generated constants -----
    float inv_max_phase_current;
    float inv_pole_pairs;
    float kv_volts_per_ehz;
    // ----- Local variables -----
    float throttle_limit_scale;
} Config_Main;

/* Exported constants --------------------------------------------------------*/
#define MAXLEDCOUNT         1000
#define DEBOUNCE_INTERVAL   10 // 10 milliseconds ==> 100Hz timer
#define DEBOUNCE_MAX        5 // Must get integrator up to 5 to count as "pressed"
#define MAX_RAMP_SPEED      (25.0f)
#define MIN_RAMP_SPEED      (-25.0f)

#define BOOTLOADER_RESET_FLAG 0xDEADBEEF

#define SERIAL_DUMP_RATE        (1)
#define TEMP_CONVERSION_RATE    (100)
#define BMS_CHECK_RATE          (10000) // Check every 10 seconds

#define MAIN_FAULT_OV               ((uint32_t)0x00000001)
#define MAIN_FAULT_UV               ((uint32_t)0x00000002)
#define MAIN_FAULT_OC               ((uint32_t)0x00000004)
#define MAIN_FAULT_FETTEMP          ((uint32_t)0x00000008)
#define MAIN_FAULT_MOTORTEMP        ((uint32_t)0x00000010)
#define MAIN_FAULT_HALL_STATE       ((uint32_t)0x00000020)
#define MAIN_FAULT_BMS_COMM         ((uint32_t)0x00000040)
#define MAIN_FAULT_BMS_OV           ((uint32_t)0x00000080)
#define MAIN_FAULT_BMS_UV           ((uint32_t)0x00000100)
#define MAIN_FAULT_BMS_V_MISMATCH   ((uint32_t)0x00000200)


#define MAINFLAG_SERIALDATAPRINT    ((uint32_t)0x00000001)
#define MAINFLAG_SERIALDATAON       ((uint32_t)0x00000002)
#define MAINFLAG_DUMPRECORD         ((uint32_t)0x00000004)
#define MAINFLAG_DUMPDATAPRINT      ((uint32_t)0x00000008)
#define MAINFLAG_DUMPDATAON         ((uint32_t)0x00000010)
#define MAINFLAG_CONVERTTEMPERATURE ((uint32_t)0x00000020)
#define MAINFLAG_HALLDETECTFAIL     ((uint32_t)0x00000040)
#define MAINFLAG_HALLDETECTPASS     ((uint32_t)0x00000080)
#define MAINFLAG_LASTCOMMSERIAL     ((uint32_t)0x00000100)
#define MAINFLAG_CHECKBMS           ((uint32_t)0x00000200)
#define MAINFLAG_DEBUG_SENDBMSRESET ((uint32_t)0x00010000)
#define MAINFLAG_DEBUG_SETBMSADDR   ((uint32_t)0x00020000)
#define MAINFLAG_DEBUG_ASKBMSBATTS  ((uint32_t)0x00040000)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//uint32_t itoa(char* buf, int32_t num);
void stringflip(char* buf, uint32_t len);
uint32_t _itoa(char* buf, int32_t num, uint32_t min_digits);
uint32_t _ftoa(char* buf, float num, uint32_t precision);
void SYSTICK_IRQHandler(void);
void User_BasicTIM_IRQ(void);
void User_PWMTIM_IRQ(void);

float MAIN_GetCurrentRampAngle(void);

void MAIN_DetectHallPositions(float curlimit);

uint8_t MAIN_SetUSBDebugOutput(uint8_t outputnum, uint8_t valuenum);
uint8_t MAIN_GetUSBDebugOutput(uint8_t outputnum);
uint8_t MAIN_SetNumUSBDebugOutputs(uint8_t numOutputs);
uint8_t MAIN_GetNumUSBDebugOutputs(void);
uint8_t MAIN_SetUSBDebugSpeed(uint8_t speedChoice);
uint8_t MAIN_GetUSBDebugSpeed(void);
uint8_t MAIN_SetUSBDebugging(uint8_t on_or_off);
uint8_t MAIN_GetUSBDebugging(void);
float MAIN_GetRampSpeed(void);
uint8_t MAIN_SetRampSpeed(float newspeed);
uint8_t MAIN_SetVar(uint8_t var, float newval);
float MAIN_GetVar(uint8_t var);
float MAIN_GetVar_EEPROM(uint8_t var);
uint8_t MAIN_SetFreq(int32_t newfreq);
int32_t MAIN_GetFreq(void);
uint8_t MAIN_SetDeadTime(int32_t newDT);
int32_t MAIN_GetDeadTime(void);
uint8_t MAIN_RequestBLDC(void);
uint8_t MAIN_RequestFOC(void);
uint8_t MAIN_EnableDebugPWM(void);
uint8_t MAIN_DisableDebugPWM(void);
uint8_t MAIN_SetCountsToFOC(uint32_t new_counts);
uint32_t MAIN_GetCountsToFOC(void);
uint8_t MAIN_SetSpeedToFOC(float new_speed);
float MAIN_GetSpeedToFOC(void);
uint8_t MAIN_SetSwitchoverEpsilon(float new_eps);
float MAIN_GetSwitchoverEpsilon(void);
void MAIN_SetError(uint32_t errorCode);
void MAIN_SoftReset(uint8_t restartInBootloader);
uint8_t MAIN_GetDashboardData(uint8_t* dataBuffer);
uint8_t MAIN_SetLimit(Main_Limit_Type lmt, float new_lmt);
float MAIN_GetLimit(Main_Limit_Type lmt);
float MAIN_GetGearRatio(void);
float MAIN_GetWheelSize(void);
uint16_t MAIN_GetPolePairs(void);
float MAIN_GetMotorKv(void);
uint8_t MAIN_SetGearRatio(float new_ratio);
uint8_t MAIN_SetWheelSize(float new_size_mm);
uint8_t MAIN_SetPolePairs(uint16_t new_pole_pairs);
uint8_t MAIN_SetMotorKv(float new_voltage_constant);
void MAIN_DumpRecord(void);
void MAIN_SaveVariables(void);
void MAIN_LoadVariables(void);
#ifdef DEBUG_DUMP_USED
void MAIN_SetDumpDebugOutput(uint8_t outputnum, uint8_t valuenum);
uint8_t MAIN_GetDumpDebugOutput(uint8_t outputnum);
#endif // DEBUG_DUMP_USED
void Delay(__IO uint32_t Delay);
uint32_t GetTick(void);
//void User_HallTIM_IRQ(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
