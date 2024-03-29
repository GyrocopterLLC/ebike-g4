/******************************************************************************
 * Filename: main.h
 * Description: Combine all the necessary include files, so that most
 *              other *.c files can just include this one header.
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

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32g4xx.h"
#include "main_data_types.h"
#include "adc.h"
#include "cordic_sin_cos.h"
#include "crc.h"
#include "data_commands.h"
#include "data_packet.h"
#include "delay.h"
#include "drv8353.h"
#include "eeprom_emulation.h"
#include "foc_lib.h"
#include "gpio.h"
#include "hall_sensor.h"
#include "hbd_data_comm.h"
#include "live_data.h"
#include "motor_loop.h"
#include "periphconfig.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "power_calcs.h"
#include "pwm.h"
#include "throttle.h"
#include "uart.h"
#include "usb_cdc.h"
#include "usb_data_comm.h"
#include "usb.h"
#include "wdt.h"

// Basic definitions used in many files
#define RETVAL_OK           (1)
#define RETVAL_FAIL         (0)

// Bootloader locations
#define BOOTLOADER_REMAPPED_TOP_OF_STACK        ((uint32_t)0x00000000)
#define BOOTLOADER_REMAPPED_RESET_VECTOR        ((uint32_t)0x00000004)
#define BOOTLOADER_TOP_OF_STACK     ((uint32_t)0x1FFF0000)
#define BOOTLOADER_RESET_VECTOR     ((uint32_t)0x1FFF0004)
#define BOOTLOADER_RESET_FLAG       ((uint32_t)0x7441634F) // "tAcO"

// Various settings
#define APP_TIM_RATE        (1000) // 1kHz update rate

// Exported functions

uint8_t MAIN_GetDashboardData(uint8_t* data); // Returns live values
uint8_t MAIN_SetControlMode(Main_Control_Methods new_ctrl_mode);
Main_Control_Methods MAIN_GetControlMode(void);
void MAIN_Reboot(void); // Restart processor
void MAIN_GoToBootloader(void); // Restart and go to bootloader at startup
void MAIN_AppTimerISR(void); // Called periodically to do housekeeping functions
void MAIN_MotorISR(void); // Called periodically to perform motor control functions

uint8_t uiFOC_SetKp(uint8_t* valptr);
uint8_t uiFOC_GetKp(uint8_t* valptr);
uint8_t uiFOC_SetKi(uint8_t* valptr);
uint8_t uiFOC_GetKi(uint8_t* valptr);
uint8_t uiFOC_SetKd(uint8_t* valptr);
uint8_t uiFOC_GetKd(uint8_t* valptr) ;
uint8_t uiFOC_SetKc(uint8_t* valptr);
uint8_t uiFOC_GetKc(uint8_t* valptr);
uint8_t uiFOC_SetPwmFreq(uint8_t* valptr);
uint8_t uiFOC_GetPwmFreq(uint8_t* valptr);
uint8_t uiFOC_SetPwmDeadtime(uint8_t* valptr);
uint8_t uiFOC_GetPwmDeadtime(uint8_t* valptr);

uint8_t uiMAIN_SetCountsToFoc(uint8_t* valptr);
uint8_t uiMAIN_GetCountsToFoc(uint8_t* valptr);
uint8_t uiMAIN_SetSpeedToFoc(uint8_t* valptr);
uint8_t uiMAIN_GetSpeedToFoc(uint8_t* valptr);
uint8_t uiMAIN_SetSwitchEps(uint8_t* valptr);
uint8_t uiMAIN_GetSwitchEps(uint8_t* valptr);
uint8_t uiMAIN_SetNumUsbOutputs(uint8_t* valptr);
uint8_t uiMAIN_GetNumUsbOutputs(uint8_t* valptr);
uint8_t uiMAIN_SetUsbSpeed(uint8_t* valptr);
uint8_t uiMAIN_GetUsbSpeed(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice1(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice1(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice2(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice2(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice3(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice3(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice4(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice4(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice5(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice5(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice6(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice6(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice7(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice7(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice8(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice8(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice9(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice9(uint8_t* valptr);
uint8_t uiMAIN_SetUsbChoice10(uint8_t* valptr);
uint8_t uiMAIN_GetUsbChoice10(uint8_t* valptr);

uint8_t uiMOTOR_GetPolePairs(uint8_t* valptr);
uint8_t uiMOTOR_SetPolePairs(uint8_t* valptr);
uint8_t uiMOTOR_GetGearRatio(uint8_t* valptr);
uint8_t uiMOTOR_SetGearRatio(uint8_t* valptr);
uint8_t uiMOTOR_GetWheelSize(uint8_t* valptr);
uint8_t uiMOTOR_SetWheelSize(uint8_t* valptr);
uint8_t uiMOTOR_GetKv(uint8_t* valptr);
uint8_t uiMOTOR_SetKv(uint8_t* valptr);

uint8_t uiLMT_GetVoltFaultMin(uint8_t* valptr);
uint8_t uiLMT_SetVoltFaultMin(uint8_t* valptr);
uint8_t uiLMT_GetVoltFaultMax(uint8_t* valptr);
uint8_t uiLMT_SetVoltFaultMax(uint8_t* valptr);
uint8_t uiLMT_GetCurFaultMax(uint8_t* valptr);
uint8_t uiLMT_SetCurFaultMax(uint8_t* valptr);
uint8_t uiLMT_GetVoltSoftCap(uint8_t* valptr);
uint8_t uiLMT_SetVoltSoftCap(uint8_t* valptr);
uint8_t uiLMT_GetVoltHardCap(uint8_t* valptr);
uint8_t uiLMT_SetVoltHardCap(uint8_t* valptr);
uint8_t uiLMT_GetPhaseCurMax(uint8_t* valptr);
uint8_t uiLMT_SetPhaseCurMax(uint8_t* valptr);
uint8_t uiLMT_GetPhaseRegenMax(uint8_t* valptr);
uint8_t uiLMT_SetPhaseRegenMax(uint8_t* valptr);
uint8_t uiLMT_GetBattCurMax(uint8_t* valptr);
uint8_t uiLMT_SetBattCurMax(uint8_t* valptr);
uint8_t uiLMT_GetBattRegenMax(uint8_t* valptr);
uint8_t uiLMT_SetBattRegenMax(uint8_t* valptr);
uint8_t uiLMT_GetFetTempSoftCap(uint8_t* valptr);
uint8_t uiLMT_SetFetTempSoftCap(uint8_t* valptr);
uint8_t uiLMT_GetFetTempHardCap(uint8_t* valptr);
uint8_t uiLMT_SetFetTempHardCap(uint8_t* valptr);
uint8_t uiLMT_GetMotorTempSoftCap(uint8_t* valptr);
uint8_t uiLMT_SetMotorTempSoftCap(uint8_t* valptr);
uint8_t uiLMT_GetMotorTempHardCap(uint8_t* valptr);
uint8_t uiLMT_SetMotorTempHardCap(uint8_t* valptr);

#endif //__MAIN_H
