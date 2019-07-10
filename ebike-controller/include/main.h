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
#include "ui.h"
#include "wdt.h"
#include "power_calcs.h"
#include "crc32.h"
#include "data_packet.h"
#include "data_commands.h"
#include "usb_data_comm.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    PB_RELEASED, PB_PRESSED
} PB_TypeDef;
/* Exported constants --------------------------------------------------------*/
#define MAXLEDCOUNT         1000
#define DEBOUNCE_INTERVAL   10 // 10 milliseconds ==> 100Hz timer
#define DEBOUNCE_MAX        5 // Must get integrator up to 5 to count as "pressed"

#define BOOTLOADER_RESET_FLAG 0xDEADBEEF

#define SERIAL_DUMP_RATE        (1)
#define TEMP_CONVERSION_RATE    (100)

#define MAIN_ERR_HALL_STATE   (0x00000800)

#define MAINFLAG_SERIALDATAPRINT    ((uint32_t)0x00000001)
#define MAINFLAG_SERIALDATAON       ((uint32_t)0x00000002)
#define MAINFLAG_DUMPRECORD         ((uint32_t)0x00000004)
#define MAINFLAG_DUMPDATAPRINT      ((uint32_t)0x00000008)
#define MAINFLAG_DUMPDATAON         ((uint32_t)0x00000010)
#define MAINFLAG_CONVERTTEMPERATURE ((uint32_t)0x00000020)
#define MAINFLAG_HALLDETECTFAIL     ((uint32_t)0x00000040)
#define MAINFLAG_HALLDETECTPASS     ((uint32_t)0x00000080)
#define MAINFLAG_LASTCOMMSERIAL     ((uint32_t)0x00000100)

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
uint8_t MAIN_SetRampSpeed(uint32_t newspeed);
uint8_t MAIN_SetRampDir(uint8_t forwardOrBackwards);
uint8_t MAIN_SetVar(uint8_t var, float newval);
float MAIN_GetVar(uint8_t var);
float MAIN_GetVar_EEPROM(uint8_t var);
uint8_t MAIN_SetFreq(int32_t newfreq);
int32_t MAIN_GetFreq(void);
uint8_t MAIN_SetDeadTime(int32_t newDT);
int32_t MAIN_GetDeadTime(void);
void MAIN_SetError(uint32_t errorCode);
void MAIN_SoftReset(uint8_t restartInBootloader);
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
