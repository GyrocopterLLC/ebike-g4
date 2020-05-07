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
#include "live_data.h"
#include "periphconfig.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "pwm.h"
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
uint8_t MAIN_EnableDebugPWM(void); // Turn on PWM outputs
uint8_t MAIN_DisableDebugPWM(void); // Turn off PWM outputs
void MAIN_Reboot(void); // Restart processor
void MAIN_GoToBootloader(void); // Restart and go to bootloader at startup
void MAIN_AppTimerISR(void); // Called periodically to do housekeeping functions
void MAIN_MotorISR(void); // Called periodically to perform motor control functions

#endif //__MAIN_H
