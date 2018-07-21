// Used resources:
// DAC
// TIM12
  

#ifndef __MAIN_H
#define __MAIN_H

/* Constants */
#define THROTTLE_MIN	(0.7f) // Less than 0.7V is zero throttle
#define THROTTLE_MAX	(2.8f) // Above 2.8V is 100% throttle
#define THROTTLE_SCALE	(0.47619f)
#define THROTTLE_STARTUP_COUNT 1500 // Wait 1.5 sec for filter to stabilize

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
#include "throttle.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "uart.h"
#include "ui.h"
#include "wdt.h"
#include "power_calcs.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


/* Exported types ------------------------------------------------------------*/
typedef enum
{
	PB_RELEASED,
	PB_PRESSED
}PB_TypeDef;
/* Exported constants --------------------------------------------------------*/
#define MAXLEDCOUNT		1000
#define DEBOUNCE_INTERVAL		10 // 10 milliseconds ==> 100Hz timer
#define DEBOUNCE_MAX			5 // Must get integrator up to 5 to count as "pressed"

#define RAMP_CALLFREQ   (20000)
#define RAMP_DEFAULTSPEED (5)

#define BOOTLOADER_RESET_FLAG 0xDEADBEEF

//#define SERIAL_DATA_RATE      (10)

#define SERIAL_DUMP_RATE        (1)
#define TEMP_CONVERSION_RATE    (100)

#define SPEED_COUNTS_TO_FOC     (1000)
#define MIN_SPEED_TO_FOC        (10.0f)

#define MAIN_STARTUP_SPEED_MAX      (65536*10*MOTOR_POLEPAIRS/60) // 10 RPM in electrical Hz (Q16 format)
#define MAIN_STARTUP_CUR_AVG_COUNT  (256)

#define MAX_USB_VALS              18
#define MAX_USB_OUTPUTS           10
#define DEFAULT_USB_OUTPUTS       5
#define DEFAULT_USB_ASSIGNMENTS   {1, 2, 3, 7, 11, 4, 5, 6, 9, 18}
#define USB_PREFIX_LENGTH         4
#define DEFAULT_USB_PREFIX        "DB05"
#define MAX_USB_SPEED_CHOICES     6
#define USB_SPEED_RELOAD_VALS     {400, 200, 100, 40, 20, 4} // 50Hz, 100Hz, 200Hz, 500Hz, 1kHz, 5kHz
#define DEFAULT_SERIAL_DATA_RATE  (400) // (20kHz/400 = 50Hz)

#define MAIN_ERR_HALL_STATE		(0x00000800)
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
//uint32_t itoa(char* buf, int32_t num);
void stringflip(char* buf, uint32_t len);
uint32_t _itoa(char* buf, int32_t num, uint32_t min_digits);
uint32_t _ftoa(char* buf, float num, uint32_t precision);
void SYSTICK_IRQHandler(void);
void User_BasicTIM_IRQ(void);
void User_PWMTIM_IRQ(void);
void MAIN_SetUSBDebugOutput(uint8_t outputnum, uint8_t valuenum);
uint8_t MAIN_GetUSBDebugOutput(uint8_t outputnum);
void MAIN_SetNumUSBDebugOutputs(uint8_t numOutputs);
uint8_t MAIN_GetNumUSBDebugOutputs(void);
void MAIN_SetUSBDebugSpeed(uint8_t speedChoice);
uint8_t MAIN_GetUSBDebugSpeed(void);
void MAIN_SetUSBDebugging(uint8_t on_or_off);
uint8_t MAIN_GetUSBDebugging(void);
void MAIN_SetRampSpeed(uint32_t newspeed);
void MAIN_SetRampDir(uint8_t forwardOrBackwards);
void MAIN_SetVar(uint8_t var, float newval);
float MAIN_GetVar(uint8_t var);
void MAIN_SetError(uint32_t errorCode);
void MAIN_SoftReset(uint8_t restartInBootloader);
void MAIN_DumpRecord(void);
void MAIN_SetDumpDebugOutput(uint8_t outputnum, uint8_t valuenum);
uint8_t MAIN_GetDumpDebugOutput(uint8_t outputnum);
void Delay(__IO uint32_t Delay);
uint32_t GetTick(void);
//void User_HallTIM_IRQ(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
