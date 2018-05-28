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
void MAIN_SetUSBDebugging(uint8_t on_or_off);
void MAIN_SetRampSpeed(uint32_t newspeed);
void MAIN_SetRampDir(uint8_t forwardOrBackwards);
void MAIN_SetVar(uint8_t var, float newval);
void MAIN_SetError(uint32_t errorCode);
void MAIN_SoftReset(uint8_t restartInBootloader);
void MAIN_DumpRecord(void);
void Delay(__IO uint32_t Delay);
uint32_t GetTick(void);
//void User_HallTIM_IRQ(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
