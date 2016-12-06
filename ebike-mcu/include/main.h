/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    26-December-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* PMSM3_1 Testing Phase definition */
#define PHASE	2

#ifndef PHASE
#error Testing phase needs to be defined.
#endif

/* Constants */
#define THROTTLE_MIN	(0.7f) // Less than 0.7V is zero throttle
#define THROTTLE_MAX	(2.8f) // Above 2.8V is 100% throttle
#define THROTTLE_SCALE	(0.47619f)
#define THROTTLE_STARTUP_COUNT 1500 // Wait 1.5 sec for filter to stabilize

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
//#include "stm324xg_eval.h"
#include "usbd_desc.h"
#include "usbd_cdc.h" 
#include "usbd_cdc_interface.h"
#include "gpio.h"
#include "DavidsFOCLib.h"
#include "hallSensor.h"
#include "adc.h"
#include "pwm.h"
#include "throttle.h"
#include "pinconfig.h"

#include <stdio.h>
#include <string.h>


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
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void User_BasicTIM_IRQ(void);
void User_PWMTIM_IRQ(void);
//void User_HallTIM_IRQ(void);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
