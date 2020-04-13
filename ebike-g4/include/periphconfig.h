/******************************************************************************
 * Filename: periphconfig.h
 * Description: Settings for MCU peripherals.
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

#ifndef __PERIPHCONFIG_H
#define __PERIPHCONFIG_H

/* Peripheral usage:
 *
 * *** TIMERS ***
 * --- Advanced motor control ---
 * TIM1 - 3phase bridge PWM output
 * TIM8 -
 * TIM20 -
 * --- General purpose, 32bit ---
 * TIM2 -
 * TIM5 - Pedal Assist (PAS) timer for Thr
 * --- General purpose, 16bit ---
 * TIM3 -
 * TIM4 - Hall sensors
 * TIM15 -
 * TIM16 -
 * TIM17 -
 * --- Basic ---
 * TIM6 - Basic timer for application scheduling
 * TIM7 -
 * --- Other ---
 * LPTIM1 -
 * IDWG - Safety reset
 * WWDG -
 * SysTick - Additional basic scheduling, delays
 * RTC - Temporary save variables into BKPSRAM to command bootloader resets
 *
 * *** ANALOG ***
 * ADC1,2,3,4,5 - Measure motor current, battery voltage, and throttle
 * DAC1 - Debugging analog outputs for viewing with an oscilloscope
 * DAC2 -
 * COMP -
 * OPAMP -
 *
 * *** COMMUNICATION INTERFACES ***
 * USART1 -
 * USART2 - Handle bar display (HBD)
 * USART3 - Battery management system (BMS)
 * LPUART1 -
 * SPI1 - Communication with 3-phase bridge driver (DRV8353RS)
 * SPI2/I2S2 -
 * SPI3/I2S3 -
 * I2C1,2,3,4 -
 * FDCAN1,2,3 -
 * SAI -
 * USB Device - Debugging communications
 * UCPD -
 *
 * *** OTHER STUFF ***
 * CRS - Correct USB clock source to host SOF
 * FSMC -
 * QUADSPI -
 * DMA1 -
 * DMA2 -
 * CRC - Generate CRC-32 for packet data interface
 * RNG -
 * HASH -
 * CRYP -
 * CORDIC - sin/cos calculations in FOC
 * FMAC - biquad filter calculations
 */

// Clocks and timing
#if !defined  (HSI_VALUE)
#define HSI_VALUE    16000000U // High speed internal oscillator = 16MHz
#endif

#if !defined (LSI_VALUE)
#define LSI_VALUE       32000U // Low speed internal oscillator = 32kHz
#endif

#define PLL_M           4U
#define PLL_N           85U
#define VCO_CLK         340000000U // VCO = HSI * PLL_N / PLL_M
#define PLL_R           2U
#define SYS_CLK         170000000U // Sysclk = VCO / PLL_R
#define AHB_DIV         1U
#define HCLKC           170000000U // HCLK = Sysclk / AHB_DIV
#define APB1_DIV        1U
#define APB1_CLK        170000000U
#define APB2_DIV        1U
#define APB2_CLK        170000000U
#define PLL_Q           2U
#define PLLQ_CLK        170000000U
#define PLL_P           8U
#define PLLP_CLK         42500000U
#define ADC_CLK         PLLP_CLK // Max is 52MHz in voltage range 1, all ADCs operational
#define IWDG_CLK        LSI_VALUE

// PWM
#define PWM_TIMER               TIM1
#define PWM_CLK                 APB2_CLK
#define PWM_TIM_CLK_ENABLE()    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN
#define PWM_IRQn                TIM1_UP_TIM16_IRQn

// Hall Sensors
#define HALL_TIMER              TIM4
#define HALL_CLK                APB1_CLK
#define HALL_TIM_CLK_ENABLE()   RCC->APB1ENR |= RCC_APB1ENR_TIM4EN
#define HALL_IRQn               TIM4_IRQn

// Pedal Assist Throttle (PAS) Timer
#define PAS_TIM                 TIM5
#define PAS_CLK                 APB1_CLK
#define PAS_TIM_CLK_ENABLE()    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN
#define PAS_IRQn                TIM5_IRQn

// Application timer
#define APP_TIM                 TIM6
#define APP_CLK                 APB1_CLK
#define APP_TIMER_CLK_ENABLE()  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN
#define APP_IRQn                TIM6_DAC_IRQn

// HBD
#define HBD_UART                USART2
#define HBD_CLK                 APB1_CLK
#define HBD_UART_CLK_ENABLE()   RCC->APB1ENR |= RCC_APB1ENR_USART2EN
#define HBD_IRQn                USART2_IRQn

// BMS
#define BMS_UART                USART3
#define BMS_CLK                 APB1_CLK
#define BMS_UART_CLK_ENABLE()   RCC->APB1ENR |= RCC_APB1ENR_USART3EN
#define BMS_IRQn                USART3_IRQn

// DRV8353 Control
#define DRV_SPI                 SPI1
#define DRV_CLK                 APB2_CLK
#define DRV_SPI_CLK_ENABLE()    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN
#define DRV_IRQn                SPI1_IRQn


// Interrupt priority settings
// Lowest number takes precedence
// Multiple interrupt sources can use the same priority level,
// but only a lower number interrupt will override a currently
// responding IRQ function.
#define PRIO_PWM        (0)
#define PRIO_HALL       (1)
#define PRIO_ADC        (2)
#define PRIO_PAS        (3)
#define PRIO_APPTIMER   (3)
#define PRIO_SYSTICK    (3)
#define PRIO_HBD_UART   (4)
#define PRIO_BMS_UART   (4)
#define PRIO_DRV_SPI    (4)
#define PRIO_USB        (5)

#endif //__PERIPHCONFIG_H
