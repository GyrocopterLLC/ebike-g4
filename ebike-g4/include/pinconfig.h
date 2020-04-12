/******************************************************************************
 * Filename: pinconfig.h
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

#ifndef __PINCONFIG_H
#define __PINCONFIG_H

// GPIOs
#define PIN(i)              (1 << i)

// User I/O
#define DRV_EN_PORT         GPIOC
#define DRV_EN_PIN          13
#define LED_PORT            GPIOC
#define GLED_PIN            14
#define RLED_PIN            15

// SPI
#define SPI_PORT            GPIOB
#define SPI_MOSI_PIN        5
#define SPI_MISO_PIN        4
#define SPI_SCK_PIN        3
#define SPI_CS_PIN          9
#define SPI_AF              (5U) // SPI1_MISO/MOSI/SCK

// Hall Sensor
#define HALL_PORT           GPIOB
#define HALL_A_PIN          6
#define HALL_B_PIN          7
#define HALL_C_PIN          8   // Note: PB8 is shared with BOOT0. This feature must be
                                // disabled otherwise the MCU may randomly start up in
                                // bootloader mode.
#define HALL_AF             (2U) // TIM4 Alternate Function

// PWM
#define PWM_HI_PORT         GPIOA
#define PWM_LO_PORT         GPIOB
#define PWM_BKIN_PORT       GPIOA
#define PWM_AHI_PIN         8
#define PWM_BHI_PIN         9
#define PWM_CHI_PIN         10
#define PWM_ALO_PIN         13
#define PWM_BLO_PIN         14
#define PWM_CLO_PIN         15
#define PWM_BKIN_PIN        15
#define PWM_AHI_AF          (6U) // TIM1_CH1
#define PWM_BHI_AF          (6U) // TIM1_CH2
#define PWM_CHI_AF          (6U) // TIM1_CH3
#define PWM_ALO_AF          (6U) // TIM1_CH1N
#define PWM_BLO_AF          (6U) // TIM1_CH2N
#define PWM_CLO_AF          (4U) // TIM1_CH3N
#define PWM_BKIN_AF         (9U) // TIM1_BKIN

// UART
#define HBD_UART_PORT       GPIOA
#define HBD_UART_TX_PIN     2
#define HBD_UART_RX_PIN     3
#define HBD_UART_AF         (7U) // USART2_TX/RX
#define BMS_UART_PORT       GPIOB
#define BMS_UART_TX_PIN     10
#define BMS_UART_RX_PIN     11
#define BMS_UART_AF         (7U) // USART3_TX/RX

// ADC
#define ADC_MTEMP_PORT      GPIOF
#define ADC_MTEMP_PIN       0
#define ADC_FTEMP_PORT      GPIOF
#define ADC_FTEMP_PIN       1
#define ADC_IA_PORT         GPIOB
#define ADC_IA_PIN          0
#define ADC_IB_PORT         GPIOA
#define ADC_IB_PIN          1
#define ADC_IC_PORT         GPIOA
#define ADC_IC_PIN          0
#define ADC_VA_PORT         GPIOA
#define ADC_VA_PIN          6
#define ADC_VB_PORT         GPIOA
#define ADC_VB_PIN          7
#define ADC_VC_PORT         GPIOB
#define ADC_VC_PIN          1
#define ADC_THR_PORT        GPIOB
#define ADC_THR_PIN         2
#define ADC_VBUS_PORT       GPIOB
#define ADC_VBUS_PIN        12

// DAC
#define DAC_PORT            GPIOA
#define DAC1_PIN            4
#define DAC2_PIN            5

// USB
#define USB_PORT      GPIOA
#define USB_DM_PIN      11
#define USB_DP_PIN      12

// Unused Pins
// None!

#endif //__PINCONFIG_H
