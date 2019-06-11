/******************************************************************************
 * Filename: pinconfig.h
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

#ifndef __PINCONFIG_H
#define __PINCONFIG_H

// GPIOs
#define PIN0		0x0001
#define PIN1		0x0002
#define PIN2		0x0004
#define PIN3		0x0008
#define PIN4		0x0010
#define PIN5		0x0020
#define PIN6		0x0040
#define PIN7		0x0080
#define PIN8		0x0100
#define PIN9		0x0200
#define PIN10		0x0400
#define PIN11		0x0800
#define PIN12		0x1000
#define PIN13		0x2000
#define PIN14		0x4000
#define PIN15		0x8000

// User I/O
#define GLED_PIN	9
#define RLED_PIN	12
#define PB_PIN		12
#define DAC1_PIN	4
#define DAC2_PIN	5
#define GLED_PORT	GPIOC
#define PB_PORT		GPIOC
#define RLED_PORT	GPIOB
#define DAC_PORT	GPIOA

// Hall Sensor
#define HALL_PIN_A		6
#define HALL_PIN_B		7
#define HALL_PIN_C		8
#define HALL_PORT		GPIOC
#define HALL_PINS_AF	((uint8_t)2)

// PWM
#define PWM_AHI_PIN		10
#define PWM_BHI_PIN		9
#define PWM_CHI_PIN		8
#define PWM_ALO_PIN		15
#define PWM_BLO_PIN		14
#define PWM_CLO_PIN		13
#define PWM_HI_PORT		GPIOA
#define PWM_LO_PORT		GPIOB
#define PWM_AF			((uint8_t)1)

// UART
#define HBD_UART_TX_PIN	10
#define HBD_UART_RX_PIN	11
#define HBD_UART_PORT	GPIOB
#define HBD_UART_AF		((uint8_t)7)
#define BMS_UART_TX_PIN	2
#define BMS_UART_RX_PIN	3
#define BMS_UART_PORT	GPIOA
#define BMS_UART_AF		((uint8_t)7)

// ADC
#define ADC_IA_PIN    0
#define ADC_IB_PIN    1
#define ADC_IC_PIN    2
#define ADC_VBUS_PIN  3
#define ADC_THR1_PIN  5
#define ADC_I_VBUS_THR1_PORT    GPIOC
#define ADC_TEMP_PIN  1
#define ADC_THR2_PIN  0
#define ADC_THR2_AND_TEMP_PORT    GPIOB

//#define ADC_I_VBUS_THR1_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
//#define ADC_THR2_AND_TEMP_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()

// USB
#define USB_PORT      GPIOA
#define USB_DM_PIN      11
#define USB_DP_PIN      12
#define USB_AF        10

// Unused Pins
#define PORTA_UNUSED	(PIN0 | PIN1 | PIN6 | PIN7 | PIN15)
#define PORTB_UNUSED	(PIN2 | PIN3 | PIN4 | PIN5 | PIN6 | PIN7 | PIN8 | PIN9)
#define PORTC_UNUSED	(PIN4 | PIN10 | PIN11 | PIN13 | PIN14 | PIN15)
#define PORTD_UNUSED	(PIN2)
#define PORTH_UNUSED	(PIN1)

#endif //__PINCONFIG_H
