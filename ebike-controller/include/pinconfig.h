#ifndef __PINCONFIG_H
#define __PINCONFIG_H

/* Peripheral usage:
 * TIM1 - 3phase bridge PWM output
 * TIM2 -
 * TIM3 - Hall sensors
 * TIM4 -
 * TIM5 -
 * TIM6 -
 * TIM7 -
 * TIM8 - Hall sampling timer
 * TIM9 -
 * TIM10 -
 * TIM11 -
 * TIM12 - Basic timer for application scheduling
 * TIM13 - PAS timer for Thr1
 * TIM14 - PAS timer for Thr2
 * USART1 -
 * USART2 - Battery management system (BMS)
 * USART3 - Handle bar display (HBD)
 * UART4 -
 * UART5 -
 * USART6 -
 */


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

// Unused Pins
#define PORTA_UNUSED	(PIN0 | PIN1 | PIN6 | PIN7 | PIN15)
#define PORTB_UNUSED	(PIN2 | PIN3 | PIN4 | PIN5 | PIN6 | PIN7 | PIN8 | PIN9)
#define PORTC_UNUSED	(PIN4 | PIN10 | PIN11 | PIN13 | PIN14 | PIN15)
#define PORTD_UNUSED	(PIN2)
#define PORTH_UNUSED	(PIN1)

#endif //__PINCONFIG_H
