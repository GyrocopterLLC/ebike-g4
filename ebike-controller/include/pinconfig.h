#ifndef __PINCONFIG_H
#define __PINCONFIG_H

// GPIOs

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
#define HALL_PINS_AF	GPIO_AF2_TIM3

// PWM
#define PWM_AHI_PIN		10
#define PWM_BHI_PIN		9
#define PWM_CHI_PIN		8
#define PWM_ALO_PIN		15
#define PWM_BLO_PIN		14
#define PWM_CLO_PIN		13
#define PWM_HI_PORT		GPIOA
#define PWM_LO_PORT		GPIOB
#define PWM_AF			GPIO_AF1_TIM1

// UART
#define HBD_UART_TX_PIN	10
#define HBD_UART_RX_PIN	11
#define HBD_UART_PORT	GPIOB
#define HBD_UART_AF		GPIO_AF7_USART3
#define BMS_UART_TX_PIN	2
#define BMS_UART_RX_PIN	3
#define BMS_UART_PORT	GPIOA
#define BMS_UART_AF		GPIO_AF7_USART2


#endif //__PINCONFIG_H
