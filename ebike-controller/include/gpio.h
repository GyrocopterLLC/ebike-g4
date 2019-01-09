/*
 * gpio.h
 *
 *  Created on: May 31, 2016
 *      Author: David
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f4xx.h"

#define EXTI_None           ((uint8_t)0x00)
#define EXTI_Rising         ((uint8_t)0x01)
#define EXTI_Falling        ((uint8_t)0x02)
#define EXTI_Rising_Falling (EXTI_Rising | EXTI_Falling)

void GPIO_Clk(GPIO_TypeDef* gpio);
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPD(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af);
void GPIO_Pulldown_Unused(void);
void GPIO_EXTI_Config(GPIO_TypeDef* gpio, uint8_t pin, uint8_t rise_fall_select);

#endif /* GPIO_H_ */
