/*
 * gpio.h
 *
 *  Created on: May 31, 2016
 *      Author: David
 */

#ifndef GPIO_H_
#define GPIO_H_

void GPIO_Clk(GPIO_TypeDef* gpio);
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af);

#endif /* GPIO_H_ */
