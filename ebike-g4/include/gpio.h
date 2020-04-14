/******************************************************************************
 * Filename: gpio.h
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

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32g4xx.h"

#define EXTI_None           ((uint8_t)0x00)
#define EXTI_Rising         ((uint8_t)0x01)
#define EXTI_Falling        ((uint8_t)0x02)
#define EXTI_Rising_Falling (EXTI_Rising | EXTI_Falling)

typedef enum _PuPd_Type {
  PuPd_NoPull,
  PuPd_PullUp,
  PuPd_PullDown
} PuPd_Type;

#define GPIO_High(gpio, pin)        ((gpio->BSRR) = (0x01U << pin))
#define GPIO_Low(gpio, pin)         ((gpio->BRR) = (0x01U << pin))

void GPIO_Clk(GPIO_TypeDef* gpio);
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPD(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_InputPU(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin);
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af);
void GPIO_SetPUPD(GPIO_TypeDef* gpio, uint8_t pin, PuPd_Type pullupdn);
void GPIO_EXTI_Config(GPIO_TypeDef* gpio, uint8_t pin,
        uint8_t rise_fall_select);

#endif /* GPIO_H_ */
