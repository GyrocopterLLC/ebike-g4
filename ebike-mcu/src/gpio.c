/*
 * gpio.c
 *
 *  Created on: May 31, 2016
 *      Author: David M Miller
 */

#include "stm32f4xx_hal.h"

/**
 * @brief  Enables the clock in the RCC for this GPIO port.
 * @param  gpio: The GPIO Port to be modified
 * @retval None
 */
void GPIO_Clk(GPIO_TypeDef* gpio)
{
	uint32_t gpionum = ((uint32_t)gpio - AHB1PERIPH_BASE);
			gpionum /= 0x0400;

	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN << gpionum);
}
/**
 * @brief  Configures a GPIO pin for output.
 * 		   - Maximum speed (50MHz), No pull up or pull down, Push-Pull output driver
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an output
 * @retval None
 */
void GPIO_Output(GPIO_TypeDef* gpio, uint8_t pin)
{
	// Clear MODER for this pin
	gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
	// Set this pin's MODER to output
	gpio->MODER |= (GPIO_MODER_MODER0_0 << (pin * 2));
	// Set output type to push-pull
	gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
	// Set pull-up/down off
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
	// Set speed to maximum
	gpio->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
}

/**
 * @brief  Configures a GPIO pin for input.
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an output
 * @retval None
 */
void GPIO_Input(GPIO_TypeDef* gpio, uint8_t pin)
{
	// Clear MODER for this pin (input mode)
	gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
}

void GPIO_Analog(GPIO_TypeDef* gpio, uint8_t pin)
{
	// Clear pull-up/down resistor setting
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
	// Set MODER to analog for this pin
	gpio->MODER |= ((GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1) << (pin * 2));
}

/**
 * @brief  Configures a GPIO pin for alternate function.
 * @param  gpio: The GPIO Port to be modified
 *         pin: The pin to be set as an output
 *         af: The alternate function setting
 * @retval None
 */
void GPIO_AF(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af)
{
	// Clear MODER for this pin
	gpio->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
	// Set this pin's MODER to alternate function
	gpio->MODER |= (GPIO_MODER_MODER0_1 << (pin * 2));
	// Set output type to push-pull
	gpio->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
	// Set pull-up/down off
	gpio->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
	// Set speed to maximum
	gpio->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
	// Set alternate function register
	if(pin >= 8)
	{
		gpio->AFR[1] &= ~((0x0F) << ((pin - 8)*4));;
		gpio->AFR[1] |= (af << ((pin - 8)*4));
	}
	else
	{
		gpio->AFR[0] &= ~((0x0F) << (pin*4));
		gpio->AFR[0] |= (af << (pin*4));
	}
}
