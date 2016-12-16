/*
 * uart.c
 *
 *  Created on: Dec 15, 2016
 *      Author: David
 */

#include "uart.h"
#include "gpio.h"
#include "pinconfig.h"


void HBD_Init(void)
{
	// Clock everything
	GPIO_Clk(HBD_UART_PORT);
	HBD_UART_CLK_ENABLE();

	// Set up the GPIOs
	GPIO_AF(HBD_UART_PORT, HBD_UART_TX_PIN, HBD_UART_AF);
	GPIO_AF(HBD_UART_PORT, HBD_UART_RX_PIN, HBD_UART_AF);

	// Initialize the peripheral
	HBD_UART->CR1 = USART_CR1_RE | USART_CR1_RXNEIE;
	HBD_UART->CR2 = 0;
	HBD_UART->CR3 = 0;
	HBD_UART->BRR = HBD_BRR;

	HBD_UART->CR1 |= USART_CR1_UE;

	// Interrupt config
	NVIC_SetPriority(USART3_IRQn, 3); //TODO: check priority settings
	NVIC_EnableIRQ(USART3_IRQn);
}
