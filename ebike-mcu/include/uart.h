#ifndef UART_H_
#define UART_H_

#include "stm32f4xx_hal.h"

#define HBD_UART				USART3
#define BMS_UART				USART2

#define HBD_UART_CLK_ENABLE()	__HAL_RCC_USART3_CLK_ENABLE()
#define BMS_UART_CLK_ENABLE()	__HAL_RCC_USART2_CLK_ENABLE()

#define USART_CLK				42000000
#define HBD_BAUDRATE			115200
#define HBD_USARTDIV			(22.8125f)
#define HBD_BRR					(22 << 4) + 13

#define BMS_BAUDRATE			115200
#define BMS_USARTDIV			(22.8125f)
#define BMS_BRR					(22 << 4) + 13

#define HBD_BUFFER_LENGTH		64
#define HBD_TXMT_TIMEOUT		100 // ms

typedef struct
{
    uint8_t Buffer[HBD_BUFFER_LENGTH];
    uint8_t RdPos, WrPos;
    uint8_t Done;
} HBDBuffer_Type;

void HBD_Init(void);
void HBD_IRQ(void);
uint8_t HBD_Receive(uint8_t* buf, uint8_t count);
uint8_t HBD_Transmit(uint8_t* buf, uint8_t count);

#endif // UART_H_
