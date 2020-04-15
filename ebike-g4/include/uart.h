/******************************************************************************
 * Filename: uart.h
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

// Used resources:
// USART2, USART3
#ifndef UART_H_
#define UART_H_

#define HBD_BAUDRATE            (38400)
#define HBD_USARTDIV            (4427) // about 0.002% error (170MHz / 4427 = 38.4007kHz)
#define HBD_OVER8               (0)
#define HBD_BRR                 (4427)

#define BMS_BAUDRATE			115200
#define BMS_USARTDIV            (1476) // about 0.02% error (170MHz / 1476 = 115.176kHz)
#define BMS_OVER8               (0)
#define BMS_BRR                 (1476)

#define HBD_BUFFER_LENGTH		128
#define HBD_TXMT_TIMEOUT		3 // ms

#define BMS_BUFFER_LENGTH       128
#define BMS_TXMT_TIMEOUT        3 // ms

typedef enum _uart_sel{
    SELECT_HBD_UART,
    SELECT_BMS_UART
} UART_Sel;

typedef struct _uart_buffer{
    uint8_t Buffer[HBD_BUFFER_LENGTH];
    uint8_t RdPos, WrPos;
    uint8_t Done;
} UARTBuffer_Type;


uint16_t UART_CalcBRR(uint32_t fck, uint32_t baud, uint8_t over8);

void UART_Init(void);
int32_t UART_InWaiting(UART_Sel uart);
uint8_t UART_IsFinishedTx(UART_Sel uart);
int32_t UART_Read(UART_Sel uart, void* buf, uint32_t count);
int32_t UART_Write(UART_Sel uart, void* buf, uint32_t count);

void UART_IRQ(UART_Sel uart);

#endif // UART_H_
