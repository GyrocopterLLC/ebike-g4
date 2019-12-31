/******************************************************************************
 * Filename: uart.c
 * Description: Low level hardware driver for the Universal Asynchronous
 *              Receiver/Transmitter (UART).
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

#include "uart.h"
#include "gpio.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "main.h"

UARTBuffer_Type BMSRxBuffer;
UARTBuffer_Type BMSTxBuffer;

UARTBuffer_Type HBDRxBuffer;
UARTBuffer_Type HBDTxBuffer;

/*** UART_CalcBRR
 * From ST reference manual RM0091:
 * In case of oversampling by 16, the equation is:
 *      Tx/Rx baud = (fck)/(USARTDIV)
 * In case of oversampling by 8, the equation is:
 *      Tx/Rx baud = (2*fck)/(USARTDIV)
 * USARTDIV is an unsigned fixed point number that is coded
 * on the USART_BRR register.
 * When OVER8=0, BRR = USARTDIV
 * When OVER8=1,
 *  - BRR[2:0] = USARTDIV[3:0] shifted 1 to the right
 *  - BRR[3] must be kept clear
 *  - BRR[15:4] = USARTDIV[15:4]
 *
 *  Strangely, the STM32F4 manual (RM0090) has a different definition.
 *  But this definition gives the same result as above!
 *  Tx/Rx baud = (fck)/(8*(2-over8)*usartdiv)
 *  Whole part of usartdiv is 12 bits, coded on USART_BRR[15:4]
 *  when over8=0, fractional part of usartdiv is 4 bits, coded on USART_BRR[3:0]
 *  when over8=1, fractional part of usartdiv is 3 bits, coded on USART_BRR[2:0], USART_BRR[3] = 0
 */
uint16_t UART_CalcBRR(uint32_t fck, uint32_t baud, uint8_t over8) {
    uint16_t usartdiv, brr;
    if(baud == 0) {
        return 0;
    }
    if(baud > fck) {
        return 0;
    }
    if(over8) {
        usartdiv = (2*fck)/baud;
        brr = (usartdiv&(0xFFF0)) + ((usartdiv&(0x000F)) >> 1);
        return brr;
    } else {
        usartdiv = fck/baud;
        return usartdiv;
    }
}

void UART_Init(void) {
    // Clock everything
    GPIO_Clk(HBD_UART_PORT);
    HBD_UART_CLK_ENABLE();

    GPIO_Clk(BMS_UART_PORT);
    BMS_UART_CLK_ENABLE();

    // Set up the GPIOs
    GPIO_AF(HBD_UART_PORT, HBD_UART_TX_PIN, HBD_UART_AF);
    GPIO_AF(HBD_UART_PORT, HBD_UART_RX_PIN, HBD_UART_AF);

    GPIO_AF(BMS_UART_PORT, BMS_UART_TX_PIN, BMS_UART_AF);
    GPIO_AF(BMS_UART_PORT, BMS_UART_RX_PIN, BMS_UART_AF);

    // Initialize the UART peripherals
    HBD_UART->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_TE;
    HBD_UART->CR2 = 0;
    HBD_UART->CR3 = 0;
    HBD_UART->BRR = HBD_BRR;

    BMS_UART->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_TE;
    BMS_UART->CR2 = 0;
    BMS_UART->CR3 = 0;
    BMS_UART->BRR = BMS_BRR;

    // Startup values in the buffers
    BMSRxBuffer.Done = 0;
    BMSRxBuffer.RdPos = 0;
    BMSRxBuffer.WrPos = 0;
    BMSTxBuffer.Done = 1;

    HBDRxBuffer.Done = 0;
    HBDRxBuffer.RdPos = 0;
    HBDRxBuffer.WrPos = 0;
    HBDTxBuffer.Done = 1;

    // Enable the UARTS
    HBD_UART->CR1 |= USART_CR1_UE;
    BMS_UART->CR1 |= USART_CR1_UE;

    // Interrupt config
    NVIC_SetPriority(HBD_IRQn, PRIO_HBD_UART);
    NVIC_SetPriority(BMS_IRQn, PRIO_BMS_UART);
    NVIC_EnableIRQ(HBD_IRQn);
    NVIC_EnableIRQ(BMS_IRQn);
}

int32_t UART_InWaiting(UART_Sel uart) {
    UARTBuffer_Type* p_RxBuffer;
    uint32_t buffer_length;
    int32_t in_waiting = 0;
    if(uart == SELECT_BMS_UART) {
        p_RxBuffer = &BMSRxBuffer;
        buffer_length = BMS_BUFFER_LENGTH;
    } else if(uart == SELECT_HBD_UART) {
        p_RxBuffer = &HBDRxBuffer;
        buffer_length = HBD_BUFFER_LENGTH;
    } else {
        return 0;
    }
    if (p_RxBuffer->WrPos > p_RxBuffer->RdPos) {
        // Write is past read...everything past the read
        // pointer up to the write pointer is available to read.
        in_waiting = p_RxBuffer->WrPos - p_RxBuffer->RdPos;
    } else if (p_RxBuffer->WrPos < p_RxBuffer->RdPos) {
        // Read is past write...that means write wrapped around.
        // Lots to read. From the read pointer to the end, then
        // from zero up to the write pointer.
        in_waiting = buffer_length - p_RxBuffer->RdPos + p_RxBuffer->WrPos;
    } else if (p_RxBuffer->WrPos == p_RxBuffer->RdPos) {
        // Depends if an overflow occurred. If it did, then the buffer
        // has been filled up to the buffer length. If it did not, then
        // the buffer is empty.
        if((p_RxBuffer->Done & 0x02)==0) {
            in_waiting = 0;
        } else {
            in_waiting = buffer_length;
        }
    }
    return in_waiting;
}

uint8_t UART_IsFinishedTx(UART_Sel uart) {
    if(uart == SELECT_BMS_UART) {
        return BMSTxBuffer.Done;
    } else if(uart == SELECT_HBD_UART) {
        return HBDTxBuffer.Done;
    }
    return 0;
}

int32_t UART_Read(UART_Sel uart, void* buf, uint32_t count) {
    uint8_t* buf8b = buf;
    UARTBuffer_Type* p_RxBuffer;
    uint32_t buffer_remaining = 0;
    uint32_t place = 0;
    uint32_t buffer_length;

    // Select the BMS or HBD UART
    if(uart == SELECT_BMS_UART) {
        p_RxBuffer = &BMSRxBuffer;
        buffer_length = BMS_BUFFER_LENGTH;
    } else if(uart == SELECT_HBD_UART) {
        p_RxBuffer = &HBDRxBuffer;
        buffer_length = HBD_BUFFER_LENGTH;
    } else {
        return 0;
    }

    // Copy up to "count" bytes from the read buffer
    if ((p_RxBuffer->Done & 0x01) == 0) {
        // No bytes read, return zero.
        return 0;
    }
    if(count == 0) {
        return 0;
    }

    if(p_RxBuffer->WrPos > p_RxBuffer->RdPos){
        // Write is past read...this means some bytes have been written
        // that have yet to be read. The remaining space from the write pointer
        // up to the end of the buffer, plus whatever bytes have already
        // been read.
        buffer_remaining = buffer_length - p_RxBuffer->WrPos
                + p_RxBuffer->RdPos;
    } else if(p_RxBuffer->WrPos < p_RxBuffer->RdPos) {
        // Read is past write...this means write pointer wrapped around.
        // Remaining space is what's in front of the write pointer before
        // it hits the read pointer.
        buffer_remaining = p_RxBuffer->RdPos - p_RxBuffer->WrPos;
    } else if(p_RxBuffer->WrPos == p_RxBuffer->RdPos) {
        // Weird case. If overflow has occurred, then the entire buffer is
        // remaining. If it hasn't, then there are zero bytes remaining.
        if((p_RxBuffer->Done & 0x02) == 0) {
            buffer_remaining = 0;
        } else {
            buffer_remaining = buffer_length;
            p_RxBuffer->Done &= ~0x02;
        }
    }


    while ((place < count) && (place < buffer_remaining)) {
        buf8b[place++] = p_RxBuffer->Buffer[p_RxBuffer->RdPos++];
        if (p_RxBuffer->RdPos >= buffer_length)
            p_RxBuffer->RdPos = 0;
    }
    // Clear "done" flag if no more bytes to read
    if (count >= buffer_remaining)
        p_RxBuffer->Done &= ~0x01;
    // Return the number of read bytes.
    return place;
}

int32_t UART_Write(UART_Sel uart, void* buf, uint32_t count) {
    uint8_t* buf8b = buf;
    uint32_t place = 0;
    UARTBuffer_Type* p_TxBuffer;
    uint32_t timeout_limit;
    USART_TypeDef* uart_hw;
    uint32_t buffer_length;

    if(uart == SELECT_BMS_UART) {
        p_TxBuffer = &BMSTxBuffer;
        timeout_limit = HBD_TXMT_TIMEOUT;
        uart_hw = BMS_UART;
        buffer_length = BMS_BUFFER_LENGTH;

    } else if(uart == SELECT_HBD_UART) {
        p_TxBuffer = &HBDTxBuffer;
        timeout_limit = BMS_TXMT_TIMEOUT;
        uart_hw = HBD_UART;
        buffer_length = HBD_BUFFER_LENGTH;
    } else {
        return 0;
    }

    if(count == 0) {
        return 0;
    }

    if (p_TxBuffer->Done) {
        // In the rare chance that the transmitter is just finishing,
        // wait for the shift register to empty
        uint32_t timeout = GetTick();
        while (!(uart_hw->SR & USART_SR_TXE)) {
            if (GetTick() > (timeout + timeout_limit)) {
                // Timeout fail
                return 0;
            }
        }

        // Safe to simply restart the buffer
        p_TxBuffer->RdPos = 0;
        p_TxBuffer->WrPos = 0;
        while ((place < buffer_length) && (place < count)) {
            p_TxBuffer->Buffer[p_TxBuffer->WrPos++] = buf8b[place++];
        }
        // Start the transfer

        uart_hw->DR = p_TxBuffer->Buffer[p_TxBuffer->RdPos++];
        uart_hw->CR1 |= (USART_CR1_TXEIE);
        p_TxBuffer->Done = 0;
    } else {
        // Can we fit more data in the buffer?
        uint8_t buffer_used;
        if (p_TxBuffer->WrPos < p_TxBuffer->RdPos) {
            buffer_used = buffer_length - p_TxBuffer->RdPos
                    + p_TxBuffer->WrPos;
        } else {
            buffer_used = p_TxBuffer->WrPos - p_TxBuffer->RdPos;
        }
        uint8_t buffer_remaining = buffer_length - buffer_used;

        if (count <= buffer_remaining) {
            while ((place < buffer_remaining) && (place < count)) {
                p_TxBuffer->Buffer[p_TxBuffer->WrPos++] = buf8b[place++];
            }
        } else {
            // Buffer was full, fail.
            return 0;
        }
    }
    // Return the number of written bytes.
    return place;
}

void UART_IRQ(UART_Sel uart) {
    UARTBuffer_Type* p_RxBuffer;
    UARTBuffer_Type* p_TxBuffer;
    USART_TypeDef* uart_hw;
    uint32_t buffer_length;
    uint8_t comm_dummy;

    if(uart == SELECT_BMS_UART) {
        p_RxBuffer = &BMSRxBuffer;
        p_TxBuffer = &BMSTxBuffer;
        uart_hw = BMS_UART;
        buffer_length = BMS_BUFFER_LENGTH;
    } else if(uart == SELECT_HBD_UART) {
        p_RxBuffer = &HBDRxBuffer;
        p_TxBuffer = &HBDTxBuffer;
        uart_hw = HBD_UART;
        buffer_length = HBD_BUFFER_LENGTH;
    } else {
        return;
    }

    // Character received!
    if ((uart_hw->SR & USART_SR_RXNE) != 0) {
        // Receive the new character, this clears the interrupt
        comm_dummy = uart_hw->DR;
        // Add it to our buffer
        p_RxBuffer->Buffer[p_RxBuffer->WrPos++] = comm_dummy;
        p_RxBuffer->Done |= 0x01;
        if (p_RxBuffer->WrPos >= buffer_length) {
            // Wrap around the circular buffer
            p_RxBuffer->WrPos = 0;
        }
        if(p_RxBuffer->WrPos == p_RxBuffer->RdPos) {
            p_RxBuffer->Done |= 0x02; // Overflow!
        }
    }
    if (((uart_hw->SR & USART_SR_TXE) != 0)
            && ((uart_hw->CR1 & USART_CR1_TXEIE) != 0)) {
        // Check if more data to send
        if (((p_TxBuffer->RdPos + 1U) == p_TxBuffer->WrPos)
                || (((p_TxBuffer->RdPos + 1U) == buffer_length)
                        && (p_TxBuffer->WrPos == 0))) {
            // We are done after this byte!
            p_TxBuffer->Done = 1;
            // Turn off the transmit data register empty interrupt
            uart_hw->CR1 &= ~(USART_CR1_TXEIE);
            // Turn on the transmit complete interrupt
            uart_hw->CR1 |= USART_CR1_TCIE;
        }
        // Load the next data. This clears the empty interrupt
        uart_hw->DR = p_TxBuffer->Buffer[p_TxBuffer->RdPos++];
        if (p_TxBuffer->RdPos >= buffer_length) {
            // Wrap around the read pointer
            p_TxBuffer->RdPos = 0;
        }
    }
    if (((uart_hw->SR & USART_SR_TC) != 0)
            && ((uart_hw->CR1 & USART_CR1_TCIE) != 0)) {
        // Transmission complete, turn off the interrupt
        uart_hw->CR1 &= ~(USART_CR1_TCIE);
    }
}
