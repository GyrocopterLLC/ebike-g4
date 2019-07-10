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

HBDBuffer_Type s_RxBuffer;
HBDBuffer_Type s_TxBuffer;

void HBD_Init(void) {
    // Clock everything
    GPIO_Clk(HBD_UART_PORT);
    HBD_UART_CLK_ENABLE();

    // Set up the GPIOs
    GPIO_AF(HBD_UART_PORT, HBD_UART_TX_PIN, HBD_UART_AF);
    GPIO_AF(HBD_UART_PORT, HBD_UART_RX_PIN, HBD_UART_AF);

    // Initialize the peripheral
    HBD_UART->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_TE;
    HBD_UART->CR2 = 0;
    HBD_UART->CR3 = 0;
    HBD_UART->BRR = HBD_BRR;

    s_RxBuffer.Done = 0;
    s_RxBuffer.RdPos = 0;
    s_RxBuffer.WrPos = 0;
    s_TxBuffer.Done = 1;

    HBD_UART->CR1 |= USART_CR1_UE;

    // Interrupt config
    NVIC_SetPriority(HBD_IRQn, PRIO_HBD_UART);
    NVIC_EnableIRQ(HBD_IRQn);
}

int32_t HBD_Receive(void* buf, uint32_t count) {
    uint8_t* buf8b = buf;
    uint32_t buffer_remaining = 0;
    uint32_t place = 0;
    // Copy up to "count" bytes from the read buffer
    if (!s_RxBuffer.Done) {
        // No bytes read, return zero.
        return 0;
    }
    if (s_RxBuffer.WrPos <= s_RxBuffer.RdPos) {
        buffer_remaining = HBD_BUFFER_LENGTH - s_RxBuffer.RdPos
                + s_RxBuffer.WrPos;
    } else {
        buffer_remaining = s_RxBuffer.WrPos - s_RxBuffer.RdPos;
    }
    while ((place < count) && (place < buffer_remaining)) {
        buf8b[place++] = s_RxBuffer.Buffer[s_RxBuffer.RdPos++];
        if (s_RxBuffer.RdPos > HBD_BUFFER_LENGTH)
            s_RxBuffer.RdPos = 0;
    }
    // Clear "done" flag if no more bytes to read
    if (count >= buffer_remaining)
        s_RxBuffer.Done = 0;
    // Return the number of read bytes.
    return place;
}

int32_t HBD_Transmit(void* buf, uint32_t count) {
    uint8_t* buf8b = buf;
    uint32_t place = 0;
    if (s_TxBuffer.Done) {
        // In the rare chance that the transmitter is just finishing,
        // wait for the shift register to empty
        uint32_t timeout = GetTick();
        while (!(HBD_UART->SR & USART_SR_TXE)) {
            if (GetTick() > (timeout + HBD_TXMT_TIMEOUT)) {
                // Timeout fail
                return -1;
            }
        }

        // Safe to simply restart the buffer
        s_TxBuffer.RdPos = 0;
        s_TxBuffer.WrPos = 0;
        while ((place < HBD_BUFFER_LENGTH) && (place < count)) {
            s_TxBuffer.Buffer[s_TxBuffer.WrPos++] = buf8b[place++];
        }
        // Start the transfer

        HBD_UART->DR = s_TxBuffer.Buffer[s_TxBuffer.RdPos++];
        HBD_UART->CR1 |= (USART_CR1_TXEIE);
        s_TxBuffer.Done = 0;
    } else {
        // Can we fit more data in the buffer?
        uint8_t buffer_used;
        if (s_TxBuffer.WrPos <= s_TxBuffer.RdPos) {
            buffer_used = HBD_BUFFER_LENGTH - s_TxBuffer.RdPos
                    + s_TxBuffer.WrPos;
        } else {
            buffer_used = s_TxBuffer.WrPos - s_TxBuffer.RdPos;
        }
        uint8_t buffer_remaining = HBD_BUFFER_LENGTH - buffer_used;

        if (count <= buffer_remaining) {
            while ((place < buffer_remaining) && (place < count)) {
                s_TxBuffer.Buffer[s_TxBuffer.WrPos++] = buf8b[place++];
            }
        } else {
            // Buffer was full, fail.
            return -1;
        }
    }
    // Return the number of written bytes.
    return place;
}

void HBD_IRQ(void) {
    uint8_t comm_dummy;
    // Character received!
    if ((HBD_UART->SR & USART_SR_RXNE) != 0) {
        // Receive the new character
        comm_dummy = HBD_UART->DR;
        // Add it to our buffer
        s_RxBuffer.Buffer[s_RxBuffer.WrPos++] = comm_dummy;
        s_RxBuffer.Done = 1;
        if (s_RxBuffer.WrPos >= HBD_BUFFER_LENGTH) {
            // Wrap around the circular buffer
            s_RxBuffer.WrPos = 0;
        }
    }
    if (((HBD_UART->SR & USART_SR_TXE) != 0)
            && ((HBD_UART->CR1 & USART_CR1_TXEIE) != 0)) {
        // Check if more data to send
        if (((s_TxBuffer.RdPos + 1) == s_TxBuffer.WrPos)
                || ((s_TxBuffer.RdPos == HBD_BUFFER_LENGTH)
                        && (s_TxBuffer.WrPos == 0))) {
            // We are done after this byte!
            s_TxBuffer.Done = 1;
            // Turn off the transmit data register empty interrupt
            HBD_UART->CR1 &= ~(USART_CR1_TXEIE);
            // Turn on the transmit complete interrupt
            HBD_UART->CR1 |= USART_CR1_TCIE;
        }

        HBD_UART->DR = s_TxBuffer.Buffer[s_TxBuffer.RdPos++];
        if (s_TxBuffer.RdPos >= HBD_BUFFER_LENGTH) {
            // Wrap around the read pointer
            s_TxBuffer.RdPos = 0;
        }
    }
    if (((HBD_UART->SR & USART_SR_TC) != 0)
            && ((HBD_UART->CR1 & USART_CR1_TCIE) != 0)) {
        // Trasmission complete, turn off the transmitter and the interrupt
        HBD_UART->CR1 &= ~(USART_CR1_TCIE);
    }
}
