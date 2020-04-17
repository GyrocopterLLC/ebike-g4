/******************************************************************************
 * Filename: interrupts.c
 * Description: Interrupt request handlers, aka interrupt service routines
 *              (ISRs). When a processor interrupt is triggered, code execution
 *              immediately jumps to the associated handler where software
 *              clears the interrupt source, preventing the handler from being
 *              re-triggered, and performs any necessary tasks.
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

#include "main.h"

/**
 * Handlers for System events
 * Most of the default handlers are used for things like hard fault, etc
 * Only SysTick handler is customized.
 */

void SysTick_Handler (void) {
    // No flags to clear. Jump straight to the handler in delay.c
    SYSTICK_IRQHandler();
}

/**
 * Handlers for ADC.
 * The active interrupts are:
 * - ADC1 injected end of conversion (JEOC)
 * - ADC3 injected end of conversion (JEOC)
 * - DMA1 Channel 1 end of transfer (TCIF)
 * - DMA1 Channel 2 end of transfer (TCIF)
 */

void ADC1_2_IRQHandler(void) {
    if((ADC1->ISR & ADC_ISR_JEOC) != 0) {
        // Injected end of conversion
        ADC1->ISR |= ADC_ISR_JEOC; // Clear the flag by writing 1 to it
    }
}

void ADC3_IRQHandler(void) {
    if((ADC3->ISR & ADC_ISR_JEOC) != 0) {
        // Injected end of conversion
        ADC3->ISR |= ADC_ISR_JEOC; // Clear the flag by writing 1 to it
    }
}

void DMA1_Channel1_IRQHandler(void) {
    if((DMA1->ISR & DMA_ISR_TCIF1) != 0) {
        // Transfer complete channel 1
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Clear the flag by writing 1
    }
}

void DMA1_Channel2_IRQHandler(void) {
    if((DMA1->ISR & DMA_ISR_TCIF2) != 0) {
        // Transfer complete channel 1
        DMA1->IFCR |= DMA_IFCR_CTCIF2; // Clear the flag by writing 1
    }
}

/**
 * Handlers for Hall sensor timer.
 * Two interrupts are used:
 * - TIM4 Update (UIF)
 * - TIM4 Channel 1 Capture (CC1F)
 */

void TIM4_IRQHandler(void) {
    if((TIM4->SR & TIM_SR_UIF) != 0) {
        // Update (timer rollover)
        TIM4->SR &= ~(TIM_SR_UIF); // Clear the flag by writing 0
        HALL_UpdateCallback();
    }
    if((TIM4->SR & TIM_SR_CC1IF) != 0) {
        // Capture (Hall state change)
        TIM4->SR &= ~(TIM_SR_CC1IF); // Clear the flag by writing 0
        HALL_CaptureCallback();
    }

}

/**
 * Handlers for PWM timer
 * Two interrupts used:
 * - TIM1 Update (UIF)
 * - TIM1 Break input (BIF)
 */

void TIM1_UP_TIM16_IRQHandler(void) {
    if((TIM1->SR & TIM_SR_UIF) != 0) {
        // Update (timer rollover)
        TIM1->SR &= ~(TIM_SR_UIF); // Clear the flag by writing 0
    }
}

void TIM1_BRK_TIM15_IRQHandler(void) {
    if((TIM1->SR & TIM_SR_BIF) != 0) {
        // Break interrupt
        TIM1->SR &= ~(TIM_SR_BIF); // Clear the flag by writing 0
    }
}

/**
 * Handlers for UART
 * Two interrupts used:
 * - USART2 (HBD UART) receiver not empty (RXNE), transmitter empty (TXE)
 * - USART3 (BMS UART) receiver not empty (RXNE), transmitter empty (TXE)
 */

void USART2_IRQHandler(void) {
    // RXNE is cleared automatically when the data register is read
    // TXE is cleared automatically when the data register is written
    UART_IRQ(SELECT_HBD_UART);
}

void USART3_IRQHandler(void) {
    // RXNE is cleared automatically when the data register is read
    // TXE is cleared automatically when the data register is written
    UART_IRQ(SELECT_BMS_UART);
}


/**
 * Handlers for USB
 * One interrupt handler, but many sources are activated:
 * - Correct Transfer (CTRM)
 * - Wakeup (WKUPM)
 * - Suspend (SUSPM)
 * - Error (ERRM)
 * - Start of Frame (SOFM)
 * - Expected Start of Frame (ESOFM)
 * - Reset (RESETM)
 * - Low power request (L1REQM)
 */
void USB_LP_IRQHandler(void) {
    // Simply call the USB driver function.
    // It takes care of all the flag clearing and handling.
    USB_IRQ();
}

void USB_HP_IRQHandler(void) {
    // Not sure why there are two IRQ handlers for USB.
    // It isn't mentioned in the datasheet.
    USB_IRQ();
}
