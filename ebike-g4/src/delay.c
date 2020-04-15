/******************************************************************************
 * Filename: delay.c
 * Description: Provides an easy millisecond delay routine. Also includes a
 *              tick counter of milliseconds elapsed since power on, or since
 *              the last 32-bit counter rollover (about 4.3 million seconds,
 *              or 1,193 hours)
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

uint32_t g_SysTick;

void DelayInit(void) {
    g_SysTick = 0;
    SysTick_Config(SYS_CLK / 1000u);
    NVIC_SetPriority(SysTick_IRQn, PRIO_SYSTICK);
}

void Delay(__IO uint32_t Delay) {
    uint32_t tickstart = 0;
    tickstart = g_SysTick;
    while ((g_SysTick - tickstart) < Delay) {
    }
}

uint32_t GetTick(void) {
    return g_SysTick;
}

void SYSTICK_IRQHandler(void) {
    g_SysTick++;
}
