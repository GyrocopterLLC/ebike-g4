/******************************************************************************
 * Filename: wdt.c
 * Description: Functions for the STM32G4 watchdog timer.
 *              The independent watchdog (IWDG) is used, which has its clock
 *              input fixed to the internal low speed oscillator. Using the
 *              prescaler and auto-reload registers, the reset timeout is set
 *              to approximately 50ms. If this time duration passes without
 *              the KEY register being written with the reload command, the
 *              MCU resets.
 *              Once started, the watchdog cannot be stopped.
 *              Feed the watchdog!
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

void WDT_Init(void) {
    // Timer is stopped in debug (e.g. breakpoints)
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
    // Start/enable the countdown timer
    IWDG->KR = IWDG_START;
    // Unlock registers
    IWDG->KR = IWDG_KEY;
    // Set prescaler
    IWDG->PR = IWDG_PSC;
    // Set reload value
    IWDG->RLR = IWDG_REL_VAL;
    // Wait for it all to take effect
    // This gets stuck. I think it needs to get a reload before it updates?
    // I don't really care about waiting for it to take effect.
    // If the reload register ends up being stuck at default, the IWDG will still work
    // but will reset after ~510 msec instead of 50 msec.
//    while((IWDG->SR) != ((uint32_t)0)) {
        // pass
//    }

}

void WDT_Feed(void) {
    IWDG->KR = IWDG_RELOAD;
}
