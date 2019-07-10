/******************************************************************************
 * Filename: debug_dump.c
 * Description: Assigns all of the Core-coupled memory (CCM) in the STM32F4
 *              processor for saving debug data. This function stores values
 *              at the same rate as the PWM calculations. This is a little too
 *              fast for the serial port transfer to keep up with, so the data
 *              is saved and sent out in batches when requested later.
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

#ifdef DEBUG_DUMP_USED

#include "stm32f4xx.h"
#include "debug_dump.h"

int16_t DumpData[MAX_DUMP_LENGTH] __attribute__((section(".bss_CCMRAM")));
uint16_t DumpLoc;
uint8_t dumpassignments[MAX_DUMP_OUTPUTS] = DEFAULT_DUMP_ASSIGNMENTS;

void DumpReset(void)
{
    DumpLoc = 0;
}

uint8_t DumpGeneration(float* outputvals)
{
    if(DumpLoc < MAX_DUMP_OUTPUTS)
    {
        DumpData[DumpLoc] = (int16_t)(usbdacvals[dumpassignments[0]-1]*1638.4f);
        DumpData[DumpLoc+1] = (int16_t)(usbdacvals[dumpassignments[1]-1]*1638.4f);
        DumpData[DumpLoc+2] = (int16_t)(usbdacvals[dumpassignments[2]-1]*1638.4f);
        DumpData[DumpLoc+3] = (int16_t)(usbdacvals[dumpassignments[3]-1]*1638.4f);
        DumpLoc+=4;
        return 0;
    }
    else
    {
        DumpLoc = 0;
        return 1;
    }
}

int16_t GetDumpData(void)
{
    if(DumpLoc < MAX_DUMP_OUTPUTS)
    return DumpData[DumpLoc++];
}

uint8_t DumpDone(void)
{
    if(DumpLoc >= MAX_DUMP_OUTPUTS)
    return 1;
    return 0;
}

#endif // DEBUG_DUMP_USED
