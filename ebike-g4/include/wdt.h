/******************************************************************************
 * Filename: wdt.h
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
// IWDG
#ifndef WDT_H_
#define WDT_H_

#define IWDG_RELOAD     ((uint32_t)0x00005555u)
#define IWDG_KEY        ((uint32_t)0x00005555u)
#define IWDG_START      ((uint32_t)0x0000CCCCu)

#define IWDG_PSC        ((uint32_t)0x00000000u) // For about 125usec granularity (32kHz/4)
#define IWDG_REL_VAL    ((uint32_t)400) // 125usec * 400 = 50msec

void WDT_init(void);
void WDT_feed(void);

#endif //WDT_H_
