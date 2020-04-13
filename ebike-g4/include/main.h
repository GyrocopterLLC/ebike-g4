/******************************************************************************
 * Filename: main.h
 * Description: Combine all the necessary include files, so that most
 *              other *.c files can just include this one header.
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

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32g4xx.h"
#include "adc.h"
#include "gpio.h"
#include "periphconfig.h"
#include "pinconfig.h"
#include "project_parameters.h"
#include "pwm.h"
#include "Timer.h" // TODO: make my own version of this guy
#include "usb.h"
#include "usb_cdc.h"


// Basic definitions used in many files
#define RETVAL_OK           (1)
#define RETVAL_FAIL         (0)

#endif //__MAIN_H
