/******************************************************************************
 * Filename: cordic_sin_cos.h
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

#ifndef _CORDIC_SIN_COS_H
#define _CORDIC_SIN_COS_H

#define USE_DEFINE_CONVERSIONS  1

#define float_to_q31_def(float_input, q31_output) \
do{                                         \
    asm(    "VCVT.S32.F32 %1, %1, #31\n\t"  \
            "VMOV %0, %1"                   \
            :"=r" (q31_output)              \
            :"t" (float_input)              \
            :);                             \
}while(0)

#define q31_to_float_def(q31_input, float_output) \
do{                                     \
    asm(    "VMOV %0, %1\n\t"           \
            "VCVT.F32.S32 %0, %0, #31"  \
            :"=t" (float_output)        \
            :"r" (q31_input)            \
            :);                         \
}while(0)

#define CORDIC_CalcSinCosDeferred_Def(theta) \
do{                                         \
    int32_t fxd_input;                      \
    float_to_q31_def(theta, fxd_input);     \
    CORDIC->WDATA = fxd_input;              \
}while(0)

#define CORDIC_GetResults_Def(sin, cos) \
do{                                 \
    int32_t fxd_sin, fxd_cos;       \
    fxd_sin = CORDIC->RDATA;        \
    fxd_cos = CORDIC->RDATA;        \
    q31_to_float_def(fxd_sin, sin); \
    q31_to_float_def(fxd_cos, cos); \
}while(0)

void CORDIC_Init(void);
void CORDIC_CalcSinCos(float theta, float* sin, float* cos) ;
void CORDIC_CalcSinCosDeferred(float theta);
void CORDIC_GetResults(float* sin, float* cos);

#endif
