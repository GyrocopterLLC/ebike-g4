/******************************************************************************
 * Filename: crc32.c
 * Description: Uses the STM32 built-in CORDIC co-processor to calculate sin
 *              and cos of an input angle.
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
 * @brief  Initializes the CORDIC peripheral. Sin/Cos mode is used.
 */
void CORDIC_Init(void) {
    volatile uint32_t dummy;
    // Turn on the clock
    RCC->AHB1ENR |= RCC_AHB1ENR_CORDICEN;

    // Initialize the needed function (sin/cos)
    // Also pump in one dummy calculation to set the second input variable (modulus m)
    // to one, which won't be changed in later calculations
    CORDIC->CSR = CORDIC_CSR_NARGS // 2 input arguments
                | CORDIC_CSR_NRES // 2 output arguments
                | CORDIC_CSR_PRECISION_0 | CORDIC_CSR_PRECISION_2 // 5 cycles, 20 iterations, error < 2^-18
                | CORDIC_CSR_FUNC_0; // Function 1 - sine


    // Enter in the dummy calculation
    CORDIC->WDATA = 0x80000000u; // Arg 1 = angle, about 0.5, which means pi/2
    CORDIC->WDATA = 0x7FFFFFFFu; // Arg 2 = modulus, 1. This won't be changed again later.

    // Read out the results.
    dummy = CORDIC->RDATA;
    (void)dummy; // So compiler doesn't complain about "set but not used"
    dummy = CORDIC->RDATA;
    (void)dummy;
    // Can change to single input argument now.
    CORDIC->CSR &= ~(CORDIC_CSR_NARGS);

}

int32_t float_to_q31(float input) {
    int32_t retval;
    asm(    "VCVT.S32.F32 %1, %1, #31\n\t"
            "VMOV %0, %1"
            :"=r" (retval)
            :"t" (input)
            :);
    return retval;
}

float q31_to_float(int32_t input) {
    float retval;
    asm(    "VMOV %0, %1\n\t"
            "VCVT.F32.S32 %0, %0, #31"
            :"=t"(retval)
            :"r"(input)
            :);
    return retval;
}

/**
 * @brief  Calculates sin(theta) and cos(theta) using the CORDIC peripheral.
 * @param  theta: input angle in range [-1,1), which is scaled to [-pi, pi)
 * @param  sin: pointer to sin(theta) result
 * @param  cos: pointer to cos(theta) result
 * @retval None
 */
void CORDIC_CalcSinCos(float theta, float* sin, float* cos) {
    int32_t fxd_input, fxd_sin, fxd_cos;
//    fxd_input = (int32_t) (theta * 2147483648.0f); // Multiply by 0x80000000 = 2147483648
    fxd_input = float_to_q31(theta);

    CORDIC->WDATA = fxd_input;

    // Get the results
    fxd_sin = CORDIC->RDATA; // Inserts wait states until result is ready
    fxd_cos = CORDIC->RDATA;

//    *sin = ((float)fxd_sin) / (2147483648.0f);
//    *cos = ((float)fxd_cos) / (2147483648.0f);
    *sin = q31_to_float(fxd_sin);
    *cos = q31_to_float(fxd_cos);
}

/**
 * @brief  Launches sin(theta) and cos(theta) calculation, result read later with CORDIC_GetResult.
 * @param  theta: input angle in range [-1,1), which is scaled to [-pi, pi)
 * @retval None
 */
void CORDIC_CalcSinCosDeferred(float theta) {
    int32_t fxd_input;
    fxd_input = float_to_q31(theta);
//    fxd_input = (int32_t) (theta * 1073741824.0f); // Multiply by 0x80000000
    CORDIC->WDATA = fxd_input;
}

/**
 * @brief  Gets the results of a previous CORDIC_CalcSinCosDeferred function call.
 * @param  sin: pointer to sin(theta) result
 * @param  cos: pointer to cos(theta) result
 * @retval None
 */
void CORDIC_GetResults(float* sin, float* cos) {
    int32_t fxd_sin, fxd_cos;
    fxd_sin = CORDIC->RDATA; // Inserts wait states until result is ready
    fxd_cos = CORDIC->RDATA;

    *sin = q31_to_float(fxd_sin);
    *cos = q31_to_float(fxd_cos);
//    *sin = ((float)fxd_sin) / (1073741824.0f);
//    *cos = ((float)fxd_cos) / (1073741824.0f);
}
