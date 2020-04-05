/******************************************************************************
 * Filename: power_calcs.c
 * Description: Calculate the total motor power based on Ialpha, Ibeta
 *              reference frame, or the total power in trapezoidal mode.
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

#include "stm32f4xx.h"
#include "arm_math.h"
#include "power_calcs.h"

// Needs some filtering on the magnitude of power and current
// Using a simple low-pass filter, about 20Hz bandwidth
#define POWER_CALCS_LPF_MULTIPLIER      (0.07f)

void power_calc(PowerCalcs* pc) {
    // From "Permanent Magnet Synchronous and Brushless DC Motors" by Ramu Krishnan
    // Chapter 3.5, Power Equivalence
    // Power in = Power out
    // For the three-phase motor, power out = Van*Ia + Vbn*Ib + Vcn*Ic
    // For a two-phase equivalent, power out = (3/2) * (Valpha*Ialpha + Vbeta*Ibeta)
    // The (3/2) comes from the three-to-two-phase conversion.

    // Calculate the voltages for each phase to neutral
    // Vx = Tx * Vbus, x = a,b,c
    // Vn = (Va + Vb + Vc)/3
    // Van = Va - Vn = (2*Va - Vb - Vc) / 3
    // ...similar for Vbn, Vcn
    // But we don't need Vcn for the balanced Clarke transform, so it's skipped
    float Van, Vbn, Vbeta;
    Van = pc->Vbus * ((2.0f * pc->Ta) - (pc->Tb) - (pc->Tc)) / 3.0f;
    Vbn = pc->Vbus * ((2.0f * pc->Tb) - (pc->Ta) - (pc->Tc)) / 3.0f;

    // Clarke transform from 3-phase to 2-phase
    // Valpha = Van (keeping the same name)
    Vbeta = (2.0f * Vbn + Van) * (ONE_OVER_SQRT3_F);

    // Calculate total power
    pc->TotalPower = (1.0f - POWER_CALCS_LPF_MULTIPLIER) * (pc->TotalPower) +
            POWER_CALCS_LPF_MULTIPLIER * ( (1.5f)*((pc->Ialpha)*Van + (pc->Ibeta)*Vbeta) );

    // Calculate battery current using Pin = Pout, and Pin = Vbattery*Ibattery
    if(pc->Vbus > 0.01f) {
        // Avoid divide by zero errors
        pc->BatteryCurrent = pc->TotalPower / pc->Vbus;
    } else {
        pc->BatteryCurrent = 0.0f;
    }

    // Also calculate the phase current using the magnitude of Ialpha & Ibeta
    pc->PhaseCurrent = (1.0f - POWER_CALCS_LPF_MULTIPLIER) * (pc->PhaseCurrent) +
            POWER_CALCS_LPF_MULTIPLIER * sqrtf(((pc->Ialpha) * (pc->Ialpha)) + ((pc->Ibeta) * (pc->Ibeta)));

}
