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

void power_calc(PowerCalcs* pc) {
    // Step 1 - calculate the voltages for each phase to neutral
    // Vx = Tx * Vbus, x = a,b,c
    // Vn = (Va + Vb + Vc)/3
    // Van = Va - Vn = (2*Va - Vb - Vc) / 3
    // ...similar for Vbn, Vcn
    float Van, Vbn, Vbeta, phaseVolts;
    Van = pc->Vbus * ((2.0f * pc->Ta) - (pc->Tb) - (pc->Tc)) / 3.0f;
    Vbn = pc->Vbus * ((2.0f * pc->Tb) - (pc->Ta) - (pc->Tc)) / 3.0f;

    // Step 2 - Clarke transform from 3-phase to 2-phase
    // Valpha = Van (keeping the same name)
    Vbeta = (2.0f * Vbn + Van) * (ONE_OVER_SQRT3_F);

    // Step 3 - Calculate magnitude of phase current
    // I_mag = (3/2) * sqrt(Ialpha^2 + Ibeta^2)
    pc->PhaseCurrent = 1.5f
            * sqrtf((pc->Ialpha) * (pc->Ialpha) + (pc->Ibeta) * (pc->Ibeta));

    // Step 4 - Calculate magnitude of phase voltage
    phaseVolts = sqrtf(Van * Van + Vbeta * Vbeta);

    // Step 5 - Calculate total power
    pc->TotalPower = phaseVolts * pc->PhaseCurrent;

    // Step 6 - Calculate battery current
    // Conservation of power:
    //      Total power = V_phase*I_phase = V_battery*I_battery
    pc->BatteryCurrent = pc->TotalPower / pc->Vbus;

}
