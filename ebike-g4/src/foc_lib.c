/******************************************************************************
 * Filename: foc_lib.c
 * Description: Field Oriented Control Software Library
 * Contents:
 *-- Space Vector Modulation (SVM)
 *    Calculates three-phase duty cycles (A, B, and C) for a two-phase
 *    stationary input (alpha and beta).
 *-- Biquadratic Filter
 *    Performs biquad filtering on arbitrary floating point input signals.
 *    Helper function to calculate biquad parameters for a low-pass filter is
 *    also provided.
 *-- Field-Oriented Control (FOC) functions
 *    Performs Clarke, Park, and inverse Park transforms.
 *-- Proportional-Integral-Derivative (PID) Controller
 *    Functions to implement a PID feedback control system. Calculations with
 *    and without derivative control are available. Also reset and default
 *    functions to clear out the PID saved values.
 *-- Ramp generator (dfsl_rampgen, dfsl_rampctrl)
 *    Provides a fixed frequency ramp signal that wraps around at the limit of
 *    a 16-bit integer. The control function (dfsl_rampctrl) helps set the
 *    frequency of the ramp.
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

void FOC_SVM(float alpha, float beta, float* tA, float* tB, float* tC) {
    // Sector determination
    uint8_t sector = 0;
    float X, Y, Z, T1, T2;
    X = beta;
    Y = (-ONE_HALF) * beta - SQRT3_OVER_2 * alpha;
    Z = (-ONE_HALF) * beta + SQRT3_OVER_2 * alpha;

    if (X > 0)
        sector += 1;
    if (Y > 0)
        sector += 2;
    if (Z > 0)
        sector += 4;

    switch (sector) {
    case 5: // Sector 1
        T1 = Z;
        T2 = X;
        *tC = (ONE_HALF * (1 - T1 - T2));
        *tB = *tC + T2;
        *tA = *tB + T1;
        break;
    case 1: // Sector 2
        T1 = -Y;
        T2 = -Z;
        *tC = (ONE_HALF * (1 - T1 - T2));
        *tA = *tC + T1;
        *tB = *tA + T2;
        break;
    case 3: // Sector 3
        T1 = X;
        T2 = Y;
        *tA = (ONE_HALF * (1 - T1 - T2));
        *tC = *tA + T2;
        *tB = *tC + T1;
        break;
    case 2: // Sector 4
        T1 = -Z;
        T2 = -X;
        *tA = (ONE_HALF * (1 - T1 - T2));
        *tB = *tA + T1;
        *tC = *tB + T2;
        break;
    case 6: // Sector 5
        T1 = Y;
        T2 = Z;
        *tB = (ONE_HALF * (1 - T1 - T2));
        *tA = *tB + T2;
        *tC = *tA + T1;
        break;
    case 4: // Sector 6
        T1 = -X;
        T2 = -Y;
        *tB = (ONE_HALF * (1 - T1 - T2));
        *tC = *tB + T1;
        *tA = *tC + T2;
        break;
    default:
        *tA = ONE_HALF;
        *tB = ONE_HALF;
        *tC = ONE_HALF;
        break;
    }
}

void FOC_Ipark(float D, float Q, float sin, float cos, float* alpha, float* beta) {
    *alpha = D * cos - Q * sin;
    *beta = Q * cos + D * sin;
}

void FOC_Clarke(float A, float B, float* Alpha, float* Beta) {
    *Alpha = A;
    *Beta = (2.0f * B + A) * INV_SQRT3;
}

void FOC_Park(float alpha, float beta, float sin, float cos, float* D, float* Q) {
    *D = alpha * cos + beta * sin;
    *Q = beta * cos - alpha * sin;
}

void FOC_BiquadCalc(Biquad_Type* biq) {
    // Calculate intermediate value
    float intermed = (biq->X) - (biq->U1 * biq->A1) - (biq->U2 * biq->A2);
    // Calculate output value
    biq->Y = (intermed * biq->B0) + (biq->U1 * biq->B1) + (biq->U2 * biq->B2);
    // Update stored values
    biq->U2 = biq->U1;
    biq->U1 = intermed;
}

void FOC_BiquadLPF(Biquad_Type* biq, float Fs, float f0, float Q) {
    // Intermediate values
    float w0, sinw0, cosw0, alpha;
    // Cancel operation if inputs are invalid
    if ((Fs == 0.0f) || (f0 == 0.0f) || (Q == 0.0f)) {
        return;
    }
    // Calculate the intermediates
    w0 = 2.0f * (f0 / Fs); // Leaving out PI, the CORDIC input is scaled by 1/PI anyway
    // Convert angle from 0->2pi to -pi->+pi
    if(w0 > 1.0f) w0 = w0 - 2.0f;
    CORDIC_CalcSinCosDeferred_Def(w0);
    CORDIC_GetResults_Def(sinw0, cosw0);
//    CORDIC_CalcSinCos(w0, &sinw0, &cosw0);
    alpha = sinw0 / (2.0f * Q);

    // Calculate the filter constants
    biq->B0 = (1.0f - cosw0) / (2.0f) / (1.0f + alpha);
    biq->B1 = (1.0f - cosw0) / (1.0f + alpha);
    biq->B2 = biq->B0;
    biq->A1 = ((-2.0f) * cosw0) / (1.0f + alpha);
    biq->A2 = (1.0f - alpha) / (1.0f + alpha);

}

void FOC_PIDdefaults(PID_Type* pid) {
    pid->Err = 0.0f;
    pid->Ui = 0.0f;
    pid->Kp = DFLT_FOC_KP;
    pid->Ki = DFLT_FOC_KI;
    pid->Kd = DFLT_FOC_KD;
    pid->Kc = DFLT_FOC_KC;
    pid->OutMin = -0.99f;
    pid->OutMax = 0.99f;
    pid->SatErr = 0.0f;
    pid->Out = 0.0f;
    pid->Up1 = 0.0f;
}
void FOC_PIDreset(PID_Type* pid) {
    pid->Err = 0.0f;
    pid->Ui = 0.0f;
    pid->SatErr = 0.0f;
    pid->Out = 0.0f;
    pid->Up1 = 0.0f;
}

void FOC_PIDcalc(PID_Type* pid) {
    float OutPreSat, Up, Ud;
    Up = pid->Err * pid->Kp;
    pid->Ui = pid->Ui + Up * pid->Ki + pid->Kc * pid->SatErr;
    Ud = pid->Kd * (Up - pid->Up1);
    OutPreSat = Up + pid->Ui + Ud;
    if (OutPreSat > pid->OutMax) {
        pid->Out = pid->OutMax;
    } else if (OutPreSat < pid->OutMin) {
        pid->Out = pid->OutMin;
    } else {
        pid->Out = OutPreSat;
    }
    pid->SatErr = pid->Out - OutPreSat;
    pid->Up1 = Up;
}

void FOC_PIcalc(PID_Type* pid) {
    float OutPreSat, Up;
    Up = pid->Err * pid->Kp;
    pid->Ui = pid->Ui + Up * pid->Ki + pid->Kc * pid->SatErr;
    OutPreSat = Up + pid->Ui;
    if (OutPreSat > pid->OutMax) {
        pid->Out = pid->OutMax;
    } else if (OutPreSat < pid->OutMin) {
        pid->Out = pid->OutMin;
    } else {
        pid->Out = OutPreSat;
    }
    pid->SatErr = pid->Out - OutPreSat;
}

/**
 * @brief  Creates a ramping output, wrapping around at +/-1.0
 * @param  rampAngle: current output value, passed in by reference and incremented
 * @param  rampInc: amount to increment / decrement the angle by
 * @retval None
 */
void FOC_RampGen(float* rampAngle, float rampInc) {
    *rampAngle += rampInc;
    if (*rampAngle > 1.0f) {
        *rampAngle -= 1.0f;
    }
    if (*rampAngle < 0.0f) {
        *rampAngle += 1.0f;
    }
}

/**
 * @brief  Calculates the correct rampInc amount to be used in FOC_RampGen
 * @param  callingFreq: frequency in Hz that FOC_RampGen will be called
 * @param  rampFreq: frequency of desired ramp output
 * @retval The calculated increment amount.
 */
float FOC_RampCtrl(float callingFreq, float rampFreq) {
    return rampFreq / callingFreq;
}


