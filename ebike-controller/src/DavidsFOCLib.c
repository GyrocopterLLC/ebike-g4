/******************************************************************************
 * Filename: DavidsFOCLib.c
 * Description: David's Field Oriented Control Software Library
 * Contents:
 * 1. Space Vector Modulate (dfsl_svm, dfsl_svmf)
 *    Calculates three-phase duty cycles (A, B, and C) for a two-phase
 *    stationary input (alpha and beta). Fixed- and floating-point math
 *    versions are provided.
 * 2. Biquadratic Filter (dfsl_biquadf, dfsl_biquadcalc_lpf)
 *    Performs biquad filtering on arbitrary floating point input signals.
 *    Helper function to calculate biquad parameters for a low-pass filter is
 *    also provided.
 * 3. Field-Oriented Control (FOC) functions
 *    (dfsl_clarke, dfsl_clarkef, dfsl_park, dfsl_parkf, dfsl_ipark,
 *    dfsl_iparkf)
 *    Performs Clarke, Park, and inverse Park transforms. Fixed- and floating-
 *    point versions available.
 * 4. Proportional-Integral-Derivative (PID) Controller
 *    (dfsl_pid, dfsl_pidf, dfsl_pi, dfsl_pif)
 *    Functions to implement a PID feedback control system. Calculations with
 *    and without derivative control, and in either fixed- or floating-point are
 *    available.
 * 5. Helper functions for PID control
 *    (dfsl_pid_defaults, dfsl_pid_reset, dfsl_pid_defaultsf, dfsl_pid_resetf)
 *    Set constants to default values, or reset integrator/derivative. In both
 *    fixed- and floating-point.
 * 6. Ramp generator (dfsl_rampgen, dfsl_rampctrl)
 *    Provides a fixed frequency ramp signal that wraps around at the limit of
 *    a 16-bit integer. The control function (dfsl_rampctrl) helps set the
 *    frequency of the ramp.
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

#include "DavidsFOCLib.h"

/** Helper math functions/defines **/

#define Q16_mpy(A, B)   (((A)*(B))>>16)
#define Q_mpy(A, B)     (int32_t)((((int64_t)(A)) * ((int64_t)(B))) >> Q_FACTOR)
/** Function Definitions **/


/**
 * Chebyshev 5th order approximation for sine
 * Input is clipped to between +pi and -pi
 * The approximation is actually only valid between 0 and pi/2
 * But since sine is so wonderfully periodic, we can make adjustments
 * input between 0 and pi/2 -> use approximation
 * input between pi/2 and pi -> approximation of {pi/2 - (input - pi/2)} = pi - input
 * input less than 0: use positive input value, use the above two sections, and negate the output
 */
static float _dfsl_sinf_chebyshev(float theta) {
    // Reference: https://www.embeddedrelated.com/showarticle/152.php
    // Limits:
    // thetamin = 0, thetamax = 1.570796 (Pi/2)
    // Coefficients:
    // c0 = 0.6021947012555071,
    // c1 = 0.5136251666803038,
    // c2 = -0.10354634422944732,
    // c3 = -0.013732035086651804,
    // c4 = 0.0013586503384922695,
    // c5 = 0.00010765948465639574
    //
    float coeffs[6] = {0.6021947012555071f,
            0.5136251666803038f,
            -0.10354634422944732f,
            -0.013732035086651804f,
            0.0013586503384922695f,
            0.00010765948465639574f
    };
    // U = (2*theta - min - max) / (max - min)
    // U = (2*theta - PI/2) / (PI/2)
    // U = (theta - pi/4) / (pi/4)
    float u = (theta - PI_OVER_4) * FOUR_OVER_PI;
    float Tprev = 1;
    float T1 = u;
    float Tnext;
    float Y = coeffs[0];
    for(uint8_t i = 1; i <= 5; i++) {
        Y += T1*coeffs[i];
        Tnext = (2.0f)*u*T1-Tprev;
        Tprev = T1;
        T1 = Tnext;
    }
    return Y;
}

float dfsl_sinf(float theta) {
    float sinval = 0.0f;
    // Clip to [-pi, +pi]
    while(theta > PI) {
        theta -= PI;
    }
    while (theta < -PI) {
        theta += PI;
    }

    // Map the entire -pi to +pi to between 0 and pi/2
    if(theta >= 0) {
        if(theta < PI_OVER_2) {
            sinval = _dfsl_sinf_chebyshev(theta);
        } else {
            sinval = _dfsl_sinf_chebyshev(PI - theta);
        }
    } else {
        theta = -theta;
        if(theta < PI_OVER_2) {
            sinval = -_dfsl_sinf_chebyshev(theta);
        } else {
            sinval = -_dfsl_sinf_chebyshev(PI - theta);
        }
    }
    return sinval;
}

float dfsl_cosf(float theta) {
    float cosval = 0.0f;
    // clip to [-pi, +pi]
    while(theta > PI) {
        theta -= PI;
    }
    while(theta < -PI) {
        theta += PI;
    }

    // Move things around so we can use sin instead
    if(theta < -PI_OVER_2) {
        cosval = -_dfsl_sinf_chebyshev(-theta - PI_OVER_2);
    }
    if((theta >= -PI_OVER_2) && (theta < 0.0f)) {
        cosval = _dfsl_sinf_chebyshev(theta + PI_OVER_2);
    }
    if((theta >= 0.0f) && (theta < PI_OVER_2)) {
        cosval = _dfsl_sinf_chebyshev(-theta + PI_OVER_2);
    }
    if(theta >= PI_OVER_2) {
        cosval = -_dfsl_sinf_chebyshev(theta - PI_OVER_2);
    }
    return cosval;
}

void dfsl_rampgen(uint16_t* rampAngle, int16_t rampInc) {
    *rampAngle += rampInc;
}

int16_t dfsl_rampctrl(uint32_t callingFreq, uint32_t rampFreq) {
    int64_t temp = rampFreq << 16;    // convert whole number frequency to q16
    temp = temp / callingFreq; // Should be less than one, that's why the previous conversion to q16
    return (int16_t) temp;
}

void dfsl_rampgenf(float* rampAngle, float rampInc) {
    *rampAngle += rampInc;
    if (*rampAngle > 1.0f) {
        *rampAngle -= 1.0f;
    }
    if (*rampAngle < 0.0f) {
        *rampAngle += 1.0f;
    }
}

float dfsl_rampctrlf(float callingFreq, float rampFreq) {
    return rampFreq / callingFreq;
}

void dfsl_pid_defaults(PID_Type* pid) {
    pid->Err = 0;
    pid->Ui = 0;
    pid->Kp = FOC_KP;
    pid->Ki = FOC_KI;
    pid->Kd = FOC_KD;
    pid->Kc = FOC_KC;
    pid->OutMin = FOC_OUTMIN;
    pid->OutMax = FOC_OUTMAX;
    pid->SatErr = 0;
    pid->Out = 0;
    pid->Up1 = 0;
}
void dfsl_pid_reset(PID_Type* pid) {
    pid->Err = 0;
    pid->Ui = 0;
    pid->SatErr = 0;
    pid->Out = 0;
    pid->Up1 = 0;
}
void dfsl_pid_defaultsf(PID_Float_Type* pid) {
    pid->Err = 0.0f;
    pid->Ui = 0.0f;
    pid->Kp = FOC_KP_F;
    pid->Ki = FOC_KI_F;
    pid->Kd = FOC_KD_F;
    pid->Kc = FOC_KC_F;
    pid->OutMin = FOC_OUTMIN_F;
    pid->OutMax = FOC_OUTMAX_F;
    pid->SatErr = 0.0f;
    pid->Out = 0.0f;
    pid->Up1 = 0.0f;
}
void dfsl_pid_resetf(PID_Float_Type* pid) {
    pid->Err = 0.0f;
    pid->Ui = 0.0f;
    pid->SatErr = 0.0f;
    pid->Out = 0.0f;
    pid->Up1 = 0.0f;
}

/**
 * dfsl_pid
 * Proportional-integral-derivative feedback controller.
 * Generates an output based on past and present input values, and past output values.
 */
void dfsl_pid(PID_Type* pid) {
    int32_t OutPreSat, Up, Ud;
    Up = Q_mpy(pid->Err, pid->Kp);
    pid->Ui = pid->Ui + Q_mpy(Up, pid->Ki) + Q_mpy(pid->Kc, pid->SatErr);
    Ud = Q_mpy(pid->Kd, (Up - pid->Up1));
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

/**
 * dfsl_pidf
 * Proportional-integral-derivative feedback controller.
 * Generates an output based on past and present input values, and past output values.
 */
void dfsl_pidf(PID_Float_Type* pid) {
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

/**
 * dfsl_pi
 * Proportional-integral feedback controller.
 * Generates an output based on past and present input values, and past output values.
 * Same as dfsl_pi, but without the derivative term.
 */
void dfsl_pi(PID_Type* pid) {
    int32_t OutPreSat, Up;
    Up = Q_mpy(pid->Err, pid->Kp);
    pid->Ui = pid->Ui + Q_mpy(Up, pid->Ki) + Q_mpy(pid->Kc, pid->SatErr);
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
 * dfsl_pif
 * Proportional-integral feedback controller.
 * Generates an output based on past and present input values, and past output values.
 * Same as dfsl_pidf, but without the derivative term.
 */
void dfsl_pif(PID_Float_Type* pid) {
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
 * dfsl_biquadf
 * Implements a biquad filter in the Type II direct form.
 */
void dfsl_biquadf(Biquad_Float_Type* biq) {
    // Calculate intermediate value
    float intermed = (biq->X) - (biq->U1 * biq->A1) - (biq->U2 * biq->A2);
    // Calculate output value
    biq->Y = (intermed * biq->B0) + (biq->U1 * biq->B1) + (biq->U2 * biq->B2);
    // Update stored values
    biq->U2 = biq->U1;
    biq->U1 = intermed;
}

/**
 * dfsl_biquadcalc_LPF
 * Calculates the constants for a biquad filter.
 * Filter type: low-pass filter (LPF)
 * Inputs:
 * -- Fs = sample rate
 * -- f0 = cutoff frequency
 * -- Q = quality factor
 */
void dfsl_biquadcalc_lpf(Biquad_Float_Type* biq, float Fs, float f0, float Q) {
    // Intermediate values
    float w0, sinw0, cosw0, alpha;
    // Cancel operation if inputs are invalid
    if ((Fs == 0.0f) || (f0 == 0.0f) || (Q == 0.0f)) {
        return;
    }
    // Calculate the intermediates
    w0 = 2.0f * PI * (f0 / Fs);
    arm_sin_cos_f32(w0 * (180.0f / PI), &sinw0, &cosw0);
    alpha = sinw0 / (2.0f * Q);

    // Calculate the filter constants
    biq->B0 = (1.0f - cosw0) / (2.0f) / (1.0f + alpha);
    biq->B1 = (1.0f - cosw0) / (1.0f + alpha);
    biq->B2 = biq->B0;
    biq->A1 = ((-2.0f) * cosw0) / (1.0f + alpha);
    biq->A2 = (1.0f - alpha) / (1.0f + alpha);

}

/** 
 * dfsl_svm
 * Implmentation of Space Vector Modulation using fixed-point math.
 * All variables are expressed in q16 format: 16 bits of whole number, 16 bits
 * after the decimal.
 * q16 max unsigned value: 4294967295 / 65536 = 65535.999847
 * q16 max signed value: 2147483647 / 65536 = 32767.9998474
 * q16 min (non-zero) value: 1/65536 = .000015259
 *
 * All returned values (tA, tB, tC) range from 0->1 (in signed Q16 that's 0->65535)
 */
void dfsl_svm(int16_t alpha, int16_t beta, int32_t* tA, int32_t* tB,
        int32_t* tC) {

    // Sector determination
    uint8_t sector = 0;
    int32_t X, Y, Z, T1, T2;
    X = beta;
    Y = Q16_mpy(-ONE_HALF_Q16,beta) - Q16_mpy(SQRT3_OVER_2_Q16, alpha);
    Z = Q16_mpy(-ONE_HALF_Q16,beta) + Q16_mpy(SQRT3_OVER_2_Q16, alpha);

    /** Determining the sector
     *
     *       beta
     *        ^
     *        |
     *        |
     *  _____________> alpha
     *        |
     *        |
     *        |
     *
     *        X
     *     \  |  /
     *      \ | /
     *       \|/
     *       /|\
 *      / | \
 *     /  |  \
 *    Z       Y
     *
     * If X is positive (beta is positive), sector is in the upper half plane
     * If Y is positive, sector is in the lower right side
     * If Z is positive, sector is in the lower left side
     *
     *
     *    \       /
     *     \  1  /
     *      \   /
     *  __5_______3___
     *    4 /   \ 2
     *     /  6  \
     *    /       \
 */

    if (X > 0)
        sector += 1;
    if (Y > 0)
        sector += 2;
    if (Z > 0)
        sector += 4;
    /* Mapping....
     * Sector code -> physical sector
     *         5 -> 1
     *         1 -> 2
     *         3 -> 3
     *         2 -> 4
     *         6 -> 5
     *         4 -> 6
     * The remaining assignments determine how much time
     * in the two nearest space vectors (T1 and T2) are
     * required to create the same vector as described by
     * the inputs, alpha and beta. Then, using the space
     * vector definitions (ie S001 = A+, B-, C-) the on-times
     * for phases A, B, and C are calculated.
     */

    switch (sector) {
    case 5: // Sector 1
        T1 = Z;
        T2 = X;
        *tC = Q16_mpy(ONE_HALF_Q16, (1 - T1 - T2));
        *tB = *tC + T2;
        *tA = *tB + T1;
        break;
    case 1: // Sector 2
        T1 = -Y;
        T2 = -Z;
        *tC = Q16_mpy(ONE_HALF_Q16, (1 - T1 - T2));
        *tA = *tC + T1;
        *tB = *tA + T2;
        break;
    case 3: // Sector 3
        T1 = X;
        T2 = Y;
        *tA = Q16_mpy(ONE_HALF_Q16, (1 - T1 - T2));
        *tC = *tA + T2;
        *tB = *tC + T1;
        break;
    case 2: // Sector 4
        T1 = -Z;
        T2 = -X;
        *tA = Q16_mpy(ONE_HALF_Q16, (1 - T1 - T2));
        *tB = *tA + T1;
        *tC = *tB + T2;
        break;
    case 6: // Sector 5
        T1 = Y;
        T2 = Z;
        *tB = Q16_mpy(ONE_HALF_Q16, (1 - T1 - T2));
        *tA = *tB + T2;
        *tC = *tA + T1;
        break;
    case 4: // Sector 6
        T1 = -X;
        T2 = -Y;
        *tB = Q16_mpy(ONE_HALF_Q16, (1 - T1 - T2));
        *tC = *tB + T1;
        *tA = *tC + T2;
        break;
    default:
        *tA = ONE_HALF_Q16;
        *tB = ONE_HALF_Q16;
        *tC = ONE_HALF_Q16;
        break;
    }

    /* Old version
     *


     // Little rearrangement for the next section
     temp = -Y;
     Y = -Z;
     Z = temp;
     // X = beta
     // Y = 0.5*beta+alpha*sqrt(3)/2
     // Z = 0.5*beta-alpha*sqrt(3)/2
     //         X
     //     Z   ^   Y
     //      \  |  /
     //       \ | /
     //  ______\|/______
     //        / \
    //       /   \
    //      /     \


     // On-time determination for the three-phase bridge
     switch(sector)
     {
     case 3:
     // Sector 3: 0° to 60°
     // T1 (time in vector-0) = -Z
     // T2 (time in vector-60) = X
     // tA = 1/2 (1-T1-T2)
     // tB = tA+T1
     // tC = tB+T2
     *tA = Q16_mpy(ONE_HALF_Q16, (1+Z-X));
     *tB = (-Z+*tA);
     *tC = (X+*tB);
     break;
     case 1:
     // Sector 1: 60° to 120°
     // T1 (time in vector-60) = Z
     // T2 (time in vector-120) = Y
     // tA = tB + T2
     // tB = 1/2 (1-T1-T2)
     // tC = tA + T1
     *tB = Q16_mpy(ONE_HALF_Q16, (1-Z-Y));
     *tA = (Z+*tB);
     *tC = (Y+*tA);
     break;
     case 5:
     // Sector 5: 120° to 180°
     // T1 (time in vector-120) = X
     // T2 (time in vector-180) = Y
     // tA = tC + T2
     // tB = 1/2 (1-T1-T2)
     // tC = tB + T1
     *tB = Q16_mpy(ONE_HALF_Q16, (1-X+Y));
     *tC = (X+*tB);
     *tA = (-Y+*tC);
     break;
     case 4:
     // Sector 4: 180° to 240°
     // T1 (time in vector-180) = -X
     // T2 (time in vector-240) = Z
     // tA = tB + T2
     // tB = tC + T1
     // tC = 1/2 (1-T1-T2)
     *tC = Q16_mpy(ONE_HALF_Q16, (1+X-Z));
     *tB = (-X+*tC);
     *tA = (Z+*tB);
     break;
     case 6:
     // Sector 6: 240° to 300°
     // T1 (time in vector-240) = -Y
     // T2 (time in vector-300) = -Z
     // tA = tC + T1
     // tB = tA + T2
     // tC = 1/2 (1-T1-T2)
     *tC = Q16_mpy(ONE_HALF_Q16, (1+Y+Z));
     *tA = (-Y+*tC);
     *tB = (-Z+*tA);
     break;
     case 2:
     // Sector 2: 300° to 360°
     // T1 (time in vector-300) = Y
     // T2 (time in vector-360) = -X
     // tA = 1/2 (1-T1-T2)
     // tB = tB + T2
     // tC = tA + T1
     *tA = Q16_mpy(ONE_HALF_Q16, (1-Y+X));
     *tC = (Y+*tA);
     *tB =(-X+*tC);
     break;
     default:
     *tA = ONE_HALF_Q16;
     *tB = ONE_HALF_Q16;
     *tC = ONE_HALF_Q16;
     }
     */

}
/** 
 * dfsl_svmf
 * Implmentation of Space Vector Modulation using floating-point math.
 */
void dfsl_svmf(float alpha, float beta, float* tA, float* tB, float* tC) {
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
    /*
     * Old version
     *
     // Little rearrangement for the next section
     temp = -Y;
     Y = -Z;
     Z = temp;
     // X = beta
     // Y = 0.5*beta+alpha*sqrt(3)/2
     // Z = 0.5*beta-alpha*sqrt(3)/2
     //         X
     //     Z   ^   Y
     //      \  |  /
     //       \ | /
     //  ______\|/______
     //        / \
    //       /   \
    //      /     \


     // On-time determination for the three-phase bridge
     switch(sector)
     {
     case 3:
     // Sector 3: 0° to 60°
     // T1 (time in vector-0) = -Z
     // T2 (time in vector-60) = X
     // tA = 1/2 (1-T1-T2)
     // tB = tA+T1
     // tC = tB+T2
     *tA = (ONE_HALF*(1-(-Z)-X));
     *tB = (-Z+*tA);
     *tC = (X+*tB);
     break;
     case 1:
     // Sector 1: 60° to 120°
     // T1 (time in vector-60) = Z
     // T2 (time in vector-120) = Y
     // tA = tB + T2
     // tB = 1/2 (1-T1-T2)
     // tC = tA + T1
     *tB = (ONE_HALF*(1-Z-Y));
     *tA = (Z+*tB);
     *tC = (Y+*tA);
     break;
     case 5:
     // Sector 5: 120° to 180°
     // T1 (time in vector-120) = X
     // T2 (time in vector-180) = Y
     // tA = tC + T2
     // tB = 1/2 (1-T1-T2)
     // tC = tB + T1
     *tB = (ONE_HALF*(1-X-(-Y)));
     *tC = (X+*tB);
     *tA = (-Y+*tC);
     break;
     case 4:
     // Sector 4: 180° to 240°
     // T1 (time in vector-180) = -X
     // T2 (time in vector-240) = Z
     // tA = tB + T2
     // tB = tC + T1
     // tC = 1/2 (1-T1-T2)
     *tC = (ONE_HALF*(1-(-X)-Z));
     *tB = (-X+*tC);
     *tA = (Z+*tB);
     break;
     case 6:
     // Sector 6: 240° to 300°
     // T1 (time in vector-240) = -Y
     // T2 (time in vector-300) = -Z
     // tA = tC + T1
     // tB = tA + T2
     // tC = 1/2 (1-T1-T2)
     *tC = (ONE_HALF*(1-(-Y)-(-Z)));
     *tA = (-Y+*tC);
     *tB = (-Z+*tA);
     break;
     case 2:
     // Sector 2: 300° to 360°
     // T1 (time in vector-300) = Y
     // T2 (time in vector-360) = -X
     // tA = 1/2 (1-T1-T2)
     // tB = tB + T2
     // tC = tA + T1
     *tA = (ONE_HALF*(1-Y-(-X)));
     *tC = (Y+*tA);
     *tB =(-X+*tC);
     break;
     default:
     *tA = ONE_HALF;
     *tB = ONE_HALF;
     *tC = ONE_HALF;
     }
     */
}

void dfsl_ipark(int16_t D, int16_t Q, int16_t angle, int16_t* alpha,
        int16_t* beta) {
    int32_t Cos, Sin, angle_32;
    angle_32 = angle << 16;
    //Cos =
    arm_sin_cos_q31(angle_32, &Sin, &Cos);
    // Back to Q16 for me
    Sin = Sin >> 16;
    Cos = Cos >> 16;
    *alpha = Q16_mpy(D,Cos) - Q16_mpy(Q, Sin);
    *beta = Q16_mpy(Q,Cos) + Q16_mpy(D, Sin);
}

void dfsl_iparkf(float D, float Q, float angle, float* alpha, float* beta) {
    float Cos, Sin;
    arm_sin_cos_f32(angle * 360.0f, &Sin, &Cos);
    *alpha = D * Cos - Q * Sin;
    *beta = Q * Cos + D * Sin;
}

void dfsl_clarke(int32_t A, int32_t B, int32_t* Alpha, int32_t* Beta) {
    *Alpha = A;
    *Beta = Q16_mpy(A + 2 * B, INV_SQRT3_Q16);
}

void dfsl_clarkef(float A, float B, float* Alpha, float* Beta) {
    *Alpha = A;
    *Beta = (2.0f * B + A) * INV_SQRT3;
}

void dfsl_park(int32_t Alpha, int32_t Beta, int16_t Angle, int16_t* D,
        int16_t* Q) {
    int32_t Cos, Sin, angle_32;
    angle_32 = Angle << 16;
    //Cos =
    arm_sin_cos_q31(angle_32, &Sin, &Cos);
    // Back to Q16 for me
    Sin = Sin >> 16;
    Cos = Cos >> 16;
    *D = Q16_mpy(Alpha,Cos) + Q16_mpy(Beta, Sin);
    *Q = Q16_mpy(Beta,Cos) - Q16_mpy(Alpha, Sin);
}

void dfsl_parkf(float Alpha, float Beta, float Angle, float* D, float* Q) {
    float Cos, Sin;
    arm_sin_cos_f32(Angle * 360.0f, &Sin, &Cos);
    *D = Alpha * Cos + Beta * Sin;
    *Q = Beta * Cos - Alpha * Sin;
}
