/******************************************************************************
 * Filename: foc_lib.h
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

#ifndef _FOC_LIB_H
#define _FOC_LIB_H

#ifndef PI
#define PI                  3.14159265358979f
#endif

#ifndef SQRT3_OVER_2
#define SQRT3_OVER_2        0.86602650378444f

#endif

#ifndef ONE_HALF
#define ONE_HALF            0.5f
#endif

#ifndef INV_SQRT3
#define INV_SQRT3           0.57735026918963f
#endif

typedef struct _PID_Type {
    float Err; // Input: Error term (Reference - feedback)
    float Ui;  // Output: Integral output
    float Kp;  // Param: Proportional gain
    float Ki;  // Param: Integral gain
    float Kd;  // Param: Derivative gain
    float Kc;  // Param: Saturation gain
    float OutMin; // Param: Minimum output value
    float OutMax; // Param: Maximum output value
    float SatErr; // Output: Saturation error
    float Out;  // Output: PID output term
    float Up1;  // Previous proportional output
} PID_Type;

typedef struct _Biquad_Type {
    float A1; // Param: A1 gain (output at one delay)
    float A2; // Param: A2 gain (output at two delays)
    float B0; // Param: B0 gain (input, no delay)
    float B1; // Param: B1 gain (input, one delay)
    float B2; // Param: B2 gain (input, two delays)
    float U1; // State: First delay register
    float U2; // State: Second delay register
    float X;  // Input: variable to be filtered
    float Y;  // Output: filtered result
} Biquad_Type;

void FOC_SVM(float alpha, float beta, float* tA, float* tB, float* tC);

void FOC_Ipark(float D, float Q, float sin, float cos, float* alpha, float* beta);
void FOC_Clarke(float A, float B, float* Alpha, float* Beta);
void FOC_Park(float alpha, float beta, float sin, float cos, float* D, float* Q);

void FOC_BiquadCalc(Biquad_Type* biq);
void FOC_BiquadLPF(Biquad_Type* biq, float Fs, float f0, float Q);

void FOC_PIDdefaults(PID_Type* pid);
void FOC_PIDreset(PID_Type* pid);
void FOC_PIDcalc(PID_Type* pid);
void FOC_PIcalc(PID_Type* pid);

void FOC_RampGen(float* rampAngle, float rampInc);
float FOC_RampCtrl(float callingFreq, float rampFreq);

#endif
