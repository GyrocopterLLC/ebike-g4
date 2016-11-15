/*

David's Field Oriented Control Software Library

Contents:
1. Space Vector Modulate
	Calculates three-phase duty cycles (A, B, and C) for a two-phase stationary input (alpha and beta)

Version 1
May 20, 2014

*/

#ifndef _DAVIDS_FOC_LIB_H_
#define _DAVIDS_FOC_LIB_H_

#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define ONE_HALF_Q16	(32768)
#define SQRT3_OVER_2_Q16 (56756) // rounding error of .0000128 (0.00148%)
#define INV_SQRT3_Q16	(37837) // rounding error of .00000347 (0.000601%)
#define ONE_HALF		(0.5f)
#define SQRT3_OVER_2	(0.8660254f)
#define INV_SQRT3		(0.5773503f)

#define Q_FACTOR		(22)	// Max value: (2^31-1)/2^22 = 511.9999997615814208984375
								// Smallest positive: 1/2^22 = 0.0000002384185791015625

// **** Biquad filter design ****
// For a 1kHz sampled system
// Low pass filter, Fc = 20Hz, Q = 0.9
#define BIQ_LPF_DEFAULTS	{-1.855062f, \
							  0.8698062f, \
							  0.003685995f, \
							  0.007371990f, \
							  0.003685995f, \
							  0.0f, \
							  0.0f, \
							  0.0f, \
							  0.0f }
typedef struct
{
	int32_t Err; // Input: Error term (Reference - feedback)
	int32_t Ui;  // Output: Integral output
	int32_t Kp;  // Param: Proportional gain
	int32_t Ki;  // Param: Integral gain
	int32_t Kd;  // Param: Derivative gain
	int32_t Kc;  // Param: Saturation gain
	int32_t OutMin; // Param: Minimum output value
	int32_t OutMax; // Param: Maximum output value
	int32_t SatErr; // Output: Saturation error
	int32_t Out;  // Output: PID output term
	int32_t Up1;  // Previous proportional output
}PID_Type;

typedef struct
{
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

}PID_Float_Type;

typedef struct
{
	float A1; // Param: A1 gain (output at one delay)
	float A2; // Param: A2 gain (output at two delays)
	float B0; // Param: B0 gain (input, no delay)
	float B1; // Param: B1 gain (input, one delay)
	float B2; // Param: B2 gain (input, two delays)
	float U1; // State: First delay register
	float U2; // State: Second delay register
	float X;  // Input: variable to be filtered
	float Y;  // Output: filtered result
}Biquad_Float_Type;

void dfsl_rampgen(uint16_t* rampAngle, uint16_t rampInc);
uint16_t dfsl_rampctrl(uint32_t callingFreq, uint32_t rampFreq);
void dfsl_pid_defaults(PID_Type* pid);
void dfsl_pid_defaultsf(PID_Float_Type* pid);
void dfsl_pid(PID_Type* pid);
void dfsl_pidf(PID_Float_Type* pid);
void dfsl_biquadf(Biquad_Float_Type* biq);
void dfsl_svm(int16_t alpha, int16_t beta, int32_t* tA, int32_t* tB, int32_t* tC);
void dfsl_svmf(float alpha, float beta, float* tA, float* tB, float* tC);
void dfsl_ipark(int16_t D, int16_t Q, int16_t angle, int16_t* alpha, int16_t* beta);
void dfsl_iparkf(float D, float Q, float angle, float* alpha, float* beta);
void dfsl_clarke(int32_t A, int32_t B, int32_t* Alpha, int32_t* Beta);
void dfsl_clarkef(float A, float B, float* Alpha, float* Beta);
void dfsl_park(int32_t Alpha, int32_t Beta, int16_t Angle, int16_t* D, int16_t* Q);
void dfsl_parkf(float Alpha, float Beta, float Angle, float* D, float* Q);

#endif //_DAVIDS_FOC_LIB_H_
