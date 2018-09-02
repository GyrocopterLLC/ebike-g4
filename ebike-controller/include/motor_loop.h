#ifndef MOTOR_LOOP_H_
#define MOTOR_LOOP_H_

#include "stm32f4xx.h"
#include "DavidsFOCLib.h"
#include "pwm.h"
#include "hallSensor.h"

typedef enum _Motor_RunState
{
	Motor_Off,		// All switching disabled.
	Motor_SixStep,  // Running in trapezoidal mode
	Motor_Startup,	// In FOC, but with untrustworthy angle
	Motor_AtSpeed,	// Switched to FOC full control
	Motor_Fault		// Something fd up
} Motor_RunState;

typedef struct _Motor_Controls
{
	Motor_RunState state;
	float ThrottleCommand;
	float BusVoltage;
} Motor_Controls;

typedef struct _Motor_Observations
{
	float iA;
	float iB;
	float iC;
	float RotorAngle;
	uint8_t HallState;
} Motor_Observations;

typedef struct _Motor_PWMDuties
{
	float tA;
	float tB;
	float tC;
} Motor_PWMDuties;

typedef struct _FOC_StateVariables
{
	float Clarke_Alpha;
	float Clarke_Beta;
	float Park_D;
	float Park_Q;
	PID_Float_Type* Id_PID;
	PID_Float_Type* Iq_PID;
} FOC_StateVariables;

void Motor_Loop(Motor_Controls* cntl, Motor_Observations* obv,
		FOC_StateVariables* foc, Motor_PWMDuties* duty);

#endif // MOTOR_LOOP_H_
