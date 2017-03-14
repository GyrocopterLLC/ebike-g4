#include "motor_loop.h"

void Motor_Loop(Motor_Controls* cntl, Motor_Observations* obv,
		FOC_StateVariables* foc, Motor_PWMDuties* duty)
{
	float ipark_a, ipark_b;

	switch(cntl->state)
	{
	case Motor_Off:
		// There's no command to give, so we can skip all that fancy processing.
		duty->tA = 0.0f;
		duty->tB = 0.0f;
		duty->tC = 0.0f;
		break;
	case Motor_Startup:
		// Running the motor in six-step mode.
		// Current monitoring is not used in determining duty cycle, unless
		// a fault condition is met (over-current)
		// State settings:
		//		6 -> +B, -C
		//		2 -> +B, -A
		//		3 -> +C, -A
		//		1 -> +C, -B
		// 		5 -> +A, -B
		//		4 -> +A, -C
		switch(obv->HallState)
		{
			// Set duty cycles - only one phase is PWM'd. Throttle command is directly
			// sent as power demand (in form of PWM duty cycle)
			// Enable the "participating" phases. One phase pwm, one phase low-side on,
			// and the third phase completely turned off
			// TODO: These should only change when Hall change happens
		case 6:
			duty->tB = cntl->ThrottleCommand;
			PHASE_A_OFF();
			PHASE_C_LOW();
			PHASE_B_PWM();
			break;
		case 2:
			duty->tB = cntl->ThrottleCommand;
			PHASE_C_OFF();
			PHASE_A_LOW();
			PHASE_B_PWM();
			break;
		case 3:
			duty->tC = cntl->ThrottleCommand;
			PHASE_B_OFF();
			PHASE_A_LOW();
			PHASE_C_PWM();
			break;
		case 1:
			duty->tC = cntl->ThrottleCommand;
			PHASE_A_OFF();
			PHASE_B_LOW();
			PHASE_C_PWM();
			break;
		case 5:
			duty->tA = cntl->ThrottleCommand;
			PHASE_C_OFF();
			PHASE_B_LOW();
			PHASE_A_PWM();
			break;
		case 4:
			duty->tA = cntl->ThrottleCommand;
			PHASE_B_OFF();
			PHASE_C_LOW();
			PHASE_A_PWM();
			break;
		default:
			// Oh shit damage control
			cntl->state = Motor_Fault;
			duty->tA = 0.0f;
			duty->tB = 0.0f;
			duty->tC = 0.0f;
			PWM_MotorOFF();
			break;
		}
		break;
	case Motor_AtSpeed:
		// Running full-fledged FOC algorithm now!

		// **************** FEEDBACK PATH *****************
		// Read angle from Hall sensors
		obv->RotorAngle = ((float)HallSensor_Get_Angle())/65536.0f;
		// Transform sensor readings
		dfsl_clarkef(obv->iA, obv->iB, &(foc->Clarke_Alpha), &(foc->Clarke_Beta));
		dfsl_parkf(foc->Clarke_Alpha,foc->Clarke_Beta,obv->RotorAngle, &(foc->Park_D), &(foc->Park_Q));
		// Input feedbacks to the Id and Iq controllers
		// Filter the currents
		/*
		Id_Filt.X = foc->Park_D;
		Iq_Filt.X = foc->Park_Q;
		dfsl_biquadf(&Id_Filt);
		dfsl_biquadf(&Iq_Filt);
		*/
		// Pass filtered current to the PI(D)s
		foc->Id_PID->Err = 0.0f - foc->Park_D;
		foc->Iq_PID->Err = (3.0f)*(cntl->ThrottleCommand) - foc->Park_Q;
		//Id_control.Err = 0.0f - Id_Filt.Y;
		//Iq_control.Err = (3.0f)*Throttle_cmd - Iq_Filt.Y;
		// Don't integrate unless the throttle is active
		if(cntl->ThrottleCommand > 0.0f)
		{
			dfsl_pidf(foc->Id_PID);
			dfsl_pidf(foc->Iq_PID);
		}

		// **************** FORWARD PATH *****************
		// Feed to inverse Park
		dfsl_iparkf(foc->Id_PID->Out,foc->Iq_PID->Out,obv->RotorAngle,&ipark_a, &ipark_b);
		//dfsl_iparkf(0, Throttle_cmd, fangle, &ipark_a, &ipark_b);
		// Inverse Park outputs to space vector modulation, output three-phase waveforms
		dfsl_svmf(ipark_a, ipark_b, &(duty->tA), &(duty->tB), &(duty->tC));
		// Convert from floats to 16-bit ints
		break;
	case Motor_Fault:
		PWM_MotorOFF();
		duty->tA = 0.0f;
		duty->tB = 0.0f;
		duty->tC = 0.0f;
		break;
	}
}
