/*

Power Calculations

Contents:
1. Calculate the total power based on Ialpha, Ibeta reference frame
        OR
1. Calculate the total power in trapezoidal mode

Version 1
July 1, 2018

*/

#include "stm32f4xx.h"
#include "arm_math.h"
#include "power_calcs.h"


void power_calc(PowerCalcs* pc)
{
  // Step 1 - calculate the voltages for each phase to neutral
  // Vx = Tx * Vbus, x = a,b,c
  // Vn = (Va + Vb + Vc)/3
  // Van = Va - Vn = (2*Va - Vb - Vc) / 3
  // ...similar for Vbn, Vcn
  float Van, Vbn, Vbeta, phaseVolts;
  Van = pc->Vbus * ((2.0f*pc->Ta) - (pc->Tb) - (pc->Tc))/3.0f;
  Vbn = pc->Vbus * ((2.0f*pc->Tb) - (pc->Ta) - (pc->Tc))/3.0f;

  // Step 2 - Clarke transform from 3-phase to 2-phase
  // Valpha = Van (keeping the same name)
  Vbeta = (2.0f*Vbn + Van)*(ONE_OVER_SQRT3_F);

  // Step 3 - Calculate magnitude of phase current
  // I_mag = (3/2) * sqrt(Ialpha^2 + Ibeta^2)
  pc->PhaseCurrent = 1.5f*sqrtf((pc->Ialpha)*(pc->Ialpha) + (pc->Ibeta)*(pc->Ibeta));

  // Step 4 - Calculate magnitude of phase voltage
  phaseVolts = sqrtf(Van*Van + Vbeta*Vbeta);

  // Step 5 - Calculate total power
  pc->TotalPower = phaseVolts * pc->PhaseCurrent;

  // Step 6 - Calculate battery current
  // Conservation of power:
  //      Total power = V_phase*I_phase = V_battery*I_battery
  pc->BatteryCurrent = pc->TotalPower / pc->Vbus;

}
