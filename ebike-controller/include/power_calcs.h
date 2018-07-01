#ifndef _POWER_CALCS_H_
#define _POWER_CALCS_H_



#define ONE_OVER_SQRT3_F  (0.57735026918962576451f)

typedef struct _PowerCalcs
{
  // Inputs
  float Ta;
  float Tb;
  float Tc;
  float Vbus;
  float Ialpha;
  float Ibeta;
  // Outputs
  float TotalPower;
  float PhaseCurrent;
  float BatteryCurrent;
} PowerCalcs;

void power_calc(PowerCalcs* pc);


#endif
