/*
 * throttle.c
 *
 *  Created on: Sep 17, 2016
 *      Author: David
 */

#include "throttle.h"
#include "gpio.h"
#include "adc.h"

Biquad_Float_Type Throttle_filt1 = THROTTLE_LPF_DEFAULTS;
Biquad_Float_Type Throttle_filt2 = THROTTLE_LPF_DEFAULTS;
Biquad_Float_Type *pThrottle_filts[2] = {&Throttle_filt1, &Throttle_filt2};
Throttle_Type sThrottle1  = THROTTLE_DEFAULTS;
Throttle_Type sThrottle2 = THROTTLE_DEFAULTS;
Throttle_Type *psThrottles[2] = {&sThrottle1, &sThrottle2};
Throttle_Analog_Type sAnalogThrottle1 = THROTTLE_ANALOG_DEFAULTS;
Throttle_Analog_Type sAnalogThrottle2 = THROTTLE_ANALOG_DEFAULTS;
Throttle_Analog_Type *psAnalogThrottles[2] = {&sAnalogThrottle1, &sAnalogThrottle2};
Throttle_PAS_Type sPasThrottle1 = THROTTLE_PAS_DEFAULTS;
Throttle_PAS_Type sPasThrottle2 = THROTTLE_PAS_DEFAULTS;
Throttle_PAS_Type *psPasThrottles[2] = {&sPasThrottle1, &sPasThrottle2};

static void throttle_hyst_and_rate_limiting(Throttle_Type *thr);

void throttle_switch_type(uint8_t thrnum, uint8_t thrtype) {
  if (thrnum == 1) {
    if (thrtype == THROTTLE_TYPE_ANALOG) {
      // Set throttle pin to analog input
      GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Disable pin change interrupt
      EXTI->IMR &= ~(EXTI_IMR_MR5);
      // Disable timer overflow interrupt
      PAS1_TIM->DIER &= ~(TIM_DIER_UIE);
    } else if (thrtype == THROTTLE_TYPE_PAS) {
      // Set throttle pin to digital input
      GPIO_Input(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Enable pin change interrupt
      EXTI->IMR |= EXTI_IMR_MR5; // Enabled for line 5 (Px5 pins)
//      EXTI->FTSR |= EXTI_FTSR_TR5;
      EXTI->RTSR |= EXTI_RTSR_TR5; // Rising edge enabled
      SYSCFG->EXTICR[1] |= (0x03 << 4); // Set EXTI5 for Port C

      // Enable this interrupt in the NVIC
      NVIC_SetPriority(PAS1_PINCHANGE_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(PAS1_PINCHANGE_IRQn);

      // Startup the PAS timer, 0.1ms precision
      // Overflow at 1 second so the motor doesn't keep on spinning after
      // you stop pedaling.
      PAS1_TIMER_CLK_ENABLE();
      PAS1_TIM->PSC = PAS_TIM_PSC;
      PAS1_TIM->ARR = PAS_TIM_ARR;
      // Enable overflow interrupt. This occurs when no pedaling has been
      // detected for one full timer cycle.
      PAS1_TIM->DIER |= TIM_DIER_UIE;
      NVIC_SetPriority(PAS1_TIMER_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(PAS1_TIMER_IRQn);

      PAS1_TIM->CR1 |= TIM_CR1_CEN;

    }
  } else if (thrnum == 2) {
    if (thrtype == THROTTLE_TYPE_ANALOG) {
      // Set throttle pin to analog input
      GPIO_Analog(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN);
      // Disable pin change interrupt
      EXTI->IMR &= ~(EXTI_IMR_MR0);
    } else if (thrtype == THROTTLE_TYPE_PAS) {
      // Set throttle pin to digital input
      GPIO_Input(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN);
      // Enable pin change interrupt
      EXTI->IMR |= EXTI_IMR_MR0; // Enabled for line 0 (Px0 pins)
//      EXTI->FTSR |= EXTI_FTSR_TR0;
      EXTI->RTSR |= EXTI_RTSR_TR0; // Rising edge enabled
      SYSCFG->EXTICR[0] |= 0x02; // Set EXTI0 for Port B

      // Enable this interrupt in the NVIC
      NVIC_SetPriority(PAS2_PINCHANGE_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(PAS2_PINCHANGE_IRQn);

      // Startup the PAS timer, 0.1ms precision
      // Overflow at 1 second so the motor doesn't keep on spinning after
      // you stop pedaling.
      PAS2_TIMER_CLK_ENABLE();
      PAS2_TIM->PSC = PAS_TIM_PSC;
      PAS2_TIM->ARR = PAS_TIM_ARR;
      // Enable overflow interrupt. This occurs when no pedaling has been
      // detected for one full timer cycle.
      PAS2_TIM->DIER |= TIM_DIER_UIE;
      NVIC_SetPriority(PAS2_TIMER_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(PAS2_TIMER_IRQn);

      PAS2_TIM->CR1 |= TIM_CR1_CEN;
    }
  }
}

void throttle_process(uint8_t thrnum) {
  if ((thrnum != 1) && (thrnum != 2)) {
    // Invalid throttle number
    return;
  }
  if (psThrottles[thrnum - 1]->throttle_type == THROTTLE_TYPE_ANALOG) {
    // Only need to process if analog throttle type
    // The PAS throttle type is taken care of with interrupt / timer callbacks

    // Filter the raw throttle voltage
    pThrottle_filts[thrnum - 1]->X = adcGetThrottle(thrnum);
    dfsl_biquadf(pThrottle_filts[thrnum - 1]);

    // *****STARTUP SEQUENCE*****
    // For the first Throttle Startup period, input is ignored
    // This allows the Biquad filter to stabilize
    // After this period, the value of the biquad filter output is
    // selected as the minimum throttle position.

    float temp_cmd = 0.0f;
    if (psAnalogThrottles[thrnum - 1]->startup_count < THROTTLE_START_TIME) {
      // Throttle startup routine.
      // No effect for about a short duration ("Deadtime")
      // Then, average the throttle position for the remainder of the startup
      // This averaged value becomes the throttle minimum position
      psAnalogThrottles[thrnum - 1]->startup_count++;
      if (psAnalogThrottles[thrnum - 1]->startup_count
          >= THROTTLE_START_DEADTIME) {
        psAnalogThrottles[thrnum - 1]->min = psAnalogThrottles[thrnum - 1]->min
            + pThrottle_filts[thrnum - 1]->Y;
      }
      psThrottles[thrnum - 1]->throttle_command = 0.0f;
    } else {
      // If this is the first time, compute the average for the minimum position
      // End of startup routine
      if (psAnalogThrottles[thrnum - 1]->startup_count == THROTTLE_START_TIME) {
        psAnalogThrottles[thrnum - 1]->startup_count++;
        // Calculate the minimum throttle position
        psAnalogThrottles[thrnum - 1]->min = psAnalogThrottles[thrnum - 1]->min
            / (THROTTLE_START_TIME - THROTTLE_START_DEADTIME);
        if ((psAnalogThrottles[thrnum - 1]->min > 1.0f)
            || (psAnalogThrottles[thrnum - 1]->min < 0.3f)) {
          psAnalogThrottles[thrnum - 1]->min = THROTTLE_MIN_DEFAULT;
        }
        // Estimate the throttle maximum position
        psAnalogThrottles[thrnum - 1]->max = adcGetVref() - THROTTLE_DROPOUT;
        if ((psAnalogThrottles[thrnum - 1]->max > 3.0f)
            || (psAnalogThrottles[thrnum - 1]->max < 1.5f)) {
          psAnalogThrottles[thrnum - 1]->max = THROTTLE_MAX_DEFAULT;
        }
        // Calculate a scale factor to apply to the raw voltage
        psAnalogThrottles[thrnum - 1]->scale_factor = (1.0f)
            / (psAnalogThrottles[thrnum - 1]->max
                - psAnalogThrottles[thrnum - 1]->min);

      }
      // If incoming throttle position is less than the recorded minimum,
      // Redo the startup routine
      if (pThrottle_filts[thrnum - 1]->Y
          < (psAnalogThrottles[thrnum - 1]->min - THROTTLE_RANGE_LIMIT)) {
        psAnalogThrottles[thrnum - 1]->startup_count = 0;
        psThrottles[thrnum - 1]->throttle_command = 0.0f;
      }
      // If incoming throttle position is greater than recorded maximum,
      // take that value as maximum and recalculate scale factor
      if (pThrottle_filts[thrnum - 1]->Y
          > (psAnalogThrottles[thrnum - 1]->max + THROTTLE_RANGE_LIMIT)) {
        psAnalogThrottles[thrnum - 1]->max = pThrottle_filts[thrnum - 1]->Y;
        psAnalogThrottles[thrnum - 1]->scale_factor = (1.0f)
            / (psAnalogThrottles[thrnum - 1]->max
                - psAnalogThrottles[thrnum - 1]->min);
      }
      // Regular throttle processing
      temp_cmd = (pThrottle_filts[thrnum - 1]->Y
          - psAnalogThrottles[thrnum - 1]->min)
          * (psAnalogThrottles[thrnum - 1]->scale_factor);
      // Clip at 0% and 100%
      if (temp_cmd < THROTTLE_OUTPUT_MIN)
        temp_cmd = THROTTLE_OUTPUT_MIN;
      if (temp_cmd > THROTTLE_OUTPUT_MAX)
        temp_cmd = THROTTLE_OUTPUT_MAX;
    }
    psThrottles[thrnum - 1]->throttle_command = temp_cmd;
    throttle_hyst_and_rate_limiting(psThrottles[thrnum - 1]);
  } else if (psThrottles[thrnum - 1]->throttle_type == THROTTLE_TYPE_PAS) {
    // Just do the hysterisis and rate limiting at this known update rate
    // Pedaling speed calculation is done separately, triggered whenever the
    // PAS sensor switches
    throttle_hyst_and_rate_limiting(psThrottles[thrnum - 1]);
  }
}

static void throttle_hyst_and_rate_limiting(Throttle_Type *thr) {
  if (thr->state) {
    // Hysteresis. If throttle was on but passes below hysteresis level, turn it off
    if (thr->throttle_command <= THROTTLE_HYST_LOW) {
      thr->throttle_command = 0.0f;
      thr->state = 0;
    }
  } else {
    // If throttle was off but passes above hysteresis level, turn it on
    if (thr->throttle_command >= THROTTLE_HYST_HIGH) {
      thr->state = 1;
    } else {
      thr->throttle_command = 0.0f;
    }
  }
  // Rate limit (upward only! no limit on how fast the throttle can fall)
  float delta = thr->throttle_command - thr->prev_output;
  if (delta > THROTTLE_SLEW_RATE) {
    thr->throttle_command = thr->prev_output + THROTTLE_SLEW_RATE;
  }
  thr->prev_output = thr->throttle_command;
}

void throttle_pas_process(uint8_t thrnum) {
  uint16_t newcount;

  // This function is entered at any rising edge of the PAS signal
  // Get the timer reading now
  if (thrnum == 1) {
    newcount = PAS1_TIM->CNT;
  } else if (thrnum == 2) {
    newcount = PAS2_TIM->CNT;
  } else
    return;

  // Figure out the rate of rotation in Hz
  float howfastwego = ((float)newcount) / ((float)PAS_CLK) / ((float)PAS_PPR);

  // Filter the speed with a rolling low-pass filter
  psPasThrottles[thrnum-1]->filtered_speed = howfastwego * PAS_FILTER +
      (psPasThrottles[thrnum-1]->filtered_speed * (1 - PAS_FILTER));

  float temp_cmd;
  // Scale and limit the output
  temp_cmd = psPasThrottles[thrnum-1]->filtered_speed * psPasThrottles[thrnum-1]->scale_factor;
  if (temp_cmd < THROTTLE_OUTPUT_MIN)
    temp_cmd = THROTTLE_OUTPUT_MIN;
  if (temp_cmd > THROTTLE_OUTPUT_MAX)
    temp_cmd = THROTTLE_OUTPUT_MAX;

  psThrottles[thrnum-1]->throttle_command = temp_cmd;
  throttle_hyst_and_rate_limiting(psThrottles[thrnum-1]);
}

void throttle_pas_timer_overflow(uint8_t thrnum)
{
  // This function is entered when the PAS timer wraps around
  // Should be occurring at a relativly short (1-2 sec) interval
  // so the motor cuts out very soon after pedaling stops

  psPasThrottles[thrnum-1]->filtered_speed = 0; // Reset the filtered speed
  psThrottles[thrnum-1]->throttle_command = 0.0f; // And reset the scaled throttle command
}

float throttle_get_command(uint8_t thrnum)
{
  return psThrottles[thrnum-1]->throttle_command;
}
