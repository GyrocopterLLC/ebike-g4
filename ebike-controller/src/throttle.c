/*
 * throttle.c
 *
 *  Created on: Sep 17, 2016
 *      Author: David
 */

#include "throttle.h"
#include "gpio.h"

Biquad_Float_Type Throttle_filt = THROTTLE_LPF_DEFAULTS;
Throttle_Type sThrottle  = THROTTLE_DEFAULTS;
Throttle_Analog_Type sAnalogThrottle = THROTTLE_ANALOG_DEFAULTS;
Throttle_PAS_Type sPasThrottle = THROTTLE_PAS_DEFAULTS;
Biquad_Float_Type sPasBiqLow = BIQ_LPF_DEFAULTS;
Biquad_Float_Type sPasBiqHigh = BIQ_LPF_DEFAULTS;


void throttle_switch_type(uint8_t thrnum, uint8_t thrtype) {
  if (thrnum == 1) {
    if (thrtype == THROTTLE_TYPE_ANALOG) {
      // Set throttle pin to analog input
      GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Disable pin change interrupt
      EXTI->IMR &= ~(EXTI_IMR_MR5);
    } else if (thrtype == THROTTLE_TYPE_PAS) {
      // Set throttle pin to digital input
      GPIO_Input(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Enable pin change interrupt
      EXTI->IMR |= EXTI_IMR_MR5; // Enabled for line 5 (Px5 pins)
//      EXTI->FTSR |= EXTI_FTSR_TR5;
      EXTI->RTSR |= EXTI_RTSR_TR5; // Rising edge enabled
      SYSCFG->EXTICR[1] |= (0x03 << 4); // Set EXTI5 for Port C

      // Enable this interrupt in the NVIC
      NVIC_SetPriority(EXTI9_5_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(EXTI9_5_IRQn);

      // Startup the PAS timer, 0.1ms precision
      // Overflow at 1 second so the motor doesn't keep on spinning after
      // you stop pedaling.
      PAS1_TIMER_CLK_ENABLE();
      PAS1_TIM->PSC = PAS_TIM_PSC;
      PAS1_TIM->ARR = PAS_TIM_ARR;
      PAS1_TIM->CR1 = TIM_CR1_CEN;
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
      NVIC_SetPriority(EXTI0_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(EXTI0_IRQn);

      // Startup the PAS timer, 0.1ms precision
      PAS2_TIMER_CLK_ENABLE();
      PAS2_TIM->PSC = PAS_TIM_PSC;
      PAS2_TIM->ARR = 0xFFFF;
      PAS2_TIM->CR1 = TIM_CR1_CEN;
    }
  }
}

float throttle_process(float raw_voltage) {
  // Filter the raw throttle voltage
  Throttle_filt.X = raw_voltage;
  dfsl_biquadf(&Throttle_filt);

  // *****STARTUP SEQUENCE*****
  // For the first Throttle Startup period, input is ignored
  // This allows the Biquad filter to stabilize
  // After this period, the value of the biquad filter output is
  // selected as the minimum throttle position.

  float temp_cmd;
  if (sAnalogThrottle.startup_count < THROTTLE_START_TIME) {
    // Throttle startup routine.
    // No effect for about a short duration ("Deadtime")
    // Then, average the throttle position for the remainder of the startup
    // This averaged value becomes the throttle minimum position
    sAnalogThrottle.startup_count++;
    if (sAnalogThrottle.startup_count >= THROTTLE_START_DEADTIME) {
      sAnalogThrottle.min = sAnalogThrottle.min + Throttle_filt.Y;
    }
    return 0.0f;
  } else {
    // If this is the first time, compute the average for the minimum position
    // End of startup routine
    if (sAnalogThrottle.startup_count == THROTTLE_START_TIME) {
      sAnalogThrottle.startup_count++;
      // Calculate the minimum throttle position
      sAnalogThrottle.min = sAnalogThrottle.min
          / (THROTTLE_START_TIME - THROTTLE_START_DEADTIME);
      if ((sAnalogThrottle.min > 1.0f) || (sAnalogThrottle.min < 0.3f)) {
        sAnalogThrottle.min = THROTTLE_MIN_DEFAULT;
      }
      // Estimate the throttle maximum position
      sAnalogThrottle.max = adcGetVref() - THROTTLE_DROPOUT;
      if ((sAnalogThrottle.max > 3.0f) || (sAnalogThrottle.max < 1.5f)) {
        sAnalogThrottle.max = THROTTLE_MAX_DEFAULT;
      }
      // Calculate a scale factor to apply to the raw voltage
      sAnalogThrottle.scale_factor = (1.0f) / (sAnalogThrottle.max - sAnalogThrottle.min);

    }
    // If incoming throttle position is less than the recorded minimum,
    // Redo the startup routine
    if (Throttle_filt.Y < (sAnalogThrottle.min - THROTTLE_RANGE_LIMIT)) {
      sAnalogThrottle.startup_count = 0;
      sThrottle.throttle_command = 0.0f;
      return 0.0f;
    }
    // If incoming throttle position is greater than recorded maximum,
    // take that value as maximum and recalculate scale factor
    if (Throttle_filt.Y > (sAnalogThrottle.max + THROTTLE_RANGE_LIMIT)) {
      sAnalogThrottle.max = Throttle_filt.Y;
      sAnalogThrottle.scale_factor = (1.0f) / (sAnalogThrottle.max - sAnalogThrottle.min);
    }
    // Regular throttle processing
    temp_cmd = (Throttle_filt.Y - sAnalogThrottle.min) * (sAnalogThrottle.scale_factor);
    // Clip at 0% and 100%
    if (temp_cmd < THROTTLE_OUTPUT_MIN)
      temp_cmd = THROTTLE_OUTPUT_MIN;
    if (temp_cmd > THROTTLE_OUTPUT_MAX)
      temp_cmd = THROTTLE_OUTPUT_MAX;
  }
  sThrottle.throttle_command = temp_cmd;
  throttle_hyst_and_rate_limiting(&sThrottle);
  return sThrottle.throttle_command;
}

void throttle_hyst_and_rate_limiting(Throttle_Type *thr)
{
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
  sPasThrottle.filtered_speed = howfastwego * PAS_FILTER +
      (sPasThrottle.filtered_speed * (1 - PAS_FILTER));

  // Scale and limit the output
  sPasThrottle.throttle_command = sPasThrottle.filtered_speed * sPasThrottle.scale_factor;
  if (sPasThrottle.throttle_command < THROTTLE_OUTPUT_MIN)
    sPasThrottle.throttle_command = THROTTLE_OUTPUT_MIN;
  if (sPasThrottle.throttle_command > THROTTLE_OUTPUT_MAX)
    sPasThrottle.throttle_command = THROTTLE_OUTPUT_MAX;
}
