/*
 * throttle.c
 *
 *  Created on: Sep 17, 2016
 *      Author: David
 */

#include "throttle.h"
#include "gpio.h"
#include "adc.h"
#include "ui.h"
#include "eeprom_emulation.h"

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
static void throttle_set_scale(Throttle_Analog_Type* thr);
//static void throttle_detect_min_max(uint8_t thrnum); // Currently unused

void throttle_init(void) {
  // Reads from EEPROM and initializes throttle settings
  // This function can also be used to refresh the RAM settings from the EEPROM
  psThrottles[0]->throttle_type = EE_ReadInt16WithDefault(EE_ADR_TYPE1, THROTTLE_TYPE_ANALOG);
  throttle_switch_type(1, psThrottles[0]->throttle_type);
  psThrottles[0]->hyst = EE_ReadFloatWithDefault(EE_ADR_HYST1, THROTTLE_HYST_DEFAULT);
  psThrottles[0]->filt = EE_ReadFloatWithDefault(EE_ADR_FILT1, THROTTLE_FILT_DEFAULT);
  dfsl_biquadcalc_lpf(pThrottle_filts[0], THROTTLE_SAMPLING_RATE, psThrottles[0]->filt,
            THROTTLE_FILT_Q_DEFAULT);
  psThrottles[0]->rise = EE_ReadFloatWithDefault(EE_ADR_RISE1, THROTTLE_RISE_DEFAULT);
  psAnalogThrottles[0]->min = EE_ReadFloatWithDefault(EE_ADR_MIN1, THROTTLE_MIN_DEFAULT);
  psAnalogThrottles[0]->max = EE_ReadFloatWithDefault(EE_ADR_MAX1, THROTTLE_MAX_DEFAULT);
  throttle_set_scale(psAnalogThrottles[0]);

  psThrottles[1]->throttle_type = EE_ReadInt16WithDefault(EE_ADR_TYPE2, THROTTLE_TYPE_ANALOG);
  throttle_switch_type(2, psThrottles[1]->throttle_type);
  psThrottles[1]->hyst = EE_ReadFloatWithDefault(EE_ADR_HYST2, THROTTLE_HYST_DEFAULT);
  psThrottles[1]->filt = EE_ReadFloatWithDefault(EE_ADR_FILT2, THROTTLE_FILT_DEFAULT);
  dfsl_biquadcalc_lpf(pThrottle_filts[1], THROTTLE_SAMPLING_RATE, psThrottles[1]->filt,
              THROTTLE_FILT_Q_DEFAULT);
  psThrottles[1]->rise = EE_ReadFloatWithDefault(EE_ADR_RISE2, THROTTLE_RISE_DEFAULT);
  psAnalogThrottles[1]->min = EE_ReadFloatWithDefault(EE_ADR_MIN2, THROTTLE_MIN_DEFAULT);
  psAnalogThrottles[1]->max = EE_ReadFloatWithDefault(EE_ADR_MAX2, THROTTLE_MAX_DEFAULT);
  throttle_set_scale(psAnalogThrottles[1]);
}

void throttle_save_to_eeprom(void) {
  EE_SaveInt16(EE_ADR_TYPE1, psThrottles[0]->throttle_type);
  EE_SaveInt16(EE_ADR_TYPE2, psThrottles[1]->throttle_type);
  EE_SaveFloat(EE_ADR_MIN1, psAnalogThrottles[0]->min);
  EE_SaveFloat(EE_ADR_MIN2, psAnalogThrottles[1]->min);
  EE_SaveFloat(EE_ADR_MAX1, psAnalogThrottles[0]->max);
  EE_SaveFloat(EE_ADR_MAX2, psAnalogThrottles[1]->max);
  EE_SaveFloat(EE_ADR_HYST1, psThrottles[0]->hyst);
  EE_SaveFloat(EE_ADR_HYST2, psThrottles[1]->hyst);
  EE_SaveFloat(EE_ADR_RISE1, psThrottles[0]->rise);
  EE_SaveFloat(EE_ADR_RISE2, psThrottles[1]->rise);
  EE_SaveFloat(EE_ADR_FILT1, psThrottles[0]->filt);
  EE_SaveFloat(EE_ADR_FILT2, psThrottles[1]->filt);
}

void throttle_switch_type(uint8_t thrnum, uint8_t thrtype) {
  if (thrnum == 1) {
    if (thrtype == THROTTLE_TYPE_NONE) {
      // Disable pin by setting input with pull-down
      GPIO_InputPD(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Disable PAS pin change interrupt
      NVIC_DisableIRQ(PAS1_PINCHANGE_IRQn);
      GPIO_EXTI_Config(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN, EXTI_None);
      // Disable PAS timer overflow interrupt
      NVIC_DisableIRQ(PAS1_TIMER_IRQn);
      PAS1_TIM->DIER &= ~(TIM_DIER_UIE);
      // Disable PAS timer
      PAS1_TIM->CR1 &= ~TIM_CR1_CEN;

    } else if (thrtype == THROTTLE_TYPE_ANALOG) {
      // Set throttle pin to analog
      GPIO_Analog(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Disable PAS pin change interrupt
      NVIC_DisableIRQ(PAS1_PINCHANGE_IRQn);
      GPIO_EXTI_Config(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN, EXTI_None);
      // Disable PAS timer overflow interrupt
      NVIC_DisableIRQ(PAS1_TIMER_IRQn);
      PAS1_TIM->DIER &= ~(TIM_DIER_UIE);
      // Disable PAS timer
      PAS1_TIM->CR1 &= ~TIM_CR1_CEN;

    } else if (thrtype == THROTTLE_TYPE_PAS) {
      // Set throttle pin to digital input
      GPIO_Input(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN);
      // Enable pin change interrupt
      GPIO_EXTI_Config(ADC_I_VBUS_THR1_PORT, ADC_THR1_PIN, EXTI_Rising);
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

      /* Setting the URS bit allows only a timer overflow to trigger an
       * interrupt. Without it, whenever the timer is reset (including when
       * the UG bit is set, like after a rising edge on the PAS is captured)
       * would cause this interrupt to be triggered. This would be bad.
       * The PAS would then always think it was timing out and not actually
       * reading any switching signals. */
      PAS1_TIM->CR1 |= TIM_CR1_URS;

      NVIC_SetPriority(PAS1_TIMER_IRQn, PRIO_PAS);
      NVIC_EnableIRQ(PAS1_TIMER_IRQn);

      // Enable the timer counting
      PAS1_TIM->CR1 |= TIM_CR1_CEN;

    }
  } else if (thrnum == 2) {
    if (thrtype == THROTTLE_TYPE_NONE) {
      // Disable pin by setting input with pull-down
      GPIO_InputPD(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN);
      // Disable PAS pin change interrupt
      NVIC_DisableIRQ(PAS2_PINCHANGE_IRQn);
      GPIO_EXTI_Config(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN, EXTI_None);
      // Disable PAS timer overflow interrupt
      NVIC_DisableIRQ(PAS2_TIMER_IRQn);
      PAS2_TIM->DIER &= ~(TIM_DIER_UIE);
      // Disable PAS EXTI IRQ// Disable PAS timer
      PAS2_TIM->CR1 &= ~TIM_CR1_CEN;

    } else if (thrtype == THROTTLE_TYPE_ANALOG) {
      // Set throttle pin to analog input
      GPIO_Analog(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN);
      // Disable PAS pin change interrupt
      NVIC_DisableIRQ(PAS2_PINCHANGE_IRQn);
      GPIO_EXTI_Config(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN, EXTI_None);
      // Disable PAS timer overflow interrupt
      NVIC_DisableIRQ(PAS2_TIMER_IRQn);
      PAS2_TIM->DIER &= ~(TIM_DIER_UIE);
      // Disable PAS timer
      PAS2_TIM->CR1 &= ~TIM_CR1_CEN;

    } else if (thrtype == THROTTLE_TYPE_PAS) {
      // Set throttle pin to digital input
      GPIO_Input(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN);
      // Enable pin change interrupt
      GPIO_EXTI_Config(ADC_THR2_AND_TEMP_PORT, ADC_THR2_PIN, EXTI_Rising);

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

      // Only allow timer overflow to trigger the interrupt.
      PAS2_TIM->CR1 |= TIM_CR1_URS;

      // Start the timer counting
      PAS2_TIM->CR1 |= TIM_CR1_CEN;
    }
  }
}

void throttle_process(uint8_t thrnum) {
  if ((thrnum != 1) && (thrnum != 2)) {
    // Invalid throttle number
    return;
  }
  /** No Throttle type **/
  if (psThrottles[thrnum - 1]->throttle_type == THROTTLE_TYPE_NONE) {
    // Set throttle to zero.
    psThrottles[thrnum - 1]->throttle_command = 0.0f;
  }

  /** Analog Throttle type **/
  else if (psThrottles[thrnum - 1]->throttle_type == THROTTLE_TYPE_ANALOG) {
    // Only need to process if analog throttle type
    // The PAS throttle type is taken care of with interrupt / timer callbacks
    float temp_cmd = 0.0f;
    // Filter the raw throttle voltage
    pThrottle_filts[thrnum - 1]->X = adcGetThrottle(thrnum);
    dfsl_biquadf(pThrottle_filts[thrnum - 1]);

    // Regular throttle processing
    temp_cmd = (pThrottle_filts[thrnum - 1]->Y
        - psAnalogThrottles[thrnum - 1]->min)
        * (psAnalogThrottles[thrnum - 1]->scale_factor);
    // Clip at 0% and 100%
    if (temp_cmd < THROTTLE_OUTPUT_MIN)
      temp_cmd = THROTTLE_OUTPUT_MIN;
    if (temp_cmd > THROTTLE_OUTPUT_MAX)
      temp_cmd = THROTTLE_OUTPUT_MAX;
    psThrottles[thrnum - 1]->throttle_command = temp_cmd;
    throttle_hyst_and_rate_limiting(psThrottles[thrnum - 1]);
  }

  /** PAS Throttle type **/
  else if (psThrottles[thrnum - 1]->throttle_type == THROTTLE_TYPE_PAS) {
    // Just do the hysterisis and rate limiting at this known update rate
    // Pedaling speed calculation is done separately, triggered whenever the
    // PAS sensor switches
    throttle_hyst_and_rate_limiting(psThrottles[thrnum - 1]);
  }
}

static void throttle_hyst_and_rate_limiting(Throttle_Type *thr) {
  if (thr->state) {
    // Hysteresis. If throttle was on but passes below hysteresis level, turn it off
    if (thr->throttle_command <= (thr->hyst)) {
      thr->throttle_command = 0.0f;
      thr->state = 0;
    }
  } else {
    // If throttle was off but passes above hysteresis level, turn it on
    if (thr->throttle_command >= (2.0f*(thr->hyst))) {
      thr->state = 1;
    } else {
      thr->throttle_command = 0.0f;
    }
  }
  // Rate limit (upward only! no limit on how fast the throttle can fall)
  float delta = thr->throttle_command - thr->prev_output;
  if (delta > thr->rise) {
    thr->throttle_command = thr->prev_output + thr->rise;
  }
  thr->prev_output = thr->throttle_command;
}

static void throttle_set_scale(Throttle_Analog_Type* thr)
{
  thr->scale_factor = (1.0f)/((thr->max) - (thr->min));
}
/*
static void throttle_detect_min_max(uint8_t thrnum) {
// Old code. Figure out what to keep in here.

  // *****STARTUP SEQUENCE*****
  // For the first Throttle Startup period, input is ignored
  // This allows the Biquad filter to stabilize
  // After this period, the value of the biquad filter output is
  // selected as the minimum throttle position.

  if (psAnalogThrottles[thrnum - 1]->startup_count < THROTTLE_START_TIME) {
    // Throttle startup routine.
    // No effect for about a short duration ("Deadtime")
    // Then, average the throttle position for the remainder of the startup
    // This averaged value becomes the throttle minimum position
    psAnalogThrottles[thrnum - 1]->startup_count++;
    if (psAnalogThrottles[thrnum - 1]->startup_count >= THROTTLE_START_DEADTIME) {
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
  }
}*/

void throttle_pas_process(uint8_t thrnum) {
  uint16_t newcount;

  // This function is entered at any rising edge of the PAS signal
  // Get the timer reading now
  if (thrnum == 1) {
    // Capture timer reading
    newcount = PAS1_TIM->CNT;
    // Reset timer counter
    PAS1_TIM->EGR |= TIM_EGR_UG;
  } else if (thrnum == 2) {
    // Capture timer reading
    newcount = PAS2_TIM->CNT;
    // Reset timer counter
    PAS2_TIM->EGR |= TIM_EGR_UG;
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

/**** Interfacing with UI ****/
uint8_t throttle_set_type(uint8_t thrnum, uint8_t thrtype)
{
  if((thrnum == 1) || (thrnum == 2)) {
    if((thrtype == THROTTLE_TYPE_NONE) || (thrtype == THROTTLE_TYPE_PAS) ||
        (thrtype == THROTTLE_TYPE_PAS))
    {
      psThrottles[thrnum-1]->throttle_type = thrtype;
      return UI_OK;
    }
  }
  return UI_ERROR;
}
uint8_t throttle_get_type(uint8_t thrnum)
{
  if((thrnum == 1) || (thrnum == 2)) {
    return psThrottles[thrnum-1]->throttle_type;
  }
  else {
    return THROTTLE_TYPE_NONE;
  }
}

uint8_t throttle_get_type_eeprom(uint8_t thrnum)
{
  uint8_t temp_type;
  if(thrnum == 1){
    temp_type = EE_ReadInt16WithDefault(EE_ADR_TYPE1, THROTTLE_TYPE_NONE);
  } else if(thrnum == 2) {
    temp_type = EE_ReadInt16WithDefault(EE_ADR_TYPE2, THROTTLE_TYPE_NONE);
  } else {
    temp_type = THROTTLE_TYPE_NONE;
  }
  return temp_type;

}

uint8_t throttle_set_min(uint8_t thrnum, float thrmin)
{
  if((thrnum == 1) || (thrnum == 2)) {
    if(thrmin >= 0.0f) {
      psAnalogThrottles[thrnum-1]->min = thrmin;
      throttle_set_scale(psAnalogThrottles[thrnum-1]);
      return UI_OK;
    }
  }
  return UI_ERROR;
}

float throttle_get_min(uint8_t thrnum)
{
  if((thrnum == 1) || (thrnum == 2)) {
    return psAnalogThrottles[thrnum-1]->min;
  }
  return 0.0f;
}

float throttle_get_min_eeprom(uint8_t thrnum)
{
  float temp = 0.0f;
  if(thrnum == 1) {
    temp = EE_ReadFloatWithDefault(EE_ADR_MIN1, 0.0f);
  } else if(thrnum == 2) {
    temp = EE_ReadFloatWithDefault(EE_ADR_MIN2, 0.0f);
  }
  return temp;
}

uint8_t throttle_set_max(uint8_t thrnum, float thrmax) {
  if ((thrnum == 1) || (thrnum == 2)) {
    if(thrmax >= 0.0f) {
      psAnalogThrottles[thrnum - 1]->max = thrmax;
      throttle_set_scale(psAnalogThrottles[thrnum-1]);
      return UI_OK;
    }
  }
  return UI_ERROR;
}

float throttle_get_max(uint8_t thrnum)
{
  if((thrnum == 1) || (thrnum == 2)) {
    return psAnalogThrottles[thrnum-1]->max;
  }
  return 0.0f;
}

float throttle_get_max_eeprom(uint8_t thrnum)
{
  float temp = 0.0f;
  if(thrnum == 1) {
    temp = EE_ReadFloatWithDefault(EE_ADR_MAX1, 0.0f);
  } else if(thrnum == 2) {
    temp = EE_ReadFloatWithDefault(EE_ADR_MAX2, 0.0f);
  }
  return temp;
}

uint8_t throttle_set_hyst(uint8_t thrnum, float thrhyst)
{
  if((thrnum == 1) || (thrnum == 2)) {
    if((thrhyst >= THROTTLE_HYST_MIN) && (thrhyst < THROTTLE_HYST_MAX)) {
      psThrottles[thrnum-1]->hyst = thrhyst;
      return UI_OK;
    }
  }
  return UI_ERROR;
}

float throttle_get_hyst(uint8_t thrnum)
{
  if((thrnum == 1) || (thrnum == 2)) {
    return psThrottles[thrnum-1]->hyst;
  }
  return 0.0f;
}

float throttle_get_hyst_eeprom(uint8_t thrnum)
{
  float temp = 0.0f;
  if(thrnum == 1) {
    temp = EE_ReadFloatWithDefault(EE_ADR_HYST1, 0.0f);
  } else if(thrnum == 2) {
    temp = EE_ReadFloatWithDefault(EE_ADR_HYST2, 0.0f);
  }
  return temp;
}

uint8_t throttle_set_filt(uint8_t thrnum, float thrfilt)
{
  if((thrnum == 1) || (thrnum == 2)) {
    if((thrfilt >= THROTTLE_FILT_MIN) && (thrfilt < THROTTLE_FILT_MAX)) {
      psThrottles[thrnum-1]->filt = thrfilt;
      // And do the filter calculation
      dfsl_biquadcalc_lpf(pThrottle_filts[thrnum-1], THROTTLE_SAMPLING_RATE, thrfilt,
          THROTTLE_FILT_Q_DEFAULT);
      return UI_OK;
    }
  }
  return UI_ERROR;
}

float throttle_get_filt(uint8_t thrnum)
{
  if((thrnum == 1) || (thrnum == 2)) {
    return psThrottles[thrnum-1]->filt;
  }
  return 0.0f;
}

float throttle_get_filt_eeprom(uint8_t thrnum)
{
  float temp = 0.0f;
  if(thrnum == 1) {
    temp = EE_ReadFloatWithDefault(EE_ADR_FILT1, 0.0f);
  } else if(thrnum == 2) {
    temp = EE_ReadFloatWithDefault(EE_ADR_FILT2, 0.0f);
  }
  return temp;
}

uint8_t throttle_set_rise(uint8_t thrnum, float thrrise)
{
  if((thrnum == 1) || (thrnum == 2)) {
    if((thrrise < THROTTLE_RISE_MAX) && (thrrise >= THROTTLE_RISE_MIN)) {
      psThrottles[thrnum-1]->rise = thrrise;
      return UI_OK;
    }
  }
  return UI_ERROR;
}

float throttle_get_rise(uint8_t thrnum)
{
  if((thrnum == 1) || (thrnum == 2)) {
    return psThrottles[thrnum-1]->rise;
  }
  return 0.0f;
}

float throttle_get_rise_eeprom(uint8_t thrnum)
{
  float temp = 0.0f;
  if(thrnum == 1) {
    temp = EE_ReadFloatWithDefault(EE_ADR_RISE1, 0.0f);
  } else if(thrnum == 2) {
    temp = EE_ReadFloatWithDefault(EE_ADR_RISE2, 0.0f);
  }
  return temp;
}
