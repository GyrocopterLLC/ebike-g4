#ifndef _THROTTLE_H_
#define _THROTTLE_H_

#include "stm32f4xx.h"
#include "DavidsFOCLib.h"
#include "project_parameters.h"
#include "pinconfig.h"
#include "periphconfig.h"

#define PAS_TIMER_INPUT_CLOCK     84000000 // APB1 clock * 2
#define PAS_TIM_PSC               8399 // 0.1ms per tick - 10kHz clock
#define PAS_TIM_ARR               9999 // Reset at 1 second (0->9999)
#define PAS_CLK                   10000
#define PAS_PPR                   12 // pulses per rotation (number of magnets)


#define THROTTLE_START_TIME			  1000
#define THROTTLE_START_DEADTIME	  500
#define THROTTLE_RANGE_LIMIT		  0.05f
#define THROTTLE_MIN_DEFAULT		  0.85f
#define THROTTLE_MAX_DEFAULT		  2.20f
#define THROTTLE_DROPOUT			    0.72f

#define THROTTLE_HYST_DEFAULT     0.025f
#define THROTTLE_HYST_MIN         0.001f
#define THROTTLE_HYST_MAX         0.1f
#define THROTTLE_FILT_DEFAULT     2.0f
#define THROTTLE_FILT_MIN         0.1f
#define THROTTLE_FILT_MAX         499.9f
#define THROTTLE_FILT_Q_DEFAULT   0.707f
#define THROTTLE_SAMPLING_RATE    1000.0f
// Limit the throttle climb rate to 50% / second
// The update rate is 1000Hz, so the rate limit is actually .05% per update
#define THROTTLE_RISE_DEFAULT     0.0005f
#define THROTTLE_RISE_MIN         0.00005f // Minimum of 5% / sec
#define THROTTLE_RISE_MAX         0.01f // Maximum of 1000% / sec

#define THROTTLE_OUTPUT_MIN       (0.00f)
#define THROTTLE_OUTPUT_MAX       (0.99f)

#define THROTTLE_TYPE_ANALOG      0
#define THROTTLE_TYPE_PAS         1
#define THROTTLE_TYPE_NONE        2

typedef struct
{
  uint8_t throttle_type;
  uint8_t state;
  float throttle_command;
  float prev_output;
  float hyst;
  float filt;
  float rise;
}Throttle_Type;

typedef struct
{
	uint32_t startup_count;
	float min;
	float max;
	float scale_factor;
}Throttle_Analog_Type;

typedef struct
{
  float filtered_speed;
  float scale_factor;
} Throttle_PAS_Type;

#define THROTTLE_DEFAULTS         {THROTTLE_TYPE_ANALOG, 0, 0.0f, 0.0f, \
                                    THROTTLE_HYST_DEFAULT, THROTTLE_FILT_DEFAULT, \
                                    THROTTLE_RISE_DEFAULT}
#define THROTTLE_ANALOG_DEFAULTS	{0, 0.0f, 0.0f, 1.0f}
#define THROTTLE_PAS_DEFAULTS     {0.0f, 0.015f}
#define PAS_FILTER                (0.125f) // LPF of 1/8

// Biquad filter: Fs = 1kHz, f0 = 2Hz, Q = 0.45
// Little bit sluggish response. Maybe feels safer?
#define THROTTLE_LPF_DEFAULTS  { \
                -1.972304f, \
                0.9724600f, \
                0.00003893429f, \
                0.00007786857f, \
                0.00003893429f, \
                0.0f, \
                0.0f, \
                0.0f, \
                0.0f }

void throttle_init(void);
void throttle_save_to_eeprom(void);
void throttle_switch_type(uint8_t thrnum, uint8_t thrtype);
void throttle_process(uint8_t thrnum);
void throttle_pas_process(uint8_t thrnum);
void throttle_pas_timer_overflow(uint8_t thrnum);
float throttle_get_command(uint8_t thrnum);

void throttle_init(void);
uint8_t throttle_set_type(uint8_t thrnum, uint8_t thrtype);
uint8_t throttle_get_type(uint8_t thrnum);
uint8_t throttle_set_min(uint8_t thrnum, float thrmin);
float throttle_get_min(uint8_t thrnum);
uint8_t throttle_set_max(uint8_t thrnum, float thrmax);
float throttle_get_max(uint8_t thrnum);
uint8_t throttle_set_hyst(uint8_t thrnum, float thrhyst);
float throttle_get_hyst(uint8_t thrnum);
uint8_t throttle_set_filt(uint8_t thrnum, float thrfilt);
float throttle_get_filt(uint8_t thrnum);
uint8_t throttle_set_rise(uint8_t thrnum, float thrrise);
float throttle_get_rise(uint8_t thrnum);

#endif /* _THROTTLE_H_ */
