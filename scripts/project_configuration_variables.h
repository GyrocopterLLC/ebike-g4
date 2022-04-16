
/******************************************************************************
 * Filename: project_configuration_variables.h
 ******************************************************************************

 Copyright (c) 2022 David Miller

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
 

// Auto-generated file. Do not edit!
// If changes are needed, edit the variable settings in 'datavars.xml' and
// re-run 'datavar_codegen.py' to recreate this file.

#ifndef PROJECT_CONFIGURATION_VARIABLES_H_
#define PROJECT_CONFIGURATION_VARIABLES_H_


/***  ADC Configuration Variable IDs ***/
#define CONFIG_ADC_NUMVARS              (6)
#define CONFIG_ADC_PREFIX               (0x0100)
#define CONFIG_ADC_RSHUNT               (0x0101) //f32:Shunt resistance in Ohms
#define CONFIG_ADC_VBUS_RATIO           (0x0102) //f32:Resistor divider ratio on Vbus analog input
#define CONFIG_ADC_VPHASE_RATIO         (0x0103) //f32:Resistor divider ratio on phase voltage analog inputs
#define CONFIG_ADC_THERM_FIXED_R        (0x0104) //f32:Pulldown resistor value in thermistor circuit
#define CONFIG_ADC_THERM_R25            (0x0105) //f32:Thermistor resistance value at 25degC
#define CONFIG_ADC_THERM_B              (0x0106) //f32:Thermistor Beta value at 25/50degC
/***  ADC Default Values ***/
#define DFLT_ADC_RSHUNT                 (0.002f)
#define DFLT_ADC_VBUS_RATIO             (25.87562f)
#define DFLT_ADC_VPHASE_RATIO           (23.0f)
#define DFLT_ADC_THERM_FIXED_R          (10000.0f)
#define DFLT_ADC_THERM_R25              (10000.0f)
#define DFLT_ADC_THERM_B                (4250.0f)
/***  ADC Minimum Values ***/
#define MIN_ADC_RSHUNT                  (0.0001f)
#define MIN_ADC_VBUS_RATIO              (1.0f)
#define MIN_ADC_VPHASE_RATIO            (1.0f)
#define MIN_ADC_THERM_FIXED_R           (100.0f)
#define MIN_ADC_THERM_R25               (100.0f)
#define MIN_ADC_THERM_B                 (100.0f)
/***  ADC Maximum Values ***/
#define MIN_ADC_RSHUNT                  (100.0f)
#define MIN_ADC_VBUS_RATIO              (1000.0f)
#define MIN_ADC_VPHASE_RATIO            (1000.0f)
#define MIN_ADC_THERM_FIXED_R           (1000000.0f)
#define MIN_ADC_THERM_R25               (1000000.0f)
#define MIN_ADC_THERM_B                 (1000000.0f)


/***  FOC Configuration Variable IDs ***/
#define CONFIG_FOC_NUMVARS              (6)
#define CONFIG_FOC_PREFIX               (0x0200)
#define CONFIG_FOC_KP                   (0x0201) //f32:Current PID loop proportional gain
#define CONFIG_FOC_KI                   (0x0202) //f32:Current PID loop integral gain
#define CONFIG_FOC_KD                   (0x0203) //f32:Current PID loop derivative gain
#define CONFIG_FOC_KC                   (0x0204) //f32:Current PID loop correctional or windup gain
#define CONFIG_FOC_PWM_FREQ             (0x0205) //i32:Switching frequency of PWM outputs
#define CONFIG_FOC_PWM_DEADTIME         (0x0206) //i32:Deadtime (time when both pos and neg outputs are off) in nanoseconds
/***  FOC Default Values ***/
#define DFLT_FOC_KP                     (0.1f)
#define DFLT_FOC_KI                     (0.001f)
#define DFLT_FOC_KD                     (0.0f)
#define DFLT_FOC_KC                     (0.05f)
#define DFLT_FOC_PWM_FREQ               (20000)
#define DFLT_FOC_PWM_DEADTIME           (750)
/***  FOC Minimum Values ***/
#define MIN_FOC_KP                      (0.0001f)
#define MIN_FOC_KI                      (1e-06f)
#define MIN_FOC_KD                      (1e-06f)
#define MIN_FOC_KC                      (1e-06f)
#define MIN_FOC_PWM_FREQ                (5000)
#define MIN_FOC_PWM_DEADTIME            (25)
/***  FOC Maximum Values ***/
#define MIN_FOC_KP                      (1000.0f)
#define MIN_FOC_KI                      (1.0f)
#define MIN_FOC_KD                      (1000.0f)
#define MIN_FOC_KC                      (1000.0f)
#define MIN_FOC_PWM_FREQ                (40000)
#define MIN_FOC_PWM_DEADTIME            (1500)


/***  MAIN Configuration Variable IDs ***/
#define CONFIG_MAIN_NUMVARS             (15)
#define CONFIG_MAIN_PREFIX              (0x0300)
#define CONFIG_MAIN_COUNTS_TO_FOC       (0x0301) //i32:Number of PWM cycles above speed to switch to FOC
#define CONFIG_MAIN_SPEED_TO_FOC        (0x0302) //f32:Speed above which to switch to FOC (Hz)
#define CONFIG_MAIN_SWITCH_EPS          (0x0303) //f32:Largest difference in angle when switching to FOC
#define CONFIG_MAIN_NUM_USB_OUTPUTS     (0x0304) //i16:Number of simultaneous debugging outputs sent over USB
#define CONFIG_MAIN_USB_SPEED           (0x0305) //i16:Speed at which data is sent over USB. 0=50Hz, 1=100Hz, 2=200Hz, 3=500Hz, 4=1kHz, 5=5kHz
#define CONFIG_MAIN_USB_CHOICE_1        (0x0306) //i16:Choice of USB debugging output number 1
#define CONFIG_MAIN_USB_CHOICE_2        (0x0307) //i16:Choice of USB debugging output number 2
#define CONFIG_MAIN_USB_CHOICE_3        (0x0308) //i16:Choice of USB debugging output number 3
#define CONFIG_MAIN_USB_CHOICE_4        (0x0309) //i16:Choice of USB debugging output number 4
#define CONFIG_MAIN_USB_CHOICE_5        (0x030A) //i16:Choice of USB debugging output number 5
#define CONFIG_MAIN_USB_CHOICE_6        (0x030B) //i16:Choice of USB debugging output number 6
#define CONFIG_MAIN_USB_CHOICE_7        (0x030C) //i16:Choice of USB debugging output number 7
#define CONFIG_MAIN_USB_CHOICE_8        (0x030D) //i16:Choice of USB debugging output number 8
#define CONFIG_MAIN_USB_CHOICE_9        (0x030E) //i16:Choice of USB debugging output number 9
#define CONFIG_MAIN_USB_CHOICE_10       (0x030F) //i16:Choice of USB debugging output number 10
/***  MAIN Default Values ***/
#define DFLT_MAIN_COUNTS_TO_FOC         (200)
#define DFLT_MAIN_SPEED_TO_FOC          (5.0f)
#define DFLT_MAIN_SWITCH_EPS            (0.00833333f)
#define DFLT_MAIN_NUM_USB_OUTPUTS       (5)
#define DFLT_MAIN_USB_SPEED             (0)
#define DFLT_MAIN_USB_CHOICE_1          (1)
#define DFLT_MAIN_USB_CHOICE_2          (2)
#define DFLT_MAIN_USB_CHOICE_3          (3)
#define DFLT_MAIN_USB_CHOICE_4          (7)
#define DFLT_MAIN_USB_CHOICE_5          (11)
#define DFLT_MAIN_USB_CHOICE_6          (4)
#define DFLT_MAIN_USB_CHOICE_7          (5)
#define DFLT_MAIN_USB_CHOICE_8          (6)
#define DFLT_MAIN_USB_CHOICE_9          (9)
#define DFLT_MAIN_USB_CHOICE_10         (18)
/***  MAIN Minimum Values ***/
#define MIN_MAIN_COUNTS_TO_FOC          (1)
#define MIN_MAIN_SPEED_TO_FOC           (0.1f)
#define MIN_MAIN_SWITCH_EPS             (1e-05f)
#define MIN_MAIN_NUM_USB_OUTPUTS        (1)
#define MIN_MAIN_USB_SPEED              (0)
#define MIN_MAIN_USB_CHOICE_1           (0)
#define MIN_MAIN_USB_CHOICE_2           (0)
#define MIN_MAIN_USB_CHOICE_3           (0)
#define MIN_MAIN_USB_CHOICE_4           (0)
#define MIN_MAIN_USB_CHOICE_5           (0)
#define MIN_MAIN_USB_CHOICE_6           (0)
#define MIN_MAIN_USB_CHOICE_7           (0)
#define MIN_MAIN_USB_CHOICE_8           (0)
#define MIN_MAIN_USB_CHOICE_9           (0)
#define MIN_MAIN_USB_CHOICE_10          (0)
/***  MAIN Maximum Values ***/
#define MIN_MAIN_COUNTS_TO_FOC          (10000)
#define MIN_MAIN_SPEED_TO_FOC           (100.0f)
#define MIN_MAIN_SWITCH_EPS             (1.0f)
#define MIN_MAIN_NUM_USB_OUTPUTS        (10)
#define MIN_MAIN_USB_SPEED              (5)
#define MIN_MAIN_USB_CHOICE_1           (22)
#define MIN_MAIN_USB_CHOICE_2           (22)
#define MIN_MAIN_USB_CHOICE_3           (22)
#define MIN_MAIN_USB_CHOICE_4           (22)
#define MIN_MAIN_USB_CHOICE_5           (22)
#define MIN_MAIN_USB_CHOICE_6           (22)
#define MIN_MAIN_USB_CHOICE_7           (22)
#define MIN_MAIN_USB_CHOICE_8           (22)
#define MIN_MAIN_USB_CHOICE_9           (22)
#define MIN_MAIN_USB_CHOICE_10          (22)


/***  THRT Configuration Variable IDs ***/
#define CONFIG_THRT_NUMVARS             (6)
#define CONFIG_THRT_PREFIX              (0x0400)
#define CONFIG_THRT_MIN                 (0x0401) //f32:Voltage on ADC when throttle is at minimum position
#define CONFIG_THRT_MAX                 (0x0402) //f32:Voltage on ADC when throttle is at maximum position
#define CONFIG_THRT_HYST                (0x0403) //f32:Hysteresis when switching on or off in volts
#define CONFIG_THRT_FILT                (0x0404) //f32:Low-pass filter cutoff frequency in Hz
#define CONFIG_THRT_RISE                (0x0405) //f32:Maximum amount of throttle percent rise per 1KHz count
#define CONFIG_THRT_RATIO               (0x0406) //f32:Resistor divider ratio for throttle analog input
/***  THRT Default Values ***/
#define DFLT_THRT_MIN                   (0.9f)
#define DFLT_THRT_MAX                   (4.0f)
#define DFLT_THRT_HYST                  (0.025f)
#define DFLT_THRT_FILT                  (2.0f)
#define DFLT_THRT_RISE                  (0.0005f)
#define DFLT_THRT_RATIO                 (1.499f)
/***  THRT Minimum Values ***/
#define MIN_THRT_MIN                    (0.0f)
#define MIN_THRT_MAX                    (0.0f)
#define MIN_THRT_HYST                   (0.0f)
#define MIN_THRT_FILT                   (0.1f)
#define MIN_THRT_RISE                   (0.0001f)
#define MIN_THRT_RATIO                  (1.0f)
/***  THRT Maximum Values ***/
#define MIN_THRT_MIN                    (10.0f)
#define MIN_THRT_MAX                    (10.0f)
#define MIN_THRT_HYST                   (1.0f)
#define MIN_THRT_FILT                   (1000.0f)
#define MIN_THRT_RISE                   (0.1f)
#define MIN_THRT_RATIO                  (1000.0f)


/***  LMT Configuration Variable IDs ***/
#define CONFIG_LMT_NUMVARS              (13)
#define CONFIG_LMT_PREFIX               (0x0500)
#define CONFIG_LMT_VOLT_FAULT_MIN       (0x0501) //f32:Fault when voltage below this
#define CONFIG_LMT_VOLT_FAULT_MAX       (0x0502) //f32:Fault when voltage above this
#define CONFIG_LMT_CUR_FAULT_MAX        (0x0503) //f32:Fault when current above this
#define CONFIG_LMT_VOLT_SOFTCAP         (0x0504) //f32:Motor begins lowering power when voltage below this
#define CONFIG_LMT_VOLT_HARDCAP         (0x0505) //f32:Motor no longer functions when voltage below this
#define CONFIG_LMT_PHASE_CUR_MAX        (0x0506) //f32:Maximum current at motor phase
#define CONFIG_LMT_PHASE_REGEN_MAX      (0x0507) //f32:Maximum reverse current at motor phase
#define CONFIG_LMT_BATT_CUR_MAX         (0x0508) //f32:Maximum current at batter
#define CONFIG_LMT_BATT_REGEN_MAX       (0x0509) //f32:Maximum reverse current at battery
#define CONFIG_LMT_FET_TEMP_SOFTCAP     (0x050A) //f32:Motor power reduced when controller temp above this
#define CONFIG_LMT_FET_TEMP_HARDCAP     (0x050B) //f32:Motor disabled when controller temp above this
#define CONFIG_LMT_MOTOR_TEMP_SOFTCAP   (0x050C) //f32:Motor power reduced when motor temp above this
#define CONFIG_LMT_MOTOR_TEMP_HARDCAP   (0x050D) //f32:Motor disabled when motor temp above this
/***  LMT Default Values ***/
#define DFLT_LMT_VOLT_FAULT_MIN         (44.8f)
#define DFLT_LMT_VOLT_FAULT_MAX         (70.4f)
#define DFLT_LMT_CUR_FAULT_MAX          (74.0f)
#define DFLT_LMT_VOLT_SOFTCAP           (10.0f)
#define DFLT_LMT_VOLT_HARDCAP           (8.0f)
#define DFLT_LMT_PHASE_CUR_MAX          (60.0f)
#define DFLT_LMT_PHASE_REGEN_MAX        (5.0f)
#define DFLT_LMT_BATT_CUR_MAX           (30.0f)
#define DFLT_LMT_BATT_REGEN_MAX         (5.0f)
#define DFLT_LMT_FET_TEMP_SOFTCAP       (75.0f)
#define DFLT_LMT_FET_TEMP_HARDCAP       (90.0f)
#define DFLT_LMT_MOTOR_TEMP_SOFTCAP     (75.0f)
#define DFLT_LMT_MOTOR_TEMP_HARDCAP     (90.0f)
/***  LMT Minimum Values ***/
#define MIN_LMT_VOLT_FAULT_MIN          (1.0f)
#define MIN_LMT_VOLT_FAULT_MAX          (1.0f)
#define MIN_LMT_CUR_FAULT_MAX           (1.0f)
#define MIN_LMT_VOLT_SOFTCAP            (1.0f)
#define MIN_LMT_VOLT_HARDCAP            (1.0f)
#define MIN_LMT_PHASE_CUR_MAX           (1.0f)
#define MIN_LMT_PHASE_REGEN_MAX         (1.0f)
#define MIN_LMT_BATT_CUR_MAX            (1.0f)
#define MIN_LMT_BATT_REGEN_MAX          (1.0f)
#define MIN_LMT_FET_TEMP_SOFTCAP        (20.0f)
#define MIN_LMT_FET_TEMP_HARDCAP        (20.0f)
#define MIN_LMT_MOTOR_TEMP_SOFTCAP      (20.0f)
#define MIN_LMT_MOTOR_TEMP_HARDCAP      (20.0f)
/***  LMT Maximum Values ***/
#define MIN_LMT_VOLT_FAULT_MIN          (250.0f)
#define MIN_LMT_VOLT_FAULT_MAX          (250.0f)
#define MIN_LMT_CUR_FAULT_MAX           (75.0f)
#define MIN_LMT_VOLT_SOFTCAP            (250.0f)
#define MIN_LMT_VOLT_HARDCAP            (250.0f)
#define MIN_LMT_PHASE_CUR_MAX           (75.0f)
#define MIN_LMT_PHASE_REGEN_MAX         (75.0f)
#define MIN_LMT_BATT_CUR_MAX            (75.0f)
#define MIN_LMT_BATT_REGEN_MAX          (75.0f)
#define MIN_LMT_FET_TEMP_SOFTCAP        (150.0f)
#define MIN_LMT_FET_TEMP_HARDCAP        (150.0f)
#define MIN_LMT_MOTOR_TEMP_SOFTCAP      (150.0f)
#define MIN_LMT_MOTOR_TEMP_HARDCAP      (150.0f)


/***  MOTOR Configuration Variable IDs ***/
#define CONFIG_MOTOR_NUMVARS            (9)
#define CONFIG_MOTOR_PREFIX             (0x0600)
#define CONFIG_MOTOR_HALL1              (0x0601) //f32:Angle when switching into hall state 1, forward rotation
#define CONFIG_MOTOR_HALL2              (0x0602) //f32:Angle when switching into hall state 2, forward rotation
#define CONFIG_MOTOR_HALL3              (0x0603) //f32:Angle when switching into hall state 3, forward rotation
#define CONFIG_MOTOR_HALL4              (0x0604) //f32:Angle when switching into hall state 4, forward rotation
#define CONFIG_MOTOR_HALL5              (0x0605) //f32:Angle when switching into hall state 5, forward rotation
#define CONFIG_MOTOR_HALL6              (0x0606) //f32:Angle when switching into hall state 6, forward rotation
#define CONFIG_MOTOR_POLEPAIRS          (0x0607) //i16:Electrical to mechanical turns ratio
#define CONFIG_MOTOR_GEAR_RATIO         (0x0608) //f32:Motor to wheel gear ratio
#define CONFIG_MOTOR_WHEEL_SIZE         (0x0609) //f32:Wheel Diameter in mm
/***  MOTOR Default Values ***/
#define DFLT_MOTOR_HALL1                (0.72109f)
#define DFLT_MOTOR_HALL2                (0.0691474f)
#define DFLT_MOTOR_HALL3                (0.896364f)
#define DFLT_MOTOR_HALL4                (0.396317f)
#define DFLT_MOTOR_HALL5                (0.569599f)
#define DFLT_MOTOR_HALL6                (0.220085f)
#define DFLT_MOTOR_POLEPAIRS            (23)
#define DFLT_MOTOR_GEAR_RATIO           (1.0f)
#define DFLT_MOTOR_WHEEL_SIZE           (700.28f)
/***  MOTOR Minimum Values ***/
#define MIN_MOTOR_HALL1                 (0.0f)
#define MIN_MOTOR_HALL2                 (0.0f)
#define MIN_MOTOR_HALL3                 (0.0f)
#define MIN_MOTOR_HALL4                 (0.0f)
#define MIN_MOTOR_HALL5                 (0.0f)
#define MIN_MOTOR_HALL6                 (0.0f)
#define MIN_MOTOR_POLEPAIRS             (1)
#define MIN_MOTOR_GEAR_RATIO            (1.0f)
#define MIN_MOTOR_WHEEL_SIZE            (100.0f)
/***  MOTOR Maximum Values ***/
#define MIN_MOTOR_HALL1                 (1.0f)
#define MIN_MOTOR_HALL2                 (1.0f)
#define MIN_MOTOR_HALL3                 (1.0f)
#define MIN_MOTOR_HALL4                 (1.0f)
#define MIN_MOTOR_HALL5                 (1.0f)
#define MIN_MOTOR_HALL6                 (1.0f)
#define MIN_MOTOR_POLEPAIRS             (1000)
#define MIN_MOTOR_GEAR_RATIO            (1000.0f)
#define MIN_MOTOR_WHEEL_SIZE            (2000.0f)


#endif // PROJECT_CONFIGURATION_VARIABLES_H_
