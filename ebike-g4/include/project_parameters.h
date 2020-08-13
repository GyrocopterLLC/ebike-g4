/******************************************************************************
 * Filename: project_parameters.h
 ******************************************************************************

 Copyright (c) 2020 David Miller

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

#ifndef PROJECT_PARAMETERS_H_
#define PROJECT_PARAMETERS_H_

typedef enum _data_type {
    Data_Type_None,
    Data_Type_Int8,
    Data_Type_Int16,
    Data_Type_Int32,
    Data_Type_Float
} Data_Type;

/***  ADC Configuration Variable IDs ***/
#define CONFIG_ADC_PREFIX           (0x0000)
#define CONFIG_ADC_NUMVARS          (6)
#define CONFIG_ADC_RSHUNT           (0x0001) //F32: Shunt resistance in ohms
#define CONFIG_ADC_VBUS_RATIO       (0x0002) //F32: Bus voltage analog resistor divider ratio
#define CONFIG_ADC_VPHASE_RATIO     (0x0003) //F32: Phase voltage analog resistor divider ratio
#define CONFIG_ADC_THERM_FIXED_R    (0x0004) //F32: Temp sensor divisor resistor (on PCB)
#define CONFIG_ADC_THERM_R25        (0x0005) //F32: Temp sensor resistance at 25degC
#define CONFIG_ADC_THERM_B          (0x0006) //F32: Temp sensor beta value
/*** ADC Default Values ***/
#define DFLT_ADC_RSHUNT             (0.002f) // 0.002 Ohms
#define DFLT_ADC_VBUS_RATIO         (25.87562f) // (100kOhm + 4.02kOhm) / 4.02kOhm = 25.87562
#define DFLT_ADC_VPHASE_RATIO       (23.0f) // (22kOhm + 1kOhm) / 1kOhm = 23.0
#define DFLT_ADC_THERM_FIXED_R      (10000.0f) // 10k resistor
#define DFLT_ADC_THERM_R25          (10000.0f) // Thermistor is 10k at 25degC (ERT-J1VR103J)
#define DFLT_ADC_THERM_B            (4250.0f) // Thermistor Beta value (ERT-J1VR103J)

/*** FOC Variable IDs ***/
#define CONFIG_FOC_PREFIX           (0x0100)
#define CONFIG_FOC_NUMVARS          (6)
#define CONFIG_FOC_KP               (0x0101) //F32: Current loop proportional gain
#define CONFIG_FOC_KI               (0x0102) //F32: Current loop integral gain
#define CONFIG_FOC_KD               (0x0103) //F32: Current loop derivative gain
#define CONFIG_FOC_KC               (0x0104) //F32: Current loop integral correction gain
#define CONFIG_FOC_PWM_FREQ         (0x0105) //I32: Switching frequency (Hz)
#define CONFIG_FOC_PWM_DEADTIME     (0x0106) //I32: Switching deadtime (ns)
/*** FOC Default Values ***/
#define DFLT_FOC_KP                 (0.1f)
#define DFLT_FOC_KI                 (0.001f)
#define DFLT_FOC_KD                 (0.0f)
#define DFLT_FOC_KC                 (0.05f)
#define DFLT_FOC_PWM_FREQ           (20000)
#define DFLT_FOC_PWM_DEADTIME       (750)

/*** Main Variable IDs ***/
#define CONFIG_MAIN_PREFIX          (0x0200)
#define CONFIG_MAIN_NUMVARS         (15)
#define CONFIG_MAIN_COUNTS_TO_FOC   (0x0201) //I32: Number of PWM cycles above speed to switch to FOC
#define CONFIG_MAIN_SPEED_TO_FOC    (0x0202) //F32: Speed above which to switch to FOC
#define CONFIG_MAIN_SWITCH_EPS      (0x0203) //F32: Largest difference in angle when switching to FOC
#define CONFIG_MAIN_NUM_USB_OUTPUTS (0x0204) //I16: Number from 1-10 of USB debugging outputs
#define CONFIG_MAIN_USB_SPEED       (0x0205) //I16: Speed of USB debug, 0 through 5 (50Hz through 5kHz)
#define CONFIG_MAIN_USB_CHOICE_1    (0x0206) //I16: Choice of variable 1 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_2    (0x0207) //I16: Choice of variable 2 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_3    (0x0208) //I16: Choice of variable 3 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_4    (0x0209) //I16: Choice of variable 4 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_5    (0x020A) //I16: Choice of variable 5 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_6    (0x020B) //I16: Choice of variable 6 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_7    (0x020C) //I16: Choice of variable 7 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_8    (0x020D) //I16: Choice of variable 8 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_9    (0x020E) //I16: Choice of variable 9 on USB (1 through 19)
#define CONFIG_MAIN_USB_CHOICE_10   (0x020F) //I16: Choice of variable 10 on USB (1 through 19)
/*** Main Default Values ***/
#define DFLT_MAIN_COUNTS_TO_FOC     (200)
#define DFLT_MAIN_SPEED_TO_FOC      (5.0f)
#define DFLT_MAIN_SWITCH_EPS        (0.00833333f) // About 3 degrees
#define DFLT_MAIN_NUM_USB_OUTPUTS   (5)
#define DFLT_MAIN_USB_SPEED         (0) // Slowest (50 Hz)
#define DFLT_MAIN_USB_CHOICE_1      (1) // Ia
#define DFLT_MAIN_USB_CHOICE_2      (2) // Ib
#define DFLT_MAIN_USB_CHOICE_3      (3) // Ic
#define DFLT_MAIN_USB_CHOICE_4      (7) // Throttle
#define DFLT_MAIN_USB_CHOICE_5      (11)// Vbus
#define DFLT_MAIN_USB_CHOICE_6      (4) // Ta
#define DFLT_MAIN_USB_CHOICE_7      (5) // Tb
#define DFLT_MAIN_USB_CHOICE_8      (6) // Tc
#define DFLT_MAIN_USB_CHOICE_9      (9) // HallAngle
#define DFLT_MAIN_USB_CHOICE_10     (18)// HallState

/*** Throttle Variable IDs ***/
#define CONFIG_THRT_PREFIX          (0x0300)
#define CONFIG_THRT_NUMVARS         (6)
#define CONFIG_THRT_MIN             (0x0301) //F32: Voltage at throttle minimum
#define CONFIG_THRT_MAX             (0x0302) //F32: Voltage at throttle maximum
#define CONFIG_THRT_HYST            (0x0303) //F32: Hysteresis when switching off or on
#define CONFIG_THRT_FILT            (0x0304) //F32: Low pass filter setting (Hz)
#define CONFIG_THRT_RISE            (0x0305) //F32: Maximum amount of throttle rise per count
#define CONFIG_THRT_RATIO           (0x0306) //F32: Throttle analog resistor divider ratio
/*** Throttle Default Values ***/
#define DFLT_THRT_MIN               (0.90f)
#define DFLT_THRT_MAX               (4.00f)
#define DFLT_THRT_HYST              (0.025f)
#define DFLT_THRT_FILT              (2.0f)
#define DFLT_THRT_RISE              (0.0005f) // 0->100 in 2 seconds
#define DFLT_THRT_RATIO             (1.499f) // Rtop = 4.99k, Rbot = 10k, Ratio = (10+4.99)/10 = 1.499

/*** Limit Variable IDs ***/
#define CONFIG_LMT_PREFIX               (0x0400)
#define CONFIG_LMT_NUMVARS              (13)
#define CONFIG_LMT_VOLT_FAULT_MIN       (0x0401) //F32: Trip fault code when voltage below this
#define CONFIG_LMT_VOLT_FAULT_MAX       (0x0402) //F32: Fault when voltage above this
#define CONFIG_LMT_CUR_FAULT_MAX        (0x0403) //F32: Fault when current (any phase) above this
#define CONFIG_LMT_VOLT_SOFTCAP         (0x0404) //F32: Start reducing current limit ("limp mode")
#define CONFIG_LMT_VOLT_HARDCAP         (0x0405) //F32: No more current below this
#define CONFIG_LMT_PHASE_CUR_MAX        (0x0406) //F32: Maximum throttle = this current
#define CONFIG_LMT_PHASE_REGEN_MAX      (0x0407) //F32: Maximum demand regen = this current
#define CONFIG_LMT_BATT_CUR_MAX         (0x0408) //F32: Clip demanded throttle when battery current at this
#define CONFIG_LMT_BATT_REGEN_MAX       (0x0409) //F32: Clip demanded regen when battery charge current here
#define CONFIG_LMT_FET_TEMP_SOFTCAP     (0x040A) //F32: Soften current when FET temps here
#define CONFIG_LMT_FET_TEMP_HARDCAP     (0x040B) //F32: No more current when FET temps here
#define CONFIG_LMT_MOTOR_TEMP_SOFTCAP   (0x040C) //F32: Soften current when motor temp here
#define CONFIG_LMT_MOTOR_TEMP_HARDCAP   (0x040D) //F32: No more current when motor temp here
/*** Limit Default Values ***/
#define DFLT_LMT_VOLT_FAULT_MIN         (44.8f) // 2.8 x 16 cells
#define DFLT_LMT_VOLT_FAULT_MAX         (70.4f) // 4.4 x 16 cells
#define DFLT_LMT_CUR_FAULT_MAX          (74.0f) // Just below max sensing level
#define DFLT_LMT_VOLT_SOFTCAP           (10.0f) // For testing, just apply 12V or more
#define DFLT_LMT_VOLT_HARDCAP           (8.0f)
#define DFLT_LMT_PHASE_CUR_MAX          (60.0f)
#define DFLT_LMT_PHASE_REGEN_MAX        (5.0f)
#define DFLT_LMT_BATT_CUR_MAX           (30.0f)
#define DFLT_LMT_BATT_REGEN_MAX         (5.0f)
#define DFLT_LMT_FET_TEMP_SOFTCAP       (75.0f)
#define DFLT_LMT_FET_TEMP_HARDCAP       (90.0f)
#define DFLT_LMT_MOTOR_TEMP_SOFTCAP     (75.0f)
#define DFLT_LMT_MOTOR_TEMP_HARDCAP     (90.0f)

/*** Motor Configuration Variable IDs ***/
#define CONFIG_MOTOR_PREFIX         (0x0500)
#define CONFIG_MOTOR_NUMVARS        (10)
#define CONFIG_MOTOR_HALL1          (0x0501) //F32: Angle of motor when switching into state 1, forward rotation
#define CONFIG_MOTOR_HALL2          (0x0502) //F32: Angle when switching into state 2
#define CONFIG_MOTOR_HALL3          (0x0503) //F32: Angle when switching into state 3
#define CONFIG_MOTOR_HALL4          (0x0504) //F32: Angle when switching into state 4
#define CONFIG_MOTOR_HALL5          (0x0505) //F32: Angle when switching into state 5
#define CONFIG_MOTOR_HALL6          (0x0506) //F32: Angle when switching into state 6
#define CONFIG_MOTOR_POLEPAIRS      (0x0507) //I16: Turns of electrical / turns of mechanical
#define CONFIG_MOTOR_GEAR_RATIO     (0x0508) //F32: Turns of mechanical motor / turns of wheel
#define CONFIG_MOTOR_WHEEL_SIZE     (0x0509) //F32: Diameter in mm
#define CONFIG_MOTOR_KV             (0x050A) //F32: Motor voltage constant (RPM / Volt)
/*** Motor Default Values ***/
// For Ebikeling 700C front 1200W motor (2->6->4->5->1->3)

// State angles sensed by freewheeling with phase voltage sensing
// Forward rotation:
// rising A (4 to 5):   0.5741980041423249
// falling C (5 to 1):  0.7215875099416836
// rising B (1 to 3):   0.8956964002575087
// falling A (3 to 2):  0.07304103128566627
// rising C (2 to 6):   0.22672750318891483
// falling B (6 to 4):  0.4016299738288212

// Reverse rotation:
// falling A (5 to 4):  0.5650003575148457
// rising B (4 to 6):   0.3910043952868503
// falling C (6 to 2):  0.21344155930285352
// rising A (2 to 3):   0.06525376758132169
// falling B (3 to 1):  0.8970313516299208
// rising C (1 to 5):   0.7205920218932402

// Defined for entering a state in forward rotation
// Average for Hall1:   0.721089765917462
// Average for Hall2:   0.06914739943349399
// Average for Hall3:   0.8963638759437147
// Average for Hall4:   0.39631718455783577
// Average for Hall5:   0.5695991808285853
// Average for Hall6:   0.22008453124588417

// Previous version
//#define DFLT_MOTOR_HALL1            (0.743786f)
//#define DFLT_MOTOR_HALL2            (0.089677f)
//#define DFLT_MOTOR_HALL3            (0.905861f)
//#define DFLT_MOTOR_HALL4            (0.412525f)
//#define DFLT_MOTOR_HALL5            (0.593083f)
//#define DFLT_MOTOR_HALL6            (0.240492f)
// New version
#define DFLT_MOTOR_HALL1            (0.721090f)
#define DFLT_MOTOR_HALL2            (0.0691474f)
#define DFLT_MOTOR_HALL3            (0.896364f)
#define DFLT_MOTOR_HALL4            (0.396317f)
#define DFLT_MOTOR_HALL5            (0.569599f)
#define DFLT_MOTOR_HALL6            (0.220085f)
#define DFLT_MOTOR_POLEPAIRS        (23)
#define DFLT_MOTOR_GEAR_RATIO       (1.0f) // Direct drive
#define DFLT_MOTOR_WHEEL_SIZE       (700.28f) // 700C-40 according to www.cateye.com tire size chart
                                                // https://www.cateye.com/data/resources/Tire_size_chart_ENG_151106.pdf
                                                // 2200 mm / pi = 700.28mm
#define DFLT_MOTOR_KV               (7.5f) // When zero, PI loop feedforward is disabled


/*** Three Phase Driver Variable IDs ***/
#define CONFIG_DRV_PREFIX           (0x0600)
#define CONFIG_DRV_NUMVARS          (3)
#define CONFIG_DRV_GATE_STRENGTH    (0x0601) //I32: bitmapped for drive strength of pull-up and pull-down, for both high and low side FETs
                                             //     Between 50mA to 1000mA for pull-up, 100mA to 2000mA for pull-down. See DRV8353 datasheet.
                                             //     Bits [3:0] = IDRIVEN_LS, bits [7:4] = IDRIVEP_LS, bits [11:8] = IDRIVEN_HS, bits [15:12] = IDRIVEP_HS
#define CONFIG_DRV_VDS_LIMIT        (0x0602) //I8: Setting between 0.06V and 2V for the VDS fault threshold. See DRV8353 datasheet.
#define CONFIG_DRV_CSA_GAIN         (0x0603) //I8: 0 = x5, 1 = x10, 2 = x20, 3 = x40

/*** Three Phase Driver Default Values ***/
#define DFLT_DRV_GATE_STRENGTH      (0x4444) // Set to 300mA pull-up, 600mA pull-down for both high and low side drivers
#define DFLT_DRV_VDS_LIMIT          (0x05)   // Set to 0.6V (about 100A if the FET has its worst-case Rdson of 6mOhm)
#define DFLT_DRV_CSA_GAIN           (0x01)   // x10 gain

/*** BMS Interactions ***/
#define CONFIG_BMS_PREFIX           (0x1A00)
#define CONFIG_BMS_ISCONNECTED      (0x1A01) //I8: Zero for not connected, one for connected
#define CONFIG_BMS_NUMBATTS         (0x1A02) //I16: Total number of batteries in the chain
#define CONFIG_BMS_GETBAT_N         (0x1A03) //F32: Voltage of a particular cell (requires 2-byte cell number, zero indexed)
#define CONFIG_BMS_GETSTATUS_N      (0x1A04) //I32: Status of a particular cell (requires 2-byte cell number, zero indexed)

/*** For EEPROM settings ***/
#define TOTAL_EE_VARS   (CONFIG_ADC_NUMVARS + CONFIG_FOC_NUMVARS \
                        + CONFIG_MAIN_NUMVARS + CONFIG_THRT_NUMVARS \
                        + CONFIG_LMT_NUMVARS + CONFIG_MOTOR_NUMVARS \
                        + CONFIG_DRV_NUMVARS)

/*** Routines - set to start ***/
#define ROUTINE_SAVE_ALL_EEPROM     (0x0101)
#define ROUTINE_LOAD_ALL_EEPROM     (0x0102)

#define ROUTINE_HALL_DETECT         (0x0201)

#define ROUTINE_SOFT_RESET          (0x0301)
#define ROUTINE_BOOTLOADER_RESET    (0x0302)

/*** Features - toggle on or off ***/
#define FEATURE_SERIAL_DATA         (0x0001)
#define FEATURE_BLDC_MODE           (0x0002)
#define FEATURE_SINE_MODE           (0x0003)
#define FEATURE_DEBUG_PWM           (0x0004)

/*** Dashboard Data Format ***/
#define DASHBOARD_DATA_LENGTH       (8*4)
// Param1: F32: Throttle position (%)
// Param2: F32: Speed (rpm)
// Param3: F32: Phase Amps
// Param4: F32: Battery Amps
// Param5: F32: Battery Volts
// Param6: F32: Controller FET Temperature (degC)
// Param7: F32: Motor Temperature (degC)
// Param8: I32: Fault Code

// For the Hall sensor detection routine
#define HALL_DETECT_RAMP_SPEED          (5.0f) // 5 Hz = 300 eRPM = 13 RPM
#define HALL_DETECT_MIN_TRANSITIONS     (16)
#define HALL_DETECT_TRANSITIONS_TO_AVG  (16)
#define HALL_DETECT_TIMEOUT_MS          (1500)

// Angle definitions - integer
// This set of defines are the integer values of angles
// as defined for a 16 bit unsigned integer.
#define U16_0_DEG       ((uint16_t)0)
#define U16_30_DEG      ((uint16_t)5461)
#define U16_60_DEG      ((uint16_t)10923)
#define U16_90_DEG      ((uint16_t)16384)
#define U16_120_DEG     ((uint16_t)21845)
#define U16_150_DEG     ((uint16_t)27307)
#define U16_180_DEG     ((uint16_t)32768)
#define U16_210_DEG     ((uint16_t)38229)
#define U16_240_DEG     ((uint16_t)43691)
#define U16_270_DEG     ((uint16_t)49152)
#define U16_300_DEG     ((uint16_t)54613)
#define U16_330_DEG     ((uint16_t)60075)

// Angle definitions - floating point
// This set of defines are the integer values of angles
// as defined for a single-precision float.
// Arbitrary choice of 6 significant figures.
#define F32_0_DEG       (0.0f)
#define F32_30_DEG      (0.0833333f)
#define F32_60_DEG      (0.166667f)
#define F32_90_DEG      (0.250000f)
#define F32_120_DEG     (0.333333f)
#define F32_150_DEG     (0.416667f)
#define F32_180_DEG     (0.500000f)
#define F32_210_DEG     (0.583333f)
#define F32_240_DEG     (0.666667f)
#define F32_270_DEG     (0.750000f)
#define F32_300_DEG     (0.833333f)
#define F32_330_DEG     (0.916667f)

// USB Live Data Definitions
#define MAX_LIVE_OUTPUTS            (10)
#define MAX_LIVE_SPEED_CHOICES      (6)  // 0: 50Hz, 1: 100Hz, 2: 200Hz, 3: 500Hz, 4: 1kHz, 5: 5kHz
//Debugging outputs
#define MAX_LIVE_DATA_CHOICES       (21)
#define LIVE_CHOICE_UNUSED          (0)
#define LIVE_CHOICE_IA              (1)
#define LIVE_CHOICE_IB              (2)
#define LIVE_CHOICE_IC              (3)
#define LIVE_CHOICE_TA              (4)
#define LIVE_CHOICE_TB              (5)
#define LIVE_CHOICE_TC              (6)
#define LIVE_CHOICE_THROTTLE        (7)
#define LIVE_CHOICE_HALLANGLE       (8)
#define LIVE_CHOICE_HALLSPEED       (9)
#define LIVE_CHOICE_HALLACCEL       (10)
#define LIVE_CHOICE_HALLSTATE       (11)
#define LIVE_CHOICE_VBUS            (12)
#define LIVE_CHOICE_ID              (13)
#define LIVE_CHOICE_IQ              (14)
#define LIVE_CHOICE_TD              (15)
#define LIVE_CHOICE_TQ              (16)
#define LIVE_CHOICE_VA              (17)
#define LIVE_CHOICE_VB              (18)
#define LIVE_CHOICE_VC              (19)
#define LIVE_CHOICE_FTEMP           (20)
#define LIVE_CHOICE_ERRORCODE       (21)


#endif /* PROJECT_PARAMETERS_H_ */
