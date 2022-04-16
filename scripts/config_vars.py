
#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Auto-generated file. Do not edit!
If changes are needed, edit the variable settings in 'datavars.xml' and
re-run 'datavar_codegen.py' to recreate this file.

Defines the configuration variables in the motor controller
"""

from collections import namedtuple

Config_Var = namedtuple('Config_Var', ['name','id','format','longname','description','default','min','max'])

MCU_Config_Vars = {
	'ADC':[

            Config_Var(name='RSHUNT',id=0x0101,format='f32',
            default=0.002,min=0.0001,max=100.0,
            longname='Shunt Resistance',
            description='Shunt resistance in Ohms'),

            Config_Var(name='VBUS_RATIO',id=0x0102,format='f32',
            default=25.87562,min=1.0,max=1000.0,
            longname='Vbus Divider Ratio',
            description='Resistor divider ratio on Vbus analog input'),

            Config_Var(name='VPHASE_RATIO',id=0x0103,format='f32',
            default=23.0,min=1.0,max=1000.0,
            longname='Vphase Divider Ratio',
            description='Resistor divider ratio on phase voltage analog inputs'),

            Config_Var(name='THERM_FIXED_R',id=0x0104,format='f32',
            default=10000.0,min=100.0,max=1000000.0,
            longname='Thermistor Pulldown Fixed Resistance',
            description='Pulldown resistor value in thermistor circuit'),

            Config_Var(name='THERM_R25',id=0x0105,format='f32',
            default=10000.0,min=100.0,max=1000000.0,
            longname='Thermistor 25degC Resistance',
            description='Thermistor resistance value at 25degC'),

            Config_Var(name='THERM_B',id=0x0106,format='f32',
            default=4250.0,min=100.0,max=1000000.0,
            longname='Thermistor B25/50 Ratio',
            description='Thermistor Beta value at 25/50degC'),
	],
	'FOC':[

            Config_Var(name='KP',id=0x0201,format='f32',
            default=0.1,min=0.0001,max=1000.0,
            longname='Proportional Gain',
            description='Current PID loop proportional gain'),

            Config_Var(name='KI',id=0x0202,format='f32',
            default=0.001,min=1e-06,max=1.0,
            longname='Integral Gain',
            description='Current PID loop integral gain'),

            Config_Var(name='KD',id=0x0203,format='f32',
            default=0.0,min=1e-06,max=1000.0,
            longname='Derivative Gain',
            description='Current PID loop derivative gain'),

            Config_Var(name='KC',id=0x0204,format='f32',
            default=0.05,min=1e-06,max=1000.0,
            longname='Windup Gain',
            description='Current PID loop correctional or windup gain'),

            Config_Var(name='PWM_FREQ',id=0x0205,format='i32',
            default=20000,min=5000,max=40000,
            longname='PWM Frequency',
            description='Switching frequency of PWM outputs'),

            Config_Var(name='PWM_DEADTIME',id=0x0206,format='i32',
            default=750,min=25,max=1500,
            longname='PWM Deadtime',
            description='Deadtime (time when both pos and neg outputs are off) in nanoseconds'),
	],
	'MAIN':[

            Config_Var(name='COUNTS_TO_FOC',id=0x0301,format='i32',
            default=200,min=1,max=10000,
            longname='Counts until FOC',
            description='Number of PWM cycles above speed to switch to FOC'),

            Config_Var(name='SPEED_TO_FOC',id=0x0302,format='f32',
            default=5.0,min=0.1,max=100.0,
            longname='Speed at FOC',
            description='Speed above which to switch to FOC (Hz)'),

            Config_Var(name='SWITCH_EPS',id=0x0303,format='f32',
            default=0.00833333,min=1e-05,max=1.0,
            longname='Switch Espilon',
            description='Largest difference in angle when switching to FOC'),

            Config_Var(name='NUM_USB_OUTPUTS',id=0x0304,format='i16',
            default=5,min=1,max=10,
            longname='Number of USB outputs',
            description='Number of simultaneous debugging outputs sent over USB'),

            Config_Var(name='USB_SPEED',id=0x0305,format='i16',
            default=0,min=0,max=5,
            longname='USB Debugging Data Rate',
            description='Speed at which data is sent over USB. 0=50Hz, 1=100Hz, 2=200Hz, 3=500Hz, 4=1kHz, 5=5kHz'),

            Config_Var(name='USB_CHOICE_1',id=0x0306,format='i16',
            default=1,min=0,max=22,
            longname='USB Choice 1',
            description='Choice of USB debugging output number 1'),

            Config_Var(name='USB_CHOICE_2',id=0x0307,format='i16',
            default=2,min=0,max=22,
            longname='USB Choice 2',
            description='Choice of USB debugging output number 2'),

            Config_Var(name='USB_CHOICE_3',id=0x0308,format='i16',
            default=3,min=0,max=22,
            longname='USB Choice 3',
            description='Choice of USB debugging output number 3'),

            Config_Var(name='USB_CHOICE_4',id=0x0309,format='i16',
            default=7,min=0,max=22,
            longname='USB Choice 4',
            description='Choice of USB debugging output number 4'),

            Config_Var(name='USB_CHOICE_5',id=0x030A,format='i16',
            default=11,min=0,max=22,
            longname='USB Choice 5',
            description='Choice of USB debugging output number 5'),

            Config_Var(name='USB_CHOICE_6',id=0x030B,format='i16',
            default=4,min=0,max=22,
            longname='USB Choice 6',
            description='Choice of USB debugging output number 6'),

            Config_Var(name='USB_CHOICE_7',id=0x030C,format='i16',
            default=5,min=0,max=22,
            longname='USB Choice 7',
            description='Choice of USB debugging output number 7'),

            Config_Var(name='USB_CHOICE_8',id=0x030D,format='i16',
            default=6,min=0,max=22,
            longname='USB Choice 8',
            description='Choice of USB debugging output number 8'),

            Config_Var(name='USB_CHOICE_9',id=0x030E,format='i16',
            default=9,min=0,max=22,
            longname='USB Choice 9',
            description='Choice of USB debugging output number 9'),

            Config_Var(name='USB_CHOICE_10',id=0x030F,format='i16',
            default=18,min=0,max=22,
            longname='USB Choice 10',
            description='Choice of USB debugging output number 10'),
	],
	'THRT':[

            Config_Var(name='MIN',id=0x0401,format='f32',
            default=0.9,min=0.0,max=10.0,
            longname='Throttle Minimum',
            description='Voltage on ADC when throttle is at minimum position'),

            Config_Var(name='MAX',id=0x0402,format='f32',
            default=4.0,min=0.0,max=10.0,
            longname='Throttle Maximum',
            description='Voltage on ADC when throttle is at maximum position'),

            Config_Var(name='HYST',id=0x0403,format='f32',
            default=0.025,min=0.0,max=1.0,
            longname='Throttle Hysteresis',
            description='Hysteresis when switching on or off in volts'),

            Config_Var(name='FILT',id=0x0404,format='f32',
            default=2.0,min=0.1,max=1000.0,
            longname='Throttle Low-pass Filter',
            description='Low-pass filter cutoff frequency in Hz'),

            Config_Var(name='RISE',id=0x0405,format='f32',
            default=0.0005,min=0.0001,max=0.1,
            longname='Throttle Rise Limit',
            description='Maximum amount of throttle percent rise per 1KHz count'),

            Config_Var(name='RATIO',id=0x0406,format='f32',
            default=1.499,min=1.0,max=1000.0,
            longname='Throttle Resistor Ratio',
            description='Resistor divider ratio for throttle analog input'),
	],
	'LMT':[

            Config_Var(name='VOLT_FAULT_MIN',id=0x0501,format='f32',
            default=44.8,min=1.0,max=250.0,
            longname='Minimum Battery Voltage',
            description='Fault when voltage below this'),

            Config_Var(name='VOLT_FAULT_MAX',id=0x0502,format='f32',
            default=70.4,min=1.0,max=250.0,
            longname='Maximum Battery Voltage',
            description='Fault when voltage above this'),

            Config_Var(name='CUR_FAULT_MAX',id=0x0503,format='f32',
            default=74.0,min=1.0,max=75.0,
            longname='Maximum Battery Current',
            description='Fault when current above this'),

            Config_Var(name='VOLT_SOFTCAP',id=0x0504,format='f32',
            default=10.0,min=1.0,max=250.0,
            longname='Limp-Home Low Voltage',
            description='Motor begins lowering power when voltage below this'),

            Config_Var(name='VOLT_HARDCAP',id=0x0505,format='f32',
            default=8.0,min=1.0,max=250.0,
            longname='Shutoff Low Voltage',
            description='Motor no longer functions when voltage below this'),

            Config_Var(name='PHASE_CUR_MAX',id=0x0506,format='f32',
            default=60.0,min=1.0,max=75.0,
            longname='Phase Current Maximum',
            description='Maximum current at motor phase'),

            Config_Var(name='PHASE_REGEN_MAX',id=0x0507,format='f32',
            default=5.0,min=1.0,max=75.0,
            longname='Phase Regen Maximum',
            description='Maximum reverse current at motor phase'),

            Config_Var(name='BATT_CUR_MAX',id=0x0508,format='f32',
            default=30.0,min=1.0,max=75.0,
            longname='Battery Max Current',
            description='Maximum current at batter'),

            Config_Var(name='BATT_REGEN_MAX',id=0x0509,format='f32',
            default=5.0,min=1.0,max=75.0,
            longname='Battery Regen Max',
            description='Maximum reverse current at battery'),

            Config_Var(name='FET_TEMP_SOFTCAP',id=0x050A,format='f32',
            default=75.0,min=20.0,max=150.0,
            longname='Controller Limp-Home Temperature',
            description='Motor power reduced when controller temp above this'),

            Config_Var(name='FET_TEMP_HARDCAP',id=0x050B,format='f32',
            default=90.0,min=20.0,max=150.0,
            longname='Controller Shutdown Temperature',
            description='Motor disabled when controller temp above this'),

            Config_Var(name='MOTOR_TEMP_SOFTCAP',id=0x050C,format='f32',
            default=75.0,min=20.0,max=150.0,
            longname='Motor Limp-Home Temperature',
            description='Motor power reduced when motor temp above this'),

            Config_Var(name='MOTOR_TEMP_HARDCAP',id=0x050D,format='f32',
            default=90.0,min=20.0,max=150.0,
            longname='Motor Shutdown Temperature',
            description='Motor disabled when motor temp above this'),
	],
	'MOTOR':[

            Config_Var(name='HALL1',id=0x0601,format='f32',
            default=0.72109,min=0.0,max=1.0,
            longname='Hall 1 Position',
            description='Angle when switching into hall state 1, forward rotation'),

            Config_Var(name='HALL2',id=0x0602,format='f32',
            default=0.0691474,min=0.0,max=1.0,
            longname='Hall 2 Position',
            description='Angle when switching into hall state 2, forward rotation'),

            Config_Var(name='HALL3',id=0x0603,format='f32',
            default=0.896364,min=0.0,max=1.0,
            longname='Hall 3 Position',
            description='Angle when switching into hall state 3, forward rotation'),

            Config_Var(name='HALL4',id=0x0604,format='f32',
            default=0.396317,min=0.0,max=1.0,
            longname='Hall 4 Position',
            description='Angle when switching into hall state 4, forward rotation'),

            Config_Var(name='HALL5',id=0x0605,format='f32',
            default=0.569599,min=0.0,max=1.0,
            longname='Hall 5 Position',
            description='Angle when switching into hall state 5, forward rotation'),

            Config_Var(name='HALL6',id=0x0606,format='f32',
            default=0.220085,min=0.0,max=1.0,
            longname='Hall 6 Position',
            description='Angle when switching into hall state 6, forward rotation'),

            Config_Var(name='POLEPAIRS',id=0x0607,format='i16',
            default=23,min=1,max=1000,
            longname='Motor Pole Pairs',
            description='Electrical to mechanical turns ratio'),

            Config_Var(name='GEAR_RATIO',id=0x0608,format='f32',
            default=1.0,min=1.0,max=1000.0,
            longname='Motor Gear Ratio',
            description='Motor to wheel gear ratio'),

            Config_Var(name='WHEEL_SIZE',id=0x0609,format='f32',
            default=700.28,min=100.0,max=2000.0,
            longname='Wheel Size',
            description='Wheel Diameter in mm'),
	],
}
