/*
 * project_parameters.h
 *
 *  Created on: Feb 8, 2017
 *      Author: David
 */

#ifndef PROJECT_PARAMETERS_H_
#define PROJECT_PARAMETERS_H_

// Motor parameters
#define MOTOR_POLEPAIRS		23

// Throttle setting
#define FULLSCALE_THROTTLE	(5.0f) // Amps

// Angle definitions - integer
// This set of defines are the integer values of angles
// as defined for a 16 bit unsigned integer.
#define U16_0_DEG		((uint16_t)0)
#define U16_30_DEG		((uint16_t)5461)
#define U16_60_DEG		((uint16_t)10923)
#define U16_90_DEG		((uint16_t)16384)
#define U16_120_DEG		((uint16_t)21845)
#define U16_150_DEG		((uint16_t)27307)
#define U16_180_DEG		((uint16_t)32768)
#define U16_210_DEG		((uint16_t)38229)
#define U16_240_DEG		((uint16_t)43691)
#define U16_270_DEG		((uint16_t)49152)
#define U16_300_DEG		((uint16_t)54613)
#define U16_330_DEG		((uint16_t)60075)

// Angle definitions - floating point
// This set of defines are the integer values of angles
// as defined for a single-precision float.
// Arbitrary choice of 6 significant figures.
#define F32_0_DEG		(0.0f)
#define F32_30_DEG		(0.0833333f)
#define F32_60_DEG		(0.166667f)
#define F32_90_DEG		(0.250000f)
#define F32_120_DEG		(0.333333f)
#define F32_150_DEG		(0.416667f)
#define F32_180_DEG		(0.500000f)
#define F32_210_DEG		(0.583333f)
#define F32_240_DEG		(0.666667f)
#define F32_270_DEG		(0.750000f)
#define F32_300_DEG		(0.833333f)
#define F32_330_DEG		(0.916667f)

// Interrupt priority settings
// Lowest number takes precedence
// Multiple interrupt sources can use the same priority level,
// but only a lower number interrupt will override a currently
// responding IRQ function.
#define PRIO_SYSTICK	3
#define PRIO_PWM		0
#define PRIO_HALL		1
#define PRIO_ADC		2
#define PRIO_APPTIMER	3
#define PRIO_HBD_UART	4
#define PRIO_PAS    5
#define PRIO_USB		6

// Hall state change table
// This state table corresponds to a forward rotating motor,
// with cable connections as follows:
// Hall A (PC6) -> NineContinent Green wire
// Hall B (PC7) -> NineContinent Blue wire
// Hall C (PC8) -> NineContinent Yellow wire
#define FORWARD_HALL_TABLE	{ 1, 5, 4, 6, 2, 3 }
// Same thing but reversed
#define REVERSE_HALL_TABLE	{ 3, 2, 6, 4, 5, 1 }
// Inverse lookup tables: Useful for determining rotation direction
// State is the table index, value is the previous state for that rotation direction
#define FORWARD_HALL_INVTABLE	{ 0, 3, 6, 2, 5, 1, 4, 0}
#define REVERSE_HALL_INVTABLE	{ 0, 5, 3, 1, 6, 4, 2, 0}

// Hall Angle correlations
// This table equates the 6 valid Hall states (1 - 6) with
// motor rotation angles. The Hall state is assumed to be
// equivalent to the angle that is halfway between state
// changes. Since the state changes are spaced every 60°
// around the motor, and the transitions start at zero,
// the halfway points are at 30°, 90°, 150°, 210°, 270°,
// and 330°.
// Angle associations per state are...
//			330->030: State 1
//			030->090: State 5
// 			090->150: State 4
//			150->210: State 6
//			210->270: State 2
//			270->330: State 3
#define HALL_ANGLES_INT		{	U16_0_DEG ,  /* State 0 - undefined */	\
								U16_180_DEG,   /* State 1 */		\
								U16_60_DEG, /* State 2 */		\
								U16_120_DEG, /* State 3 */ 		\
								U16_300_DEG, /* State 4 */		\
								U16_240_DEG,  /* State 5 */		\
								U16_0_DEG, /* State 6 */		\
								U16_0_DEG    /* State 7 - undefined */	\
							}
// And the same for floating point
#define HALL_ANGLES_FLOAT	{	F32_0_DEG,		\
								F32_180_DEG,	\
								F32_60_DEG,		\
								F32_120_DEG,	\
								F32_300_DEG,	\
								F32_240_DEG,	\
								F32_0_DEG,		\
								F32_0_DEG		\
							}

#define HALL_ANGLES_TO_DRIVE_FLOAT       {     F32_0_DEG,              \
                                      F32_270_DEG,    \
                                      F32_150_DEG,             \
                                      F32_210_DEG,    \
                                      F32_30_DEG,    \
                                      F32_330_DEG,    \
                                      F32_90_DEG,              \
                                      F32_0_DEG               \
                                  }

#endif /* PROJECT_PARAMETERS_H_ */
