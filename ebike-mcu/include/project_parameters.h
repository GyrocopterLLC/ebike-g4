/*
 * project_parameters.h
 *
 *  Created on: Feb 8, 2017
 *      Author: David
 */

#ifndef PROJECT_PARAMETERS_H_
#define PROJECT_PARAMETERS_H_


// Interrupt priority settings
// Lowest number takes precedence
// Multiple interrupt sources can use the same priority level,
// but only a lower number interrupt will override a currently
// responding IRQ function.
#define PRIO_PWM		0
#define PRIO_HALL		1
#define PRIO_ADC		2
#define PRIO_APPTIMER	3
#define PRIO_HBD_UART	4
#define PRIO_USB		6

// Hall state change table
// This state table corresponds to a forward rotating motor,
// with cable connections as follows:
// Hall A (PC6) -> NineContinent Green wire
// Hall B (PC7) -> NineContinent Blue wire
// Hall C (PC8) -> NineContinent Yellow wire
#define FORWARD_HALL_TABLE	{ 0, 1, 5, 6, 3, 2, 4, 0}

#endif /* PROJECT_PARAMETERS_H_ */
