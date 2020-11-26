/*
 * statemachine.h
 *
 *  Created on: Nov 25, 2020
 *      Author: root
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "MKL25Z4.h"
#include "fsl_debug_console.h"
#include "stdint.h"
#include "stdbool.h"

extern volatile uint8_t flag;

typedef enum {

	s_ROUTINE,
	s_ACCEL
} state_t;

#define ACCEL_TIMEOUT 10000


void state_machine(void);




#endif /* STATEMACHINE_H_ */
