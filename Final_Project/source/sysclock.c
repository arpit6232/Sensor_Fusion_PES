/*
 * sysclock.c
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#include "MKL25Z4.h"
#include "clock.h"
#include "sysclock.h"


void sysclock_init()
{
	// MCG_C1: CLKS (bit 7-6) = 00
	MCG->C1 |= MCG_C1_CLKS(0);  // Select PLL/FLL as clock source

	// MCG_C1: IREFS (bit 2)  = 1
	MCG->C1 |= MCG_C1_IREFS(1); // Select Inernal Reference clock
								// source for FLL

	// MCG_C4: DRST_DRS (bit 6-5) = 01
	MCG->C4 |= MCG_C4_DRST_DRS(1); // Select DCO range as Mid range
								   // DCO generates the FLL Clock

	// MCG_C4: DMX32 (bit 7) = 1
	MCG->C4 |= MCG_C4_DMX32(1);    // Select DCO frequency as 48Mhz
}
