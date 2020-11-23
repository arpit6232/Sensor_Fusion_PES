/*
 * clock.c
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#include "stdint.h"
#include "clock.h"
#include "sysclock.h"
#include "MKL25Z4.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"


void InitClock() {
	const int xtal = XTAL_FREQ;
	const int8_t divider = XTAL_PEE_DIVIDE;
	const int8_t multiplier = XTAL_PEE_UPSCALE;
	sysclock_init(); // 48Mhz
}
