/*
 * clock.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef CLOCK_H_
#define CLOCK_H_

// Shamelessly from https://www.youtube.com/watch?v=uiSTB4jkxhw

#define XTAL_FREQ			(8000000u) 	/* Hz, FRDM-KL25Z has an on-board 8 MHz xtal */
#define XTAL_PEE_DIVIDE		(4u)		/* divide by 4 (8 MHz --> 2 MHz) */
#define XTAL_PEE_UPSCALE	(24u)		/* scale up by 24 (2 MHz --> 48 MHz) */

#define CORE_CLOCK			(XTAL_FREQ/XTAL_PEE_DIVIDE*XTAL_PEE_UPSCALE) /* Hz */

void InitClock();

#endif /* CLOCK_H_ */
