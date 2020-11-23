/*
 * systick.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

/**
* @brief Defines for the system tick behaviour
*/

#define SYSTICK_FREQUENCY		(4000u) /* Hz */

#define SYSTEM_CLOCK_FREQ      48000000UL  // 48 Mhz
#define SYTICK_TIME_FREQ       (4000U) // 1000 Khz
#define SYSTICK_TMR_RELOAD_VAL ((SYSTEM_CLOCK_FREQ / SYTICK_TIME_FREQ) - 1UL) // 48000 - 1

/**
* @brief Function to initialize the SysTick interrupt
*/
void InitSysTick();


#endif /* SYSTICK_H_ */
