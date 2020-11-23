/*
 * systick.c
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */


#include "MKL25Z4.h"
#include "sysclock.h"
#include "systick.h"

/**
 * @brief The system tick counter
 */
volatile uint32_t SystemMilliseconds = 0;

/**
 * @brief 250µs Counter
 */
static uint32_t freeRunner = 0;


void InitSysTick() {
	SysTick->LOAD  = (uint32_t)(SYSTICK_TMR_RELOAD_VAL);              /* set reload register */
	  NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
	  SysTick->VAL   = 0UL;                                             /* Load the SysTick Counter Value */
	  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |    /* use processor clock instead of external clock */
	                   SysTick_CTRL_TICKINT_Msk   |		/* enable interrupt if timer reaches zero */
	                   SysTick_CTRL_ENABLE_Msk;			/* enable the systick timer */
}


void SysTick_Handler()
{
	SystemMilliseconds += ((++freeRunner) & 0b100) >> 2;
	freeRunner &= 0b11;
}
