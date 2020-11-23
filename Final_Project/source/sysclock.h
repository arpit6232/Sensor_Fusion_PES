/*
 * sysclock.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef SYSCLOCK_H_
#define SYSCLOCK_H_


#define SYSCLOCK_FREQUENCY (48000000U)

/*
 * Initializes the system clock. You should call this first in your
 * program.
 */
void sysclock_init();

#endif /* SYSCLOCK_H_ */
