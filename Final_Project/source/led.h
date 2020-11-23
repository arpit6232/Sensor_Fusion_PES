/*
 * led.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef LED_H_
#define LED_H_

#include "MKL25Z4.h"


// basic light switch
#define LED1_POS (1)	// on port A
#define LED2_POS (2)	// on port A
#define SW1_POS (5)		// on port A

#define MASK(x) (1UL << (x))

// Freedom KL25Z LEDs
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)	// on port B
#define BLUE_LED_POS (1)		// on port D

#define LED_RedOn()		PTB->PCOR = MASK(RED_LED_POS);
#define LED_RedOff()	PTB->PSOR = MASK(RED_LED_POS);
#define LED_GreenOn()	PTB->PCOR = MASK(GREEN_LED_POS);
#define LED_GreenOff()	PTB->PSOR = MASK(GREEN_LED_POS);
#define LED_BlueOn()	PTD->PCOR = MASK(BLUE_LED_POS);
#define LED_BlueOff()	PTD->PSOR = MASK(BLUE_LED_POS);

//// function prototypes
void Init_RGB_LEDs(void);
void Control_RGB_LEDs(unsigned int red_on, unsigned int green_on, unsigned int blue_on);
void Toggle_RGB_LEDs(unsigned int red, unsigned int green, unsigned int blue);

void LED_Init();
void LED_Red();
void LED_Green();
void LED_Blue();
void LED_Yellow();
void LED_Magenta();
void LED_Cyan();
void LED_White();
void LED_Off();
void TrafficLight();
void DoubleFlash();


#endif /* LED_H_ */
