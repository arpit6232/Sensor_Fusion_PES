/*
 * led.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef LED_H_
#define LED_H_

#include "MKL25Z4.h"
#include "clock.h"


// basic light switch
#define LED1_POS (1)	// on port A
#define LED2_POS (2)	// on port A
#define SW1_POS (5)		// on port A
#define FULL_BRIGHTNESS 48000
#define RED_PWM 	TPM2->CONTROLS[0].CnV
#define GREEN_PWM 	TPM2->CONTROLS[1].CnV
#define BLUE_PWM 	TPM0->CONTROLS[1].CnV

#define MASK(x) (1UL << (x))

// Freedom KL25Z LEDs
#define RED_LED_POS (18)		// on port B
#define GREEN_LED_POS (19)	// on port B
#define BLUE_LED_POS (1)		// on port D

#define LED_RedOff()	TPM2->CONTROLS[0].CnV = 0;
#define LED_RedOn()		TPM2->CONTROLS[0].CnV = FULL_BRIGHTNESS;
#define LED_GreenOn()	TPM0->CONTROLS[1].CnV = 0;
#define LED_GreenOff()	TPM0->CONTROLS[1].CnV = FULL_BRIGHTNESS;
#define LED_BlueOn()	TPM2->CONTROLS[1].CnV = 0;
#define LED_BlueOff()	TPM2->CONTROLS[1].CnV = FULL_BRIGHTNESS;

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
