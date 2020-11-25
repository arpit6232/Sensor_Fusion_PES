/*
 * led.c
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#include "led.h"
#include "MKL25Z4.h"
#include "delay.h"
#include "clock.h"

void Init_RGB_LEDs(void) {
	// Enable clock to ports B and D
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;;

	// Make 3 pins GPIO
//	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;
//	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(1);
//	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;
//	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(1);
//	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;
//	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(1);


	PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(3);
	PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(3);
	PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(4);

	// Set ports to outputs
	PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
	PTD->PDDR |= MASK(BLUE_LED_POS);

	TPM2->CONTROLS[0].CnV = 0;
	TPM2->CONTROLS[1].CnV = 0;
	TPM0->CONTROLS[1].CnV = 0;
}

void Control_RGB_LEDs(unsigned int red_on, unsigned int green_on, unsigned int blue_on) {
	if (red_on) {
			PTB->PCOR = MASK(RED_LED_POS);
	} else {
			PTB->PSOR = MASK(RED_LED_POS);
	}
	if (green_on) {
			PTB->PCOR = MASK(GREEN_LED_POS);
	}	else {
			PTB->PSOR = MASK(GREEN_LED_POS);
	}
	if (blue_on) {
			PTD->PCOR = MASK(BLUE_LED_POS);
	}	else {
			PTD->PSOR = MASK(BLUE_LED_POS);
	}
}

void LED_Red()
{
	LED_RedOn();
	LED_GreenOff();
	LED_BlueOff();
}

void LED_Green()
{
	LED_RedOff();
	LED_GreenOn();
	LED_BlueOff();
}

void LED_Blue()
{
	LED_RedOff();
	LED_GreenOn();
	LED_BlueOff();
}

void LED_Yellow()
{
	LED_RedOn();
	LED_GreenOn();
	LED_BlueOff();
}

void LED_Magenta()
{
	LED_RedOn();
	LED_GreenOff();
	LED_BlueOn();
}

void LED_Cyan()
{
	LED_RedOff();
	LED_GreenOn();
	LED_BlueOn();
}

void LED_White()
{
	LED_RedOn();
	LED_GreenOn();
	LED_BlueOn();
}

void LED_Off()
{
	LED_RedOff();
	LED_GreenOff();
	LED_BlueOff();
}

void LED_Init() {
		// Enable clock to ports B and D
		SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTD_MASK;;

		// Make 3 pins GPIO
		PORTB->PCR[RED_LED_POS] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[RED_LED_POS] |= PORT_PCR_MUX(3);
		PORTB->PCR[GREEN_LED_POS] &= ~PORT_PCR_MUX_MASK;
		PORTB->PCR[GREEN_LED_POS] |= PORT_PCR_MUX(3);
		PORTD->PCR[BLUE_LED_POS] &= ~PORT_PCR_MUX_MASK;
		PORTD->PCR[BLUE_LED_POS] |= PORT_PCR_MUX(4);

		// Set ports to outputs
		PTB->PDDR |= MASK(RED_LED_POS) | MASK(GREEN_LED_POS);
		PTD->PDDR |= MASK(BLUE_LED_POS);

		TPM2->CONTROLS[0].CnV = 0;
		TPM2->CONTROLS[1].CnV = 0;
		TPM0->CONTROLS[1].CnV = 0;
}

void TrafficLight()
{
	LED_Off();
	LED_Off();
	LED_Off();
	LED_Off();
}

void DoubleFlash()
{
	LED_White();
	delay_ms(50);
	TrafficLight();
	delay_ms(50);
	LED_White();
	delay_ms(50);
	TrafficLight();
}

