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

void InitTPM() {

	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK | SIM_SCGC6_TPM2_MASK;

	//set clock source for tpm: 48 MHz
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

	//load the counter and mod
	TPM0->MOD = PWM_PERIOD-1;
	TPM2->MOD = PWM_PERIOD-1;

	// Prescalar set to 1, no division
	TPM0->SC |= (TPM_SC_CPWMS(0)| TPM_SC_CMOD(1));
	TPM2->SC |= (TPM_SC_CPWMS(0)| TPM_SC_CMOD(1));

//	// Contiunue Operation in Debug Mode
//	TPM0->CONF |= TPM_CONF_DBGMODE(3);
//	TPM2->CONF |= TPM_CONF_DBGMODE(3);

//	// Set channel 1 to edge-aligned low-true PWM
//	// Channel Based Setup to Edge-alligned active-low PWM
//	TPM2->CONTROLS[0].CnSC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_ELSA(0) | TPM_CnSC_MSB(1)  | TPM_CnSC_MSA(0));
//	TPM2->CONTROLS[1].CnSC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_ELSA(0) | TPM_CnSC_MSB(1)  | TPM_CnSC_MSA(0));
//	TPM0->CONTROLS[1].CnSC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_ELSA(0) | TPM_CnSC_MSB(1)  | TPM_CnSC_MSA(0));

	TPM0->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->CONTROLS[1].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	TPM2->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;

	// Clock prescaler is 7 (PWM clock devided by 128)
	// This makes PWM clock as 48000000/128 = 375000 Hz (375Khz)
	TPM0->SC |= TPM_SC_PS(7);
	TPM2->SC |= TPM_SC_PS(7);


//	// Clock prescaler is 7 (PWM clock devided by 128)
//	// This makes PWM clock as 48000000/128 = 375000 Hz (375Khz)
//	TPM0->SC |= TPM_SC_PS(1);
//	TPM2->SC |= TPM_SC_PS(1);

	// Setting Initial Duty cycle to 0
	TPM2->CONTROLS[0].CnV = 0;
	TPM2->CONTROLS[1].CnV = 0;
	TPM0->CONTROLS[1].CnV = 0;

}

void TPM0_IRQHandler() {
	TPM0->SC |= TPM_SC_TOIE_MASK; // reset overflow flag
}

void TPM2_IRQHandler() {
	TPM2->SC |= TPM_SC_TOIE_MASK; // reset overflow flag
}
