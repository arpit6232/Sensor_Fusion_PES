/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    Final_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

#include "bme.h"
#include "assert.h"
#include "clock.h"
#include "systick.h"
#include "systick.h"
#include "delay.h"
#include "buffer.h"
#include "math.h"

#include "i2c.h"
#include "i2carbiter.h"
#include "led.h"

#include "init_sensors.h"
#include "mma8451q.h"
#include "statemachine.h"


#define I2CARBITER_COUNT 	(1)					/*< Number of I2C devices we're talking to */
i2carbiter_entry_t i2carbiter_entries[I2CARBITER_COUNT]; /*< Structure for the pin enabling/disabling manager */

//
///**
// * @brief Handler for interrupts on port A
// */
//void PORTA_IRQHandler()
//{
//
////	PORTA->PCR[14] |= PORT_PCR_ISF_MASK;
//    register uint32_t isfr_mma = MMA8451Q_INT_PORT->ISFR;
//
//	/* check MMA8451Q */
//    register uint32_t fromMMA8451Q 	= (isfr_mma & ((1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN)));
//		if (fromMMA8451Q) {
//		LED_RedOn();
//		PORTA->PCR[14] |= PORT_PCR_ISF_MASK;
//		uint8_t Int_SourceTrans = I2C_ReadRegister(MMA8451Q_I2CADDR, 0x1E);
//		/* clear interrupts using BME decorated logical OR store */
//		PORTA->ISFR |= (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN);
//		//		BME_OR_W(&MMA8451Q_INT_PORT->ISFR, (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN));
//    }
//}
//


/************************************************************************/
/* I2C arbiter configuration                                            */
/************************************************************************/

void InitI2CArbiter()
{
    /* prior to configuring the I2C arbiter, enable the clocks required for
    * the used pins
    */
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTE_MASK;

    /* configure I2C arbiter
    * The arbiter takes care of pin selection
    */
    I2CArbiter_PrepareEntry(&i2carbiter_entries[0], MMA8451Q_I2CADDR, 24, 5, 25, 5);
    I2CArbiter_Configure(i2carbiter_entries, I2CARBITER_COUNT);
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Hello World\r\n");
//    float roll = 0.0,  pitch = 0.0;
//    int PWM_Green=0, PWM_Blue = 0;
//    int OldRange = 180, NewRange = 48000;

    /* initialize the core clock and the systick timer */
//	InitClock();
	InitSysTick();
	InitTPM();

	/* initialize the RGB led */
	LED_Init();

	/* Began the Code */
	DoubleFlash();

	/* initialize the I2C bus */
	I2C_Init();
	i2c_init();

//	/* initialize I2C arbiter */
	InitI2CArbiter();

	/* initialize the Sensor */
	InitMMA8451Q();
//	assert(init_mma());

	/* Began the Code */
	TrafficLight();

//	mma8451q_acc_t acc;
//	MMA8451Q_InitializeData(&acc);
//	int readMMA;
//	while(1) {
//		LED_RedOff();
//		readMMA = 1;
//
//		if (readMMA)
//		{
//			read_full_xyz(&acc);
//			convert_xyz_to_roll_pitch(&acc, &roll, &pitch);
//			PWM_Green = (((int)roll * NewRange ) / OldRange );
//			PWM_Blue = (((int)pitch * NewRange ) / OldRange );
//			if(PWM_Blue < 533) {
//				PWM_Blue = PWM_Blue *-1;
//			}
//			if((int) pitch > 80) {
//				GREEN_PWM = 0;
//			}
//			GREEN_PWM = PWM_Green;
//			BLUE_PWM = PWM_Blue;
//
//			PRINTF("\r\n roll: %d, pitch: %d", (int)roll, (int)pitch);
//		}
//	}

	state_machine();

    return 0 ;
}
