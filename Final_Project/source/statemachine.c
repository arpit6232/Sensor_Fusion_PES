/*
 * statemachine.c
 *
 *  Created on: Nov 25, 2020
 *      Author: root
 */

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

#ifdef DEBUG
	#define MSG_DEBUG PRINTF
#else // non-debug mode - get rid of printing message
	#define MSG_DEBUG(...)
#endif


/* Structure for Handling */
struct mma_state_t{
	state_t state;
} mma_t  = {
	.state       = s_ROUTINE
};



/*!
* \def DATA_FUSE_MODE Set to the <code>1</code> to enable raw sensor data transmission or disable with <code>0</code> to enable data fusion
*/
#define DATA_FETCH_MODE 0

/*!
* \def DATA_FUSE_MODE Set to the opposite of {\ref DATA_FETCH_MODE} and enables sensor fusion
*/
#define DATA_FUSE_MODE (!DATA_FETCH_MODE)
static volatile uint8_t poll_mma8451q = 1;
volatile uint8_t flag;


/**
 * @brief Handler for interrupts on port A
 */
void PORTA_IRQHandler()
{

//	PORTA->PCR[14] |= PORT_PCR_ISF_MASK;
    register uint32_t isfr_mma = MMA8451Q_INT_PORT->ISFR;

	/* check MMA8451Q */
    register uint32_t fromMMA8451Q 	= (isfr_mma & ((1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN)));
		if (fromMMA8451Q) {
//		LED_RedOn();
		PORTA->PCR[14] |= PORT_PCR_ISF_MASK;
		uint8_t Int_SourceTrans = I2C_ReadRegister(MMA8451Q_I2CADDR, 0x1E);
		/* clear interrupts using BME decorated logical OR store */
		PORTA->ISFR |= (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN);
		//		BME_OR_W(&MMA8451Q_INT_PORT->ISFR, (1 << MMA8451Q_INT1_PIN) | (1 << MMA8451Q_INT2_PIN));
    }
		flag =1;
}


void state_machine(void) {

	float roll = 0.0,  pitch = 0.0;
	int PWM_Green=0, PWM_Blue = 0;
	int OldRange = 180, NewRange = 48000;
	state_t new_state = mma_t.state;
	mma8451q_acc_t acc;
	MMA8451Q_InitializeData(&acc);
	int readMMA;
	MSG_DEBUG("\n\r Initializing Inertial Sensor State Machine");

	while(1) {
		switch(new_state) {
		case s_ROUTINE:
			reset_timer();
			flag = 0;
			MSG_DEBUG("\n\r Inertial Sensor Acceleration within Bounds");

			while(flag != 1) {
				LED_RedOff();
				readMMA = 1;

				if (readMMA) {
					read_full_xyz(&acc);
					convert_xyz_to_roll_pitch(&acc, &roll, &pitch);
					PWM_Green = (((int)roll * NewRange ) / OldRange );
					PWM_Blue = (((int)pitch * NewRange ) / OldRange );
					if(PWM_Blue < 533) {
						PWM_Blue = PWM_Blue *-1;
					}
					if((int) pitch > 80) {
						GREEN_PWM = 0;
					}
					GREEN_PWM = PWM_Green;
					BLUE_PWM = PWM_Blue;

					PRINTF("\r\n roll: %d, pitch: %d", (int)roll, (int)pitch);
				}
			}
			if(flag == 1) {
				new_state = s_ACCEL;
			}
			break;


		case s_ACCEL:

			reset_timer();
			MSG_DEBUG("\n\r Accelerated too Fast");
			while(get_timer() < ACCEL_TIMEOUT) {
				read_full_xyz(&acc);
				convert_xyz_to_roll_pitch(&acc, &roll, &pitch);
				PWM_Green = (((int)roll * NewRange ) / OldRange );
				PWM_Blue = (((int)pitch * NewRange ) / OldRange );
				if(PWM_Blue < 533) {
					PWM_Blue = PWM_Blue *-1;
				}
				if((int) pitch > 80) {
					PWM_Green = 0;
				}
				GREEN_PWM = 0;
				BLUE_PWM = 0;
				delay_ms(200);
				GREEN_PWM = PWM_Green;
				BLUE_PWM = PWM_Blue;
				delay_ms(200);
			}
			flag = 0;
			new_state = s_ROUTINE;
		}
	}

}
