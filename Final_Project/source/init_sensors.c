/*
 * init_sensors.c
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef INIT_SENSORS_C_
#define INIT_SENSORS_C_

#include "i2c.h"
#include "mma8451q.h"
#include "init_sensors.h"
#include "assert.h"
#include "MKL25Z4.h"
#include "stdio.h"
#include "fsl_debug_console.h"
#include "led.h"
#include "i2carbiter.h"

#define MASK(x)		(1UL << (x))


static union {
//#if ENABLE_MMA8451Q
    mma8451q_confreg_t mma8451q_configuration;
//#endif
//    mpu6050_confreg_t mpu6050_configuration;
//    hmc5883l_confreg_t hmc5883l_configuration;
} config_buffer;


void InitMMA8451Q()
{
#if ENABLE_MMA8451Q
	mma8451q_confreg_t *configuration = &config_buffer.mma8451q_configuration;

    PRINTF("MMA8451Q: initializing ...\r\n");

    /* configure interrupts for accelerometer */
    /* INT1_ACCEL is on PTA14, INT2_ACCEL is on PTA15 */
    SIM->SCGC5 |= (1 << SIM_SCGC5_PORTA_SHIFT) & SIM_SCGC5_PORTA_MASK; /* power to the masses */
    MMA8451Q_INT_PORT->PCR[MMA8451Q_INT1_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
    MMA8451Q_INT_PORT->PCR[MMA8451Q_INT2_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0b1010) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
    MMA8451Q_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1 << MMA8451Q_INT1_PIN) | GPIO_PDDR_PDD(1 << MMA8451Q_INT2_PIN));

	//    /* prepare interrupts for pin change / PORTA */
	NVIC_SetPriority(PORTA_IRQn, 2); // 0, 1, 2, or 3
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);

    /* switch to the correct port */
    I2CArbiter_Select(MMA8451Q_I2CADDR);

    /* perform identity check */
    uint8_t id = MMA8451Q_WhoAmI();
    assert(id = 0x1A);
    PRINTF("MMA8451Q: device found.\r\n");

    /* configure accelerometer */
    MMA8451Q_EnterPassiveMode();
    MMA8451Q_Reset();
    delay_ms(20);

    /* TODO: Initiate self-test */

    /* read configuration and modify */
    MMA8451Q_FetchConfiguration(configuration);

    MMA8451Q_SetSensitivity(configuration, MMA8451Q_SENSITIVITY_2G, MMA8451Q_HPO_DISABLED);
    MMA8451Q_SetDataRate(configuration, MMA8451Q_DATARATE_100Hz, MMA8451Q_LOWNOISE_ENABLED);
    MMA8451Q_SetOversampling(configuration, MMA8451Q_OVERSAMPLING_HIGHRESOLUTION);
    MMA8451Q_ClearInterruptConfiguration(configuration);
    MMA8451Q_SetInterruptMode(configuration, MMA8451Q_INTMODE_OPENDRAIN, MMA8451Q_INTPOL_ACTIVELOW);
    MMA8451Q_ConfigureInterrupt(configuration, MMA8451Q_INT_DRDY, MMA8451Q_INTPIN_INT2);

    MMA8451Q_StoreConfiguration(configuration);
    MMA8451Q_EnterActiveMode();

    PRINTF("MMA8451Q: configuration done.\r\n");
#endif
}

//
//int init_mma()
//{
//	//set active mode, 14 bit samples and 800 Hz ODR
//	i2c_write_byte(MMA_ADDR, REG_CTRL1, 0x01);
//	MMA8451Q_INT_PORT->PCR[MMA8451Q_INT1_PIN] = PORT_PCR_MUX(0x1) | PORT_PCR_IRQC(0xA) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* interrupt on falling edge, pull-up for open drain/active low line */
////	MMA8451Q_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1 << MMA8451Q_INT1_PIN) | GPIO_PDDR_PDD(1 << MMA8451Q_INT2_PIN));
//	MMA8451Q_INT_GPIO->PDDR &= ~(GPIO_PDDR_PDD(1 << MMA8451Q_INT1_PIN));
//
//
//	//    MMA8451Q_REG_CTRL_REG5
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, 0x00);
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG3, 0x00);             // Push-pull, active low interrupt
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG4, 0x01);             // Enable DRDY interrupt
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG5, 0x01);             // DRDY interrupt routed to INT1 -
//
////    MMA8451Q_StoreConfiguration(configuration);
////    MMA8451Q_EnterActiveMode();
//
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG1, 0x00);
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG3, 0x00);             // Push-pull, active low interrupt
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG4, 0x01);             // Enable DRDY interrupt
//	I2C_WriteRegister(MMA8451Q_I2CADDR, MMA8451Q_REG_CTRL_REG5, 0x01);             // DRDY interrupt routed to INT1 -
//
//
//	//    /* prepare interrupts for pin change / PORTA */
//	NVIC_SetPriority(PORTA_IRQn, 2); // 0, 1, 2, or 3
//	NVIC_ClearPendingIRQ(PORTA_IRQn);
//	NVIC_EnableIRQ(PORTA_IRQn);
//	return 1;
//}
//


#endif /* INIT_SENSORS_C_ */
