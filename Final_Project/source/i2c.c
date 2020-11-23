/*
 * i2c.c
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#include "i2c.h"
#include "delay.h"
#include "assert.h"


/**
 * @brief Initialises the I2C interface
 */
void I2C_Init()
{
	/* enable clock gating to I2C0 */
#if !I2C_USE_BME
	SIM->SCGC4 |= (1 << SIM_SCGC4_I2C0_SHIFT) & SIM_SCGC4_I2C0_MASK;
#else
	BME_OR_W(&SIM->SCGC4, (1 << SIM_SCGC4_I2C0_SHIFT) & SIM_SCGC4_I2C0_MASK);
#endif

#if 1 /* in ancient times this was hardcoded */

	/* enable the clock gate to port E */
#if !I2C_USE_BME
	SIM->SCGC5 |= (1 << SIM_SCGC5_PORTE_SHIFT) & SIM_SCGC5_PORTE_MASK;
#else
	BME_OR_W(&SIM->SCGC5, (1 << SIM_SCGC5_PORTE_SHIFT) & SIM_SCGC5_PORTE_MASK);
#endif

	/* configure port E pins to I2C operation for MMA8451Q */
	PORTE->PCR[24] = PORT_PCR_MUX(5); /* SCL */
	PORTE->PCR[25] = PORT_PCR_MUX(5); /* SDA */

#endif

	/* configure the I2C clock */
	/*
	 * I2C0 is clocked by the bus clock, that is core/2.
	 * For the MMA8451Q inertial sensor on the FRDM-25KLZ board the
	 * maximum SCL frequency is 400 kHz.
	 * Assuming PEE mode with core=48MHz, 400 kHz = 48MHz/2 / 60,
	 * which means a prescaler (SCL divider) of 60.
	 * According to table 38-41, I2C divider and hold values,
	 * the closest SCL diver is 64 (375 kHz SCL), which is ICR value 0x12.
	 * Alternatively, the SCL divider of 30 can be used (ICR=0x05) in combination
	 * with a multiplicator of 2 (MULT=0x01).
	 * A note states that ICR values lower than 0x10 might result in a varying
	 * SCL divider (+/- 4). However the data sheet does not state anything
	 * useful about that.
	 */
	I2C0->F = I2C_F_MULT(0x00) | I2C_F_ICR(0x12); /* divide by 64 instead, so 375 kHz */

	/* enable the I2C module */
	I2C0->C1 = (1 << I2C_C1_IICEN_SHIFT) & I2C_C1_IICEN_MASK;
}

/**
 * @brief Reads an 8-bit register from an I2C slave
 */
uint8_t I2C_ReadRegister(register uint8_t slaveId, register uint8_t registerAddress)
{
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();

	/* send I2C start signal and set write direction, also enables ACK */
	I2C_SendStart();

	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));

	/* send the register address */
	I2C_SendBlocking(registerAddress);

	/* signal a repeated start condition */
	I2C_SendRepeatedStart();

	/* send the read address */
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));

	/* switch to receive mode but disable ACK because only one data byte will be read */
	I2C_EnterReceiveModeWithoutAck();

	/* read a dummy byte to drive the clock */
	I2C_ReceiverModeDriveClock();

	/* stop signal */
	I2C_SendStop();

	/* fetch the last received byte */
	register uint8_t result = I2C0->D;
	return result;
}

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be greater than or equal to two.
 * @param[out] buffere The buffer to write into
 */
static void I2C_ReadRegistersInternal(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, uint8_t *const buffer)
{
	assert(registerCount >= 2);

	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();

	/* send I2C start signal and set write direction, also enables ACK */
	I2C_SendStart();

	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));

	/* send the register address */
	I2C_SendBlocking(startRegisterAddress);

	/* signal a repeated start condition */
	I2C_SendRepeatedStart();

	/* send the read address */
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));

	/* switch to receive mode and assume more than one register */
	I2C_EnterReceiveModeWithAck();

	/* read a dummy byte to drive the clock */
	I2C_ReceiverModeDriveClock();

	/* for all remaining bytes, read */
	--registerCount;
	uint8_t index = 0;
	while (--registerCount > 0)
	{
		/* fetch and store value */
		register uint8_t value = I2C0->D;
		buffer[index++] = value;

		/* wait for completion */
		I2C_Wait();
	}

	/* disable ACK and read second-to-last byte */
	I2C_DisableAck();

	/* fetch and store value */
	buffer[index++] = I2C0->D;

	/* wait for completion */
	I2C_Wait();

	/* stop signal */
	I2C_SendStop();

	/* fetch the last received byte */
	buffer[index++] = I2C0->D;
}

/**
 * @brief Reads multiple 8-bit registers from an I2C slave
 * @param[in] slaveId The slave device ID
 * @param[in] startRegisterAddress The first register address
 * @param[in] registerCount The number of registers to read; Must be larger than zero.
 * @param[out] buffer The buffer to write into
 */
void I2C_ReadRegisters(register uint8_t slaveId, register uint8_t startRegisterAddress, register uint8_t registerCount, register uint8_t *buffer)
{
	assert(registerCount > 0);

	if (registerCount >= 2)
	{
		I2C_ReadRegistersInternal(slaveId, startRegisterAddress, registerCount, buffer);
	}
	else
	{
		assert(1 == registerCount);
		register uint8_t result = I2C_ReadRegister(slaveId, startRegisterAddress);
		buffer[0] = result;
	}
}

/**
 * @brief Reads an 8-bit register from an I2C slave
 */
void I2C_WriteRegister(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t value)
{
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();

	/* send I2C start signal and set write direction*/
	I2C_SendStart();

	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));

	/* send the register address */
	I2C_SendBlocking(registerAddress);

	/* send the register address */
	I2C_SendBlocking(value);

	/* issue stop signal by clearing master mode. */
	I2C_SendStop();
}

/**
 * @brief Reads an 8-bit register from an I2C slave, modifies it by FIRST and-ing with {@see andMask} and THEN or-ing with {@see orMask} and writes it back
 * @param[in] slaveId The slave id
 * @param[in] registerAddress The register to modify
 * @param[in] orMask The mask to OR the register with
 * @param[in] andMask The mask to AND the register with
 * @return The register after modification
 */
uint8_t I2C_ModifyRegister(register uint8_t slaveId, register uint8_t registerAddress, register uint8_t andMask, register uint8_t orMask)
{
	/* loop while the bus is still busy */
	I2C_WaitWhileBusy();

	/* send the slave address and register */
	I2C_SendStart();
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	I2C_SendBlocking(registerAddress);

	/* signal a repeated start condition */
	I2C_SendRepeatedStart();
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));

	/* switch to receive mode but disable ACK because only one data byte will be read */
	I2C_EnterReceiveModeWithoutAck();
	I2C_ReceiverModeDriveClock();

	/* instead of a stop signal, send repeated start again */
	I2C_SendRepeatedStart();

	/* fetch the last received byte */
	register uint8_t value = I2C0->D;

	/* modify the register */
	value &= andMask;
	value |= orMask;

	/* send the slave address and wait for the I2C bus operation to complete */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));

	/* send the register address */
	I2C_SendBlocking(registerAddress);

	/* send the register address */
	I2C_SendBlocking(value);

	/* issue stop signal by clearing master mode. */
	I2C_SendStop();
	return value;
}

/**
 * @brief Resets the bus by toggling master mode if the bus is busy. This will interrupt ongoing traffic, so use with caution.
 */
void I2C_ResetBus()
{
	BME_OR_B(&I2C0->S, ((1 << I2C_S_IICIF_SHIFT) << I2C_S_IICIF_MASK));
	BME_OR_B(&I2C0->S, ((1 << I2C_S_IICIF_SHIFT) << I2C_S_IICIF_MASK)); /* clear interrupt flag */
}

/**
 * @brief Initiates a register read after the module was brought into TX mode.
 * @param[in] slaveId The slave id
 * @param[in] registerAddress the register to read from
 */
void I2C_InitiateRegisterReadAt(const register uint8_t slaveId, const register uint8_t registerAddress)
{
	/* send register id */
	I2C_SendBlocking(I2C_WRITE_ADDRESS(slaveId));
	I2C_SendBlocking(registerAddress);

	/* enter read mode */
	I2C_SendRepeatedStart();
	I2C_SendBlocking(I2C_READ_ADDRESS(slaveId));
	I2C_EnterReceiveModeWithAck();
	I2C_ReceiverModeDriveClock();
}



