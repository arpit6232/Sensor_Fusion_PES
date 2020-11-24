/*
 * init_sensors.h
 *
 *  Created on: Nov 23, 2020
 *      Author: root
 */

#ifndef INIT_SENSORS_H_
#define INIT_SENSORS_H_

#ifndef INIT_SENSORS_H
#define INIT_SENSORS_H

#define ENABLE_MMA8451Q 1						/*! Used to enable or disable MMA8451Q fetching */

#define MMA8451Q_INT_PORT	PORTA				/*! Port at which the MMA8451Q INT1 and INT2 pins are attached */
#define MMA8451Q_INT_GPIO	GPIOA				/*! Port at which the MMA8451Q INT1 and INT2 pins are attached */
#define MMA8451Q_INT1_PIN	14					/*! Pin at which the MMA8451Q INT1 is attached */
#define MMA8451Q_INT2_PIN	15					/*! Pin at which the MMA8451Q INT2 is attached */

//#define MPU6050_INT_PORT	PORTA				/*! Port at which the MPU6050 INT pin is attached */
//#define MPU6050_INT_GPIO	GPIOA				/*! Port at which the MPU6050 INT pin is attached */
//#define MPU6050_INT_PIN		13					/*! Pin at which the MPU6050 INT is attached */

//#include "fixmath.h"

/**
* @brief Sets up the MMA8451Q communication
*/
void InitMMA8451Q();

///**
//* @brief Sets up the MPU6050 communication
//*/
//void InitMPU6050();
//
/**
* @brief Sets up the HMC5883L communication
*/
void InitHMC5883L();
//
///**
//* @brief Gets the scaling value for the MPU6050 accelerometer
//*/
//fix16_t mpu6050_accelerometer_get_scaler();
//
///**
//* @brief Gets the scaling value for the MPU6050 gyroscope
//*/
//fix16_t mpu6050_gyroscope_get_scaler();
//
///**
//* @brief Gets the scaling value for the HMC5883L magnetometer
//*/
//fix16_t hmc5883l_magnetometer_get_scaler();

#endif

#endif /* INIT_SENSORS_H_ */
