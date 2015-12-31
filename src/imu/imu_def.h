/*
 * invensense.h
 *
 *  Created on: Oct 2, 2015
 *      Author: jcobb
 */

#ifndef INVENSENSE_H_
#define INVENSENSE_H_


#define IMU_INV_MPU6050
//#define IMU_INV_MPU9150

#ifdef IMU_INV_MPU6050
#include "invensense/mpu6050.h"
#else if IMU_INV_MPU9150
#include "invensense/mpu9150.h"
#endif



#endif /* INVENSENSE_H_ */
