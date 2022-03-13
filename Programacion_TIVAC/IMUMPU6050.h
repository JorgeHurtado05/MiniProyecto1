/*
 * IMUMPU6050.h
 *
 *  Created on: Mar 12, 2022
 *      Author: Jorge Hurtado
 *      Description: IMU MPU6050 Library
 */

#ifndef IMUMPU6050_H_
#define IMUMPU6050_H_


/*
 * Definimos variables necesarias para lectura IMU
 */
float faccel[3],fgyro[3];
tMPU6050 sMPU6050;
/*
 * Instancias para I2C y MPU
 */
volatile bool g_bIMUMPU6050Done;
tI2CMInstance g_sI2CMSimpleInst;

#endif /* IMUMPU6050_H_ */
