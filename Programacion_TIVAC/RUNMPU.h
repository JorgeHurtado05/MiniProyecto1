/*
 * RUNMPU.h
 *
 *  Created on: Mar 12, 2022
 *      Author: Jorge Hurtado
 *      Description: IMU MPU6050 library (Reads via I2C)
 *      Basado en Librerias extraidas de: https://github.com/mathmagson/mpu6050_tm4c123g
 *
 */

#ifndef RUNMPU_H_
#define RUNMPU_H_

/*
 * Prototipo de funciones
 */
void InitI2C0(void);
void ConfigureUART(void);
void delayMS(int ms);
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status);
void I2CMSimpleIntHandler(void);
void MPU6050Example(void);
void uart2_init(void);

/*
 * Variables
 */
float fAccel[3], fGyro[3];
//tMPU6050 sMPU6050;
float x, y, z;

//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
//tI2CMInstance g_sI2CMSimpleInst;

#endif /* RUNMPU_H_ */
