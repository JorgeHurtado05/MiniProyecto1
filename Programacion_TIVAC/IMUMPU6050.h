/*
 * IMUMPU6050.h
 *
 *  Created on: Mar 12, 2022
 *      Author: Jorge Hurtado
 *      Description: IMU MPU6050 Library
 *      Basado en Librerias extraidas de: https://github.com/mathmagson/mpu6050_tm4c123g
 */

#ifndef IMUMPU6050_H_
#define IMUMPU6050_H_


/*
 * Definimos variables necesarias para lectura IMU
 */
float faccel[3],fgyro[3];
/*
 * Instancias para I2C y MPU
 */
volatile bool DoneFlagMPU6050;

/*
 * Prototipos de funciones:
 */

void I2C0_INIT(void);
void I2CMSimpleIntHandler(void);
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status);
void MPU6050INIT(void);
void MPU6050READ (float *x, float *y, float *z);


#endif /* IMUMPU6050_H_ */
