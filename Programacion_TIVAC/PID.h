/*
 * RUNMPU.h
 *
 *  Created on: Mar 12, 2022
 *      Author: Jorge Hurtado
 *      Description: PID LIBRARY, SPI_DAC Protocol.
 *      Basado en Librerias extraidas de: Sistemas de Control1 Proyecto Final.
 *
 */

#ifndef PID_H_
#define PID_H_

/*
 * Prototipo de funciones
 */
void Timer0IntHandler(void);
void SetupSPITimer0 (void);

/*
 * Variables
 */

volatile float v0, v1;
float Kp;
float Ki;
float Kd;


#endif /* PID_H_ */
