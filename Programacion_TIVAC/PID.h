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
void PID_RESTADOR (void);


/*
 * Declaracion Set Functions
 */
void Set_Var_V0(float val);
void Set_Var_V1(float val);
void Set_Var_Kp(float val);
void Set_Var_Kd(float val);
void Set_Var_Ki(float val);

/*
 * Declaracion Get Functions
 */
int get_Var_timerReady(void);

#endif /* PID_H_ */
