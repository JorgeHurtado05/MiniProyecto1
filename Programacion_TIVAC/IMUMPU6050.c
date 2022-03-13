/*
 * IMUMPU6050.c
 *
 *  Created on: Mar 12, 2022
 *      Author: Jorge Hurtado
 *      Description: IMU MPU6050 library (Reads via I2C)
 *      Basado en Librerias extraidas de: https://github.com/mathmagson/mpu6050_tm4c123g
 *
 */

/*
 * Include Necessary libraries:
 */

#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include <math.h>
#include "IMUMPU6050.h"


/*
 * Prototipos de funciones:
 */

void I2C0_INIT(void);
void I2CMSimpleIntHandler(void);
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status);
void MPU6050INIT(void);
void MPU6050READ (float *x, float *y, float *z);

/*
 * I2C protocol via I2C0 (TIVAC)
 */

void I2C0_INIT(void){
 tI2CMInstance g_sI2CMSimpleInst;
// Peripheral Setup for I2C0.
 SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
 SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
 // PortB (PB2__SCL)&&(PB3__SDA) SETUP
 GPIOPinConfigure(GPIO_PB2_I2C0SCL);
 GPIOPinConfigure(GPIO_PB3_I2C0SDA);
 GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
 GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
 //Master INIT I2C0. system clock assign
 I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);
 //FIFO reset.
 HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
 //Initialize I2C Protocol.
 I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());
}

void I2CMSimpleIntHandler(void)
{
    tI2CMInstance g_sI2CMSimpleInst;
    // INTERRUPCION I2C
    I2CMIntHandler(&g_sI2CMSimpleInst);
}


/*
 * MPU6050 CALLBACK
 */

void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // Espera a que termine la transmision de datos I2C. g_bIMUMPU6050
    if (ui8Status != I2CM_STATUS_SUCCESS)
       {
       }
       DoneFlagMPU6050 = true;
}

/*
 * MPU6050 INITIALIZE AND SETUP.
 */

void MPU6050INIT(void)
{
    tI2CMInstance g_sI2CMSimpleInst;
    tMPU6050 sMPU6050;
    //Limpiamos la bandera para esperar a que termine la transmision de datos via MPU6050callback
     DoneFlagMPU6050 = false;
     /*
      * Inicializamos la IMU mediante la funcion MPU6050Init Seleccionando el modulo fisico
      * (MPU6050), Activamos a la TIVAC como master en I2C, Seleccionamos la direccion I2C
      * de la IMU (0x68), Seleccionamos la funcion (callback) asociada a la IMU. Variable
      * si hay algun error.
      */
     MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
     // esperamos a que el callback se cumpla (Transmision I2C terminada)
     while (!DoneFlagMPU6050)
     {
     }
     DoneFlagMPU6050 = false;
     /*
      * Configuramos los parametros del accelerometro.
      * +-4g
      *
      */
     MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
         MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
     // esperamos a que el callback se cumpla (Transmision I2C terminada)
      while (!DoneFlagMPU6050)
      {
      }
      DoneFlagMPU6050 = false;
     /*
      * Ajustamos PWR_MGMT_1
      */
     MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00,
                            0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
      // esperamos a que el callback se cumpla (Transmision I2C terminada)
      while (!DoneFlagMPU6050)
      {
      }
      DoneFlagMPU6050 = false;
      /*
       * Ajustamos PWR_MGMT_2
       */
     MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
     // esperamos a que el callback se cumpla (Transmision I2C terminada)
    while (!DoneFlagMPU6050)
    {
    }
}

/*
 * Lectura de Datos provenientes de MPU6050
 * I2C: 0x68
 */

void MPU6050READ (float *x, float *y, float *z)
{
    tMPU6050 sMPU6050;
    // Bajamos la bandera para leer la IMU
    DoneFlagMPU6050 = false;
    // Mandamos a leer la IMU con la siguiente funcion. (Prepara I2C para lectura)
    MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
    // esperamos a que el callback se cumpla (Transmision I2C terminada)
    while (!DoneFlagMPU6050)
    {
    }
    // Obtenemos los datos del Acelerometro
    MPU6050DataAccelGetFloat(&sMPU6050, &faccel[0], &faccel[1], &faccel[2]);
    // Obtenemos los datos del Giroscopio
    MPU6050DataGyroGetFloat(&sMPU6050, &fgyro[0], &fgyro[1], &fgyro[2]);
    /*
     * Nos interesa conocer el angulo (no la lectura del giroscopio y acelerometro)
     * Hacemos las modificaciones pertinentes para obtener angulo.
     */
    *z = fgyro[2];
    *x = (atan2(faccel[0], sqrt (faccel[1] * faccel[1] + faccel[2] * faccel[2]))*180.0)/3.14;
    *y = (atan2(faccel[1], sqrt (faccel[0] * faccel[0] + faccel[2] * faccel[2]))*180.0)/3.14;

}

