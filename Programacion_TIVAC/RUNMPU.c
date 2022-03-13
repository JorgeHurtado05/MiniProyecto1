/*
 * RUNMPU.c
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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include <math.h>
//#include "uart.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "RUNMPU.h"

//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
tI2CMInstance g_sI2CMSimpleInst;

//
//Device frequency
//
int clockFreq;

void InitI2C0(void)
{
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

void ConfigureUART(void) { // Função retirada do exemplo hello.c
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

void delayMS(int ms) {
    //ROM_SysCtlDelay( (ROM_SysCtlClockGet()/(3*1000))*ms ) ;  // more accurate
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;  // less accurate
}


/*
 * MPU6050 CALLBACK
 */
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    // Espera a que termine la transmision de datos I2C. g_bMPU6050
    if (ui8Status != I2CM_STATUS_SUCCESS)
       {
       }
    g_bMPU6050Done  = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

//
// The MPU6050 example.
//
void MPU6050Example(void)
{
    float fAccel[3], fGyro[3];
    tMPU6050 sMPU6050;
    float x = 0, y = 0, z = 0;
    //Limpiamos la bandera para esperar a que termine la transmision de datos via MPU6050callback
    g_bMPU6050Done = false;
    /*
     * Inicializamos la IMU mediante la funcion MPU6050Init Seleccionando el modulo fisico
     * (MPU6050), Activamos a la TIVAC como master en I2C, Seleccionamos la direccion I2C
     * de la IMU (0x68), Seleccionamos la funcion (callback) asociada a la IMU. Variable
     * si hay algun error.
     */
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    // esperamos a que el callback se cumpla (Transmision I2C terminada)
    while (!g_bMPU6050Done)
    {
    }
    //Limpiamos la bandera para esperar a que termine la transmision de datos via MPU6050callback
    g_bMPU6050Done = false;
    /*
     * Configuramos los parametros del accelerometro.
     * +-4g
     *
     */
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG, ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
    // esperamos a que el callback se cumpla (Transmision I2C terminada)
    while (!g_bMPU6050Done)
    {
    }
    //Limpiamos la bandera para esperar a que termine la transmision de datos via MPU6050callback
    g_bMPU6050Done = false;
    /*
     * Ajustamos PWR_MGMT_1
     */
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_1, 0x00, 0b00000010 & MPU6050_PWR_MGMT_1_DEVICE_RESET, MPU6050Callback, &sMPU6050);
    // esperamos a que el callback se cumpla (Transmision I2C terminada)
    while (!g_bMPU6050Done)
    {
    }
    //Limpiamos la bandera para esperar a que termine la transmision de datos via MPU6050callback
    g_bMPU6050Done = false;
    /*
     * Ajustamos PWR_MGMT_2
     */
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_PWR_MGMT_2, 0x00, 0x00, MPU6050Callback, &sMPU6050);
    // esperamos a que el callback se cumpla (Transmision I2C terminada)
    while (!g_bMPU6050Done)
    {
    }
    while (1)
    {

        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1],
            &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        z = fGyro[2];
        x = (atan2(fAccel[0], sqrt (fAccel[1] * fAccel[1] + fAccel[2] * fAccel[2]))*180.0)/3.14;
        y = (atan2(fAccel[1], sqrt (fAccel[0] * fAccel[0] + fAccel[2] * fAccel[2]))*180.0)/3.14;
        UARTprintf("Ang. X: %d | Ang. Y: %d | Ang. Z: %d\n", (int)x, (int)y, (int)z);
        //delayMS(100);
    }
}
