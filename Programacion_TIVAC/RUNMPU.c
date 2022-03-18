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
#include "PID.h"


//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
tI2CMInstance g_sI2CMSimpleInst;

/*
 * Variables
 */
static float fAccel[3], fGyro[3];
tMPU6050 sMPU6050;
static float x = 0, y = 0, z = 0;
static uint8_t  Valy_uart=0;


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
     //I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, 16000000);

}

void ConfigureUART(void) { // Funcion extraida del ejemplo hello.c

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
    /*
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    UARTClockSourceSet(UART2_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(2, 115200, 16000000);*/
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
void MPU6050Setup(void)
{
    //tMPU6050 sMPU6050;
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
    /*
    while (1)
    {
        MPU_READ_ANGLE ();
    }
    */
}

void MPU_READ_ANGLE (void)
{

    //tMPU6050 sMPU6050;
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
    //Valy_uart=(int)y;
    //Valy_uart=1;
    //UARTCharPut(UART2_BASE, Valy_uart);
    //UARTSend((uint8_t *)pui8Buffer, uint32_t ui32Count)
    //UARTSend(x, 8);
    //UARTCharPut(UART2_BASE, Valy_uart);
}


/*
 * UART CODE
 */


/*
void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART2_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    ROM_UARTIntClear(UART2_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART2_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART2_BASE,
                                   ROM_UARTCharGetNonBlocking(UART2_BASE));
    }
}

void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPutNonBlocking(UART2_BASE, *pui8Buffer++);
    }
}
void uart2_init(void)
{
    //Prepare System for Uart2
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    //Config UART2 pinout Config BaudRate 115200
    GPIOPinConfigure(GPIO_PD7_U2TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2); // enable uart2
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7); // pines de control del uart
    UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTIntClear(UART2_BASE, UART_INT_RX | UART_INT_RT | UART_INT_TX | UART_INT_FE | UART_INT_PE | UART_INT_BE | UART_INT_OE | UART_INT_RI | UART_INT_CTS | UART_INT_DCD | UART_INT_DSR);

    //huhu
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
    ROM_IntMasterEnable();
    ROM_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    ROM_UARTConfigSetExpClk(UART2_BASE, ROM_SysCtlClockGet(), 115200,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    //huhu
}

*/


/*
 * Get Function
 */

float get_Var_x(void)
{
return x;
}

float get_Var_y(void)
{
return y;
}

float get_Var_z(void)
{
return z;
}

float get_Var_fAccelx(void)
{
return fAccel[0];
}

float get_Var_fAccely(void)
{
return fAccel[1];
}

float get_Var_fAccelz(void)
{
return fAccel[2];
}
