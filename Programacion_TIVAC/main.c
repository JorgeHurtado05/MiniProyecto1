/*
 * Jorge Rafael Hurtado Garcia
 * 18052
 * MiniProyecto1
 * Diseno e Innovacion
 * Codigo TIVAC
 *
 * Lectura de Angulo de IMU MPU6050 I2C
 * Controlador PID Con salida a DAC SPI
 *
 * Basado en los ejemplos single_ended.c y timers.c de TivaWare
 *
 */

/*
 * Incluimos las librerias necesarias para el proyecto.
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
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include <math.h>
//#include "uart.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "RUNMPU.h"
#include "PID.h"

/*
 * Variables de control PID
 */




int main(void)
{
    Kp = 1;
    Ki = 1;
    Kd = 1;
    v0 = 10;
    InitI2C0();
    ConfigureUART();
    SetupSPITimer0 ();
    MPU6050Example();
}

