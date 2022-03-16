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
 * Basado en la libreria MPU6050 extraida de:
 * https://github.com/mathmagson/mpu6050_tm4c123g
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
 * Variables para Coneccion de PID con MPU6050
 */

float Anguloy,Kp,Ki,Kd,v1;



int main(void)
{
    Kp=2;
    Ki=0;
    Kd=0;
    v1=0;
    InitI2C0();
    ConfigureUART();
    SetupSPITimer0 ();
    MPU6050Setup();
    Set_Var_Kp(Kp);
    Set_Var_Ki(Ki);
    Set_Var_Kd(Kd);
    Set_Var_V1(v1);
    while(1){
        MPU_READ_ANGLE ();
        //Set V0 (angulox)
        Anguloy=get_Var_y();
        Set_Var_V0(Anguloy);
        delayMS(500);
    }
}

