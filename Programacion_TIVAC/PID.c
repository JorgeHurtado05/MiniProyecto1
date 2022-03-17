/*
 * RUNMPU.c
 *
 *  Created on: Mar 12, 2022
 *      Author: Jorge Hurtado
 *      Description: PID LIBRARY, SPI_DAC Protocol.
 *      Basado en Librerias extraidas de: Sistemas de Control1 Proyecto Final.
 *
 */


/*
 * Include Libraries
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"
#include "RUNMPU.h"
#include "PID.h"

/*
 * Declaracion de Variables de Ingreso de datos
 */

static float v0, v1;
static float Kp;
static float Ki;
static float Kd;
static int timerReady=0;


/*
 * Declaracion de Variables para PID
 */

float uk=0;
float uk2=0;
float ek=0;
float ed=0;
float Ek=0;
float ek_1 = 0;
float Ek_1 = 0;
float delta =0.001;
uint16_t uk_int = 0;

#define NUM_SPI_DATA    1                 // Número de palabras que se envían cada vez
#define SPI_FREC  4000000                 // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16                 // Número de bits que se envían cada vez, entre 4 y 16

uint16_t dato = 0b0111000000000000;       // Para lo que se envía por SPI.


void SetupSPITimer0 (void)
{
    uint32_t pui32residual[NUM_SPI_DATA];
    uint16_t freq_muestreo = 1000;    // En Hz
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ); // 80 MHz
    // Configuración de SPI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
    SSIEnable(SSI0_BASE);

    //Configurando Timer 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_muestreo));
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
}




void Timer0IntHandler(void)
{
    timerReady=0;
    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    pui32DataTx[0] = (uint32_t)(dato);
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA ; ui32Index++)
    {
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }
    while(SSIBusy(SSI0_BASE))
    {
    }
    PID_RESTADOR ();
}


void PID_RESTADOR (void)
{
/*
 * PID RESTADOR
 */
       ek = v0 - v1;
       ed = ek - ek_1;
       Ek = Ek_1 + ek;
       // Sumatoria de PID
       uk = Kp*ek + Ki*Ek*delta + (Kd*ed)/delta;
       ek_1 = ek;
       Ek_1 = Ek;
/*
 * MAPEO PID (LIMITE DE RANGO)
 */
       if (uk > 10){
           uk = 10;
       }
       if (uk < -10){
           uk = -10;
       }
// Mapeo para salida por SPI para el DAC rango -26 a 26 grados permisibles
       uk2 = (uk+10)*4095.0/20;
       uk_int = (int)uk2;
       dato = 0b0111000000000000;
       dato = dato + uk_int;
}





/*
 * Set Functions
 */

void Set_Var_V0(float val)
{
    v0=val ;
}

void Set_Var_V1(float val)
{
    v1=val ;
}

void Set_Var_Kp(float val)
{
    Kp=val ;
}

void Set_Var_Kd(float val)
{
    Kd=val ;
}

void Set_Var_Ki(float val)
{
    Ki=val ;
}


/*
 * Get Function
 */

int get_Var_timerReady(void)
{
return timerReady;
}

