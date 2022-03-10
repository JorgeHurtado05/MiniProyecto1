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
 *
 * Incluimos las librerias necesarias para el proyecto.
 *
 */


#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
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

/*
 * Declaracion de Variables SPI
 */

#define NUM_SPI_DATA    1                 // Número de palabras que se envían cada vez
#define SPI_FREC  4000000                 // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16                 // Número de bits que se envían cada vez, entre 4 y 16
uint16_t dato = 0b0111000000000000;       // Para lo que se envía por SPI.

/*
 * Declaracion de Variables I2C
 */

#define SLAVE_ADDR 0x68     /* 0110 1000 */
// parametros MPU6050
float axf, ayf, azf, gxf, gyf, gzf, sum;

/*
 *
 *  Declaracion de Variables para PID
 *
 */

float v0, v1;                             // Estas variables alojan los valores de entrada del PID.
float uk=0;
float uk2=0;
float ek=0;
float ed=0;
float Ek=0;
float ek_1 = 0;
float Ek_1 = 0;
float Kp = 10;
float Ki = 0;
float Kd = 0;
float delta =0.001;
uint16_t uk_int = 0;

/*
 *
 * Definicion del Handler para el Timer 0
 *
 */

void
Timer0IntHandler(void)
{
    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;
    pui32DataTx[0] = (uint32_t)(dato);
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA ; ui32Index++)
    {
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }
    while(SSIBusy(SSI0_BASE))
    {
    }

    /*
     * Codigo para Lectura ADC (cortesia de Catedraticos Control1)
     *
     * Descomentar esta seccion si se ocupa el ADC
     */

    /*
    uint32_t pui32ADC0Value[2];                         // Arreglo para 2 valores.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, 2);                  // Notar el cambio de "secuencia" (2 en lugar de 3).
    while(!ADCIntStatus(ADC0_BASE, 2, false))           // Notar el cambio de "secuencia".
    {
    }
    ADCIntClear(ADC0_BASE, 2);                          // Limpiamos la interrupción del ADC0
    ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Value);   // Notar el cambio de "secuencia".
    v0 = pui32ADC0Value[0]*3.3/4095.0;  // Convertir a voltios    ADC0  = referencia
    v1 = pui32ADC0Value[1]*3.3/4095.0; // Convertir a voltios    ADC1  = salida (y)
    */

/*
 * Definicion de Codigo para PID Simple.
 *
 * REEMPLAZAR V0 Y V1 POR DATOS DE ANGULO DE IMU
 */
       v0 = 0*3.3/4095.0;  // Conversion de Datos
       v1 = 0*3.3/4095.0; // Convertir de Datos
// Desarrollo de PID REstador
       ek = v0 - v1;
       ed = ek - ek_1;
       Ek = Ek_1 + ek;
       // Sumatoria de PID
       uk = Kp*ek + Ki*Ek*delta + (Kd*ed)/delta;
       ek_1 = ek;
       Ek_1 = Ek;
// Limites de Uk para que salida este entre el rango
       if (uk > 12){
           uk = 12;
       }
       if (uk < -12){
           uk = -12;
       }
// Mapeo para salida por SPI para el DAC
       uk2 = uk*4095.0/3.3;
       uk_int = (int)uk2;
       dato = 0b0111000000000000;
       dato = dato + uk_int;
}


int
main(void)
{
    uint32_t pui32residual[NUM_SPI_DATA];
    uint16_t freq_muestreo = 1000;    // En Hz

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 80 MHz

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

    /*
     *
     * Configuracion Parametros I2C
     *
     */

    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;



    /*
     * Codigo de Activacion de ADC
     * Descomentar si se requiere el uso de ADC
     */

    /*
    // Configuración del ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                        //Activando puerto de los AIN0 y ANI1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);                        // Configura el pin PE3 como AN-Input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);                        // Configura el pin PE2 como AN-Input
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);       // Se configura la secuencia 2 para tomar 2 de 4 posibles muestras.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);             // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);  // Step 1 en la secuencia 2: Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
    ADCSequenceEnable(ADC0_BASE, 2);                                    // Activando secuencia 2 de ADC0
    ADCIntClear(ADC0_BASE, 2);                                          // Limpiamos la bandera de interrupción del ADC0.
    */

    while(1)
    {
    }
}
