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

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
void uart_init(void);


int main(void){

    //Set System Clock
    SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    //Enable Peripheral Control GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Inicializamos UART
    uart_init();
    //Assign GPIO Inputs for Sensors
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6);
    // Sensor Variables
    uint8_t P1_LED=0;
    uint8_t P2_LED=0;
    uint8_t P3_LED=0;
    uint8_t P4_LED=0;

    //Loop Para lectura e impresion de datos.
    while(1)
    {
        // Lectura de los PushButtons.

        // If variable Names are not familiar please refer to the associated pdf file with pinout and Electric Scheme.
        // Same applies for port management in GPIO read and write.

        P1_LED = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2);
        P2_LED = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);
        P3_LED = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4);
        P4_LED = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_6);
        //Enviamos estado de los PushButtons por UART a la TIVAC
        UARTCharPut(UART2_BASE, P1_LED);
        UARTCharPut(UART2_BASE, P2_LED);
        UARTCharPut(UART2_BASE, P3_LED);
        UARTCharPut(UART2_BASE, P4_LED);
        //Iniciamos la revision de que parqueos estan ocupados.

        //if P1 busy, reset red led (same led, reset>red, set>green )print parking not available in display(0)
        if(P1_LED==0)
        {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
        }


        //System delay on loop to restart system
        SysCtlDelay(1);
    }

}



void uart_init(void)
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
}
