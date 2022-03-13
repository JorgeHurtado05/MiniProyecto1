/*
 * Jorge Rafael Hurtado Garcia
 * 18052
 * Codigo MiniProyecto1
 * Diseno e Innovacion
 */

/*
 * Important Information Blynk Connection
 * Se importan librerias extraidas de:
 * https://iotcircuithub.com/home-automation-using-nodemcu-and-blynk/
 * Este recurso hace posible la coneccion del dispositivo
 * a la nube de Blynk.
 */

//Se coloca el ID y token para Blynk.cloud.com
#define BLYNK_TEMPLATE_ID "TMPLwp0w_lx4"
#define BLYNK_DEVICE_NAME "MiniProyecto1"
#define BLYNK_AUTH_TOKEN "T8X8oX5fTxjy_xlrjlg68JKR1qCPvgqp"
//definimos parametros de librerias asociadas.
#define BLYNK_FIRMWARE_VERSION        "0.1.0"
#define BLYNK_PRINT Serial
#define BLYNK_DEBUG
#include "BlynkEdgent.h"
//definimos el microcontrolador (Basado en esp8266)
#define USE_NODE_MCU_BOARD

/*
 * Definimos los puertos conectados en el nodemcu
 */

#define wifiLed   16   //D0
/*
 * Definimos los Pines Virtuales (Blynk)
 */
 
#define VPIN_BUTTON_1    V1

/*
 * Actualizamos los valores de la nube desde blynk
 */



BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(100);
  BlynkEdgent.begin();
  pinMode(wifiLed, OUTPUT);

}


void loop() {
  
  BlynkEdgent.run(); 
}