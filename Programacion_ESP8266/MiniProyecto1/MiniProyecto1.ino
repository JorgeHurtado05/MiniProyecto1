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
#define VPIN_BUTTON_2    V2
#define VPIN_BUTTON_3    V3

/*
 * Declaracion de Variables
 */

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

/*
 * Actualizamos los valores de la nube desde blynk
 */



BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
  Blynk.syncVirtual(VPIN_BUTTON_3);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  delay(100);
  BlynkEdgent.begin();
  pinMode(wifiLed, OUTPUT);

}


void loop() {
  
  BlynkEdgent.run();
  Blynk.virtualWrite(V3,5);

  // print the string when a newline arrives:
  if (stringComplete) {
    Blynk.virtualWrite(V2,0);
    Blynk.virtualWrite(V2,inputString);
    //Serial.println(inputString.to);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
