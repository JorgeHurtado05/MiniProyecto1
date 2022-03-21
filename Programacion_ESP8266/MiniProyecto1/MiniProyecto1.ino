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

// Librerias de la IMU MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

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
#define VPIN_BUTTON_4    V4 
#define VPIN_BUTTON_5    V5
#define VPIN_BUTTON_6    V6
#define VPIN_BUTTON_7    V7 
#define VPIN_BUTTON_8    V8
#define VPIN_BUTTON_9    V9
#define VPIN_BUTTON_10    V10
#define VPIN_BUTTON_11    V11
#define VPIN_BUTTON_12    V12
#define VPIN_BUTTON_13    V13
#define VPIN_BUTTON_14    V14
#define VPIN_BUTTON_15    V15

/*
 * Declaracion de Variables
 */

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

/*
 * Declaracion Variables PID
 */
static float v0, v1;
static float Kp;
static float Ki;
static float Kd;
float uk=0;
float uk2=0;
float ek=0;
float ed=0;
float Ek=0;
float ek_1 = 0;
float Ek_1 = 0;
float delta =0.001;
uint16_t uk_int = 0;

/*
 * Actualizamos los valores de la nube desde blynk
 */



BLYNK_CONNECTED() {
  // Request the latest state from the server
  Blynk.syncVirtual(VPIN_BUTTON_1);
  Blynk.syncVirtual(VPIN_BUTTON_2);
  Blynk.syncVirtual(VPIN_BUTTON_3);
  Blynk.syncVirtual(VPIN_BUTTON_4);
  Blynk.syncVirtual(VPIN_BUTTON_5);
  Blynk.syncVirtual(VPIN_BUTTON_6);
  Blynk.syncVirtual(VPIN_BUTTON_7);
  Blynk.syncVirtual(VPIN_BUTTON_8);
  Blynk.syncVirtual(VPIN_BUTTON_9);
  Blynk.syncVirtual(VPIN_BUTTON_10);
  Blynk.syncVirtual(VPIN_BUTTON_11);
  Blynk.syncVirtual(VPIN_BUTTON_12);
  Blynk.syncVirtual(VPIN_BUTTON_13);
  Blynk.syncVirtual(VPIN_BUTTON_14);
  Blynk.syncVirtual(VPIN_BUTTON_15);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  
  delay(100);
  BlynkEdgent.begin();
  pinMode(wifiLed, OUTPUT);

/*
 * Setup MPU6050
 */
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

}


void loop() {
  
  BlynkEdgent.run();
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float x = (atan2(a.acceleration.x, sqrt (a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z))*180.0)/3.14;
  float y = (atan2(a.acceleration.y, sqrt (a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z))*180.0)/3.14;
 /*
  * Get Kp, Ki, Kd
  */

  Kp=VPIN_BUTTON_11;
  Ki=VPIN_BUTTON_13;
  Kd=VPIN_BUTTON_14;
  v1=VPIN_BUTTON_4;
/*
 * Desbalance
 */

if (VPIN_BUTTON_15==HIGH){
  v1=-20;
  Blynk.virtualWrite(V4,v1);
  digitalWrite(wifiLed,VPIN_BUTTON_15);
  }
 
/*
 * PID RESTADOR
 */
 
  ek = y - v1;
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

    
  Blynk.virtualWrite(V1,y);
  Blynk.virtualWrite(V2,x);
  Blynk.virtualWrite(V3,uk_int);
  Blynk.virtualWrite(V12,uk);
  Blynk.virtualWrite(V5,a.acceleration.x);
  Blynk.virtualWrite(V6,a.acceleration.y);
  Blynk.virtualWrite(V7,a.acceleration.z);
  Blynk.virtualWrite(V8,g.gyro.x);
  Blynk.virtualWrite(V9,g.gyro.y);
  Blynk.virtualWrite(V10,g.gyro.z);
}
