/*
 -----------------------------------------------------------------------------------------------------
 | VERSIÓN NO.2 DEL CÓDIGO PARA LA PROGRAMACIÓN DE LAS ESTACIONES DE MONITOREO DE CALIDAD DEL AIRE |
 ----------------------------------------------------------------------------------------------------

  NOVEDADES:
    - Se cambió el método de transmisión de los dispositivos, agregando la librería CayenneLPP.
    - Se agregó la función de un IWDG (Independent Watchdog Timer), para reiniciar el MCU en caso de problemas en la ejecución del código
    - Se habilitó un reingreso del dispositivo al servidor cada 24 horas, en caso de la desconexión del mismo por algún problema
  
  CHANGES MADE BY: Nicolás Quintero

*/

//LIBRERÍAS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SoftwareSerial.h>
#include "PMS.h"
#include <TheThingsNetwork.h>
#include <CayenneLPP.h>
#include <Average.h>
#include <IWatchdog.h>

//DEFINICIONES
#define LED PC13
#define RESET_RN PB9
#define GPS_Serial Serial      //Comunicación Serial del GPS conectado a la comunicación serial predeterminada del MCU
#define freqPlan TTN_FP_US915  //Frecuencia de LoRa usada en Colombia (915MHz)
#define iter_send_gps 10       //Numero de iteracciones o lazos de tiempos para transmitir la información del GPS
#define  LIMIT_joint_cont 288 //Numero de iteracciones para volver a hacer join, de acuerdo al tiempo de transmisión

//INSTANCIAS
HardwareSerial Lora_Serial(PA3, PA2);   //Comunicación Serial con el módulo LoRa RN2903
HardwareSerial PMS_Serial(PB11, PB10);  //Comunicación Serial con el sensor de material particulado PMS5003
SoftwareSerial Debug_Serial(PB6, PB7);  //Visualización en el monitor serial

PMS pms(PMS_Serial);  //Objeto Sensor PMS5003
PMS::DATA data;       //Variable que almacena la lectura del sensor

SFE_UBLOX_GNSS gps;  //Objeto GPS

TheThingsNetwork ttn(Lora_Serial, Debug_Serial, freqPlan);  //Se establece la estructura necesaria para la librería TheThingsNetwork

CayenneLPP lpp(51);  //Se establece la estructura necesaria para la librería CayenneLPP, con el fin de enviar tramas de datos

const char *appEui = "XXXXXXXXX";                  //Se configura la variable en 0, porque se utiliza identificación OTAA (Es necesaria para el método JOIN)
const char *appKey = "XXXXXXXXX";  //Identificador OTAA

//Variables de almacenamiento de la lectura del sensor que realizan el promedio móvil de los datos leídos
Average<float> PM1P0_AVG(60);
Average<float> PM2P5_AVG(60);
Average<float> PM10_AVG(60);

float PM1P0 = 0;
float PM2P5 = 0;
float PM10 = 0;

//Variables adicionales
bool B_LED = 0;
byte tim_print = 0;
byte tim_PM = 0;
long time_reset = 0;
bool Confirmed_Data_Send = false;
int security_join_cont = 0;

//Variables de almacenamiento de la lectura del GPS y para realizar el envío de la misma
byte fixType;
float latitude;
float longitude;
int altitude;
int cont_gps = 0;

//-----------------------------//
//Variables para realizar la transmisión cada 3 min aproximadamente
unsigned long previousMillis = 0;

const long interval = 5 * 60000;
//-----------------------------//

void GPS_Get(void);

//Realizar la iteración de los contadores, para ejecutar las tareas requeridas en el loop()
void Update_IT_callback(void) {
  tim_print++;
  tim_PM++;
}

void setup() {

  time_reset = 120000000;  //Tiempo para reset en microsegundos

  IWatchdog.begin(time_reset);  //Inicialización Watchdog Timer, con el fin de que si en dos minutos no se resetea, reinicie el MCU

  //Definición del temporizador
  #if defined(TIM1)
    TIM_TypeDef *Instance = TIM1;
  #else
    TIM_TypeDef *Instance = TIM2;
  #endif


  HardwareTimer *MyTim = new HardwareTimer(Instance);  //Creación y asignación de un temporizador

  pinMode(LED, OUTPUT); //Configuración del pin como salida

  //Reinicio del módulo LoRa RN2903
  pinMode(RESET_RN, OUTPUT);
  delay(100);
  digitalWrite(RESET_RN, LOW);
  delay(1000);
  digitalWrite(RESET_RN, HIGH);

  MyTim->setOverflow(1, HERTZ_FORMAT);         //Configuración del temporizador: Desbordamiento cada 1 seg
  MyTim->attachInterrupt(Update_IT_callback);  //Configuración del temporizador: Rutina a ejecutar luego del desbordamiento
  MyTim->resume();                             //Configuración del temporizador: Inicialización/Arranque

  Lora_Serial.begin(57600);  //Se inicializa la comunicación Serial con el RN2903
  Debug_Serial.begin(9600);  //Se inicializa la comunicación Serial con el MCU
  PMS_Serial.begin(9600);    //Se inicializa la comunicación Serial con el PMS5003
  GPS_Serial.begin(9600);    //Se inicializa la comunicación Serial con el GPS

  delay(300);

  pms.passiveMode();  //Activa el modo pasivo del sensor (sólo realiza mediciones si se le pregunta por las mismas)
  pms.wakeUp();       // Inicializa el sensor y activa el funcionamiento del ventilador interno del mismo

  Debug_Serial.println("Iniciando...");
  delay(300);

  if(gps.begin(Serial) == false) {
    Debug_Serial.println(F("Not GPS"));
  } else {
    Debug_Serial.println("GPS connected");
  }

  ttn.showStatus(); //Imprime en el monitor serial la información del módulo LoRa

  Debug_Serial.println("-- JOIN"); 

  ttn.join(appEui, appKey); //Realiza el ingreso del dispositivo al servidor

  lpp.reset();  //Resetea el buffer o payload
}

void loop() {

  /*
  Condiciones funcionales de acuerdo al temporizador para que cada segundo:
  1. El led del MCU (PC13 ó variable LED) encienda o apague dependiendo de su estado anterior
  2. Se realice la lectura y almacenamiento de las variables del GPS y el sensor PMS5003
  */

  if (tim_print >= 1) {
    tim_print = 0;
    digitalWrite(LED, !digitalRead(LED));
    B_LED = !B_LED;
    fixType = gps.getFixType();
  }

  if (tim_PM >= 2) {
    GPS_Get();
    pms.requestRead();  //Al estar en modo pasivo se requiere esta función, que pregunta para hacer la lectura
    if (pms.readUntil(data)) {
      //Almacenan la información de lo recibido por el sensor en las variables que realizan el promedio móvil de los datos capturados
      PM1P0_AVG.push(data.PM_AE_UG_1_0);
      PM2P5_AVG.push(data.PM_AE_UG_2_5);
      PM10_AVG.push(data.PM_AE_UG_10_0);
    }
    tim_PM = 0;
  }

  //Envia datos dependiendo el valor en milisegundos de la variable interval
  if (millis() - previousMillis >= interval) {
    previousMillis = millis();

    //Toma la media de los promedios hechos de la lectura del sensor
    PM1P0 = (float)PM1P0_AVG.mean()/10; //Se divide en 10, para evitar errores en el envío de la información e información inválida
    PM2P5 = (float)PM2P5_AVG.mean()/10; //Se divide en 10, para evitar errores en el envío de la información e información inválida
    PM10 = PM10_AVG.mean();

    //Almacena las variables a enviar en el buffer de CayenneLPP para el envío
    lpp.reset();
    lpp.addAnalogInput(1, PM1P0);
    lpp.addAnalogInput(2, PM2P5);
    lpp.addAnalogInput(3, PM10);
    
    //Sumatoria ascendente de las variables para el envío de la información del GPS cada 10 envíos de información general y para realizar el reingreso cada 24 horas
    cont_gps = cont_gps + 1;
    security_join_cont = security_join_cont + 1;

    EnableSendGps(cont_gps); //Llama a la función para enviar la información del GPS

    //Si existe algún error en la transmisión de los datos o transcurren 24 horas, se realiza el reingreso del dispositivo al servidor
    if (ttn.sendBytes(lpp.getBuffer(), lpp.getSize(), 1, Confirmed_Data_Send, 0) == TTN_ERROR_SEND_COMMAND_FAILED || security_join_cont >= LIMIT_joint_cont) {
      ttn.reset(0);
      Debug_Serial.println("-- RESET AND JOIN:");  //Necesario para visualizacción en el monitor serial; innecesario para programación final
      ttn.join(appEui, appKey);
      security_join_cont = 0;
    }
  }
  IWatchdog.reload();  //Resetea el Watchdog Timer
}

//FUNCIONES

//Obtiene los datos leídos por el GPS
void GPS_Get(void) {
  if (fixType == 0) Debug_Serial.println(F("No fix"));
  else {
    latitude = (float)gps.getLatitude()/10000000; //Se debe dividir por 1e7, por la codificación de la librería
    longitude = (float)gps.getLongitude()/10000000; //Se debe dividir por 1e7, por la codificación de la librería
    altitude = gps.getAltitude()/1000; //Se debe dividir por 1e3, porque la altitud la entrega en mm
  }
}

//Permite enviar la información del GPS en intermitencia (cumplidas dos transmisiones de datos, envía valores de gps)
void EnableSendGps(int cont) {
  if (cont == iter_send_gps) {
    cont_gps = 0;
    lpp.addGPS(4, latitude, longitude, altitude);
  }
}
