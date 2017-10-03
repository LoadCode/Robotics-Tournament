/*
 * halcon_milenario.ino
 * Author: Julio Echeverri - Isabel Cardona
 */ 

#include <OrangutanMotors.h>
#include <PololuQTRSensors.h>
#include <TimerOne.h>

#define NUM_SENSORS             6  // numero de sensores utilizados
#define NUM_SAMPLES_PER_SENSOR  4  // La lectura del sensor ser� el premedio de 4 muestras
#define EMITTER_PIN             2  // Emisor controlado por el pin digital 2
#define PIN_LED_1               3
#define PIN_LED_2               5
#define START_BUTTON            7
#define POT_POSITION            7 // Entrada anal�gica 7 (donde est� conectado el potenci�metro)


/*Tipos de datos personalizados*/
enum SemaforoColor {VERDE, ROJO, INDETERMINADO};
enum Estado {CALIBRANDO, BRECHA_SUPERADA, SEMAFORO_SUPERADO, CARRERA_TERMINADA};

/*Declaraci�n de constantes*/
const int MOTOR_MIN_SPEED = 25;
const int setpoint = 3200;
const long int PERIODO = 50000; // Ts = 50 ms
const double kp;
const double ki;
const double kd;
const double maxOutput = 255;
const double minOutput = 20;

//https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRAExample/QTRAExample.ino
/*Declaraci�n de variables/objetos globales*/
SemaforoColor Semaforo;
OrangutanMotors motores;
//Los sensores 0 a 5 est�n conectados a las entradas anal�gicas 0 a 5 respectivamente
PololuQTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
volatile unsigned int sensorValues[NUM_SENSORS];
static volatile boolean brechaSuperada    = false;
static volatile boolean semaforoVerde     = false;
static volatile boolean semaforoDetectado = false;
static volatile boolean metaDetectada     = false;
static volatile boolean ejecutarControl   = true;

/*Variables para el controlador*/
volatile double posicionActual = 0;
volatile double error          = 0;
volatile double lastPosicion   = 0;
volatile double dError         = 0;
volatile double errSum         = 0;
volatile double integral       = 0;
volatile double controlOutput  = 0;


/*Declaraci�n de Rutinas*/
void CalibracionSensores();
void CalibracionMotores();
void CalculoControlador();
void ObstaculoSuperado(short obstaculo);
void Informacion(Estado state);
SemaforoColor LeerColorSemaforo();


void setup()
{
	pinMode(PIN_LED_2, OUTPUT);
	pinMode(PIN_LED_1, OUTPUT);
	
	/*Calibraci�n de los sensores*/
	Informacion(CALIBRANDO);
	CalibracionMotores();
		
	Timer1.initialize(PERIODO);
	Timer1.attachInterrupt(CalculoControlador);
}


void loop()
{
	// Esperar para iniciar la carrera
	while(digitalRead(START_BUTTON)); // presionar el bot�n para iniciar la carrera
}


void CalibracionSensores()
{
  // giro en sentido horario
  for(int j = 0; j<7; j++)
  {
    if(j == 0)
      motores.setSpeeds(-35,35);
    else if(j%2 == 0)
      motores.setSpeeds(-70,70);
    else
      motores.setSpeeds(70,-70);
    for (int i = 0; i < 20; i++)
      qtra.calibrate();
  }
  motores.setSpeeds(-35,35);
  motores.setSpeeds(0,0); //frenado de motores
  delay(800);
}


void CalculoControlador()
{
	posicionActual = qtra.readLine(sensorValues);
	if (ejecutarControl)
	{
		error = setpoint - posicionActual;
		integral += ki * error;
		
		// anti-windup
		if(integral > maxOutput)
			integral = maxOutput;
		else if(integral < minOutput)
			integral = minOutput;
		
		dError = lastPosicion - posicionActual; // dError = -dInput
		
		controlOutput = kp * error + integral + kd * dError;
		
		// saturaci�n de la salida
		if(controlOutput > maxOutput)
			controlOutput = maxOutput;
		else if(controlOutput < minOutput)
			controlOutput = minOutput;
		
		lastPosicion = posicionActual;
	}
}


void Informacion(Estado state)
{
	switch (state)
	{
		case CALIBRANDO:
			digitalWrite(PIN_LED_1, LOW);
			digitalWrite(PIN_LED_2, HIGH);
			break;
		case BRECHA_SUPERADA:
			digitalWrite(PIN_LED_1, HIGH);
			digitalWrite(PIN_LED_2, LOW);
			break;
		case SEMAFORO_SUPERADO:
			digitalWrite(PIN_LED_1, HIGH);
			digitalWrite(PIN_LED_2, HIGH);
			break;
		case CARRERA_TERMINADA:
			digitalWrite(PIN_LED_1, LOW);
			digitalWrite(PIN_LED_2, LOW);
			break;
	}
}

















































/*                // USE THIS CODE TO VERIFY THE FUNCTIONALLITY OF THE TIMER LIB IN BABY ORANGUTAN
#include "TimerOne/TimerOne.h"

#define LED_PIN 8
#define TS      50000  // 50ms period


void ControllerCompute();


void setup()
{
	pinMode(LED_PIN, OUTPUT);
	Timer1.initialize(TS);
	Timer1.attachInterrupt(ControllerCompute);
	Serial.begin(9600);
}

void loop()
{
	delay(500);
}

void ControllerCompute()
{
	Serial.println("Calculando PID");
}
*/


/*  Alternative way of calling the library 
#include "OrangutanMotors/OrangutanMotors.h"
#include "TimerOne/TimerOne.h"
#include "PololuQTRSensors/PololuQTRSensors.h"
*/
