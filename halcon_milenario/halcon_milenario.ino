/*
 * halcon_milenario.ino
 * Author: Julio Echeverri - Isabel Cardona
 */ 

#include <OrangutanMotors.h>
#include <PololuQTRSensors.h>
#include <TimerOne.h>


#define NUM_SENSORS             6  // numero de sensores utilizados
#define NUM_SAMPLES_PER_SENSOR  4  // La lectura del sensor será el premedio de 4 muestras
#define EMITTER_PIN             2  // Emisor controlado por el pin digital 2
#define PIN_LED_1               9
#define PIN_LED_2               7
#define START_BUTTON            4
#define POT_POSITION            7 // Entrada analógica 7 (donde está conectado el potenci�metro)


/*Tipos de datos personalizados*/
enum SemaforoColor {VERDE, ROJO, INDETERMINADO};
enum Estado {CALIBRANDO, BRECHA_SUPERADA, SEMAFORO_SUPERADO, CARRERA_TERMINADA};

/*Declaraci�n de constantes*/
const int MOTOR_MIN_SPEED = 100;
const int MOTOR_MAX_SPEED = 254;
const int setpoint = 2500;
const long int PERIODO = 23000; // Ts = 23 ms
const double kp = 0.09;
const double ki = 0.001;
const double kd = 0.007;
const double maxOutput = 255;
const double minOutput = -255;

//https://github.com/pololu/qtr-sensors-arduino/blob/master/examples/QTRAExample/QTRAExample.ino
/*Declaraci�n de variables/objetos globales*/
SemaforoColor Semaforo;
OrangutanMotors motores;
//Los sensores 0 a 5 están conectados a las entradas anal�gicas 0 a 5 respectivamente
PololuQTRSensorsAnalog qtra((unsigned char[]) {0, 1, 2, 3, 4, 5}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
volatile unsigned int sensorValues[NUM_SENSORS];
static volatile boolean brechaSuperada    = false;
static volatile boolean semaforoVerde     = false;
static volatile boolean semaforoDetectado = false;
static volatile boolean metaDetectada     = false;
static volatile boolean ejecutarControl   = false;
boolean ejecucion = true;

/*Variables para el controlador*/
volatile double posicionActual = 0;
volatile double error          = 0;
volatile double lastPosicion   = 0;
volatile double dError         = 0;
volatile double errSum         = 0;
volatile double integral       = 0;
volatile double controlOutput  = 0;
volatile double motorSpeedLeft = 0;
volatile double motorSpeedRight= 0;

/*Declaraci�n de Rutinas*/
void CalibracionSensores();
void CalculoControlador();
void ObstaculoSuperado(short obstaculo);
void Informacion(Estado state);
SemaforoColor LeerColorSemaforo();


void setup()
{
	pinMode(PIN_LED_2, OUTPUT);
	pinMode(PIN_LED_1, OUTPUT);
	
	/*Calibración de los sensores*/
  while(digitalRead(START_BUTTON));
	Informacion(CALIBRANDO);
	CalibracionSensores();
	digitalWrite(PIN_LED_1, LOW);	
	Timer1.initialize(PERIODO);
	Timer1.attachInterrupt(CalculoControlador);
}


void loop()
{
  while(digitalRead(START_BUTTON)); // presionar el botón para iniciar la carrera
  ejecucion = true;
  while(ejecucion == true)
  { 
    while(!digitalRead(START_BUTTON)) // se mantiene ejecutando el ciclo mientras el botón no sea presionado
    {
      ejecutarControl = true;
      delay(15000);
      ejecutarControl = false;
      motores.setSpeeds(0,0); 
    }
    if(digitalRead(START_BUTTON))
      ejecucion = false;
  }
  
}


void CalibracionSensores()
{
  // giro en sentido horario
  for(int j = 0; j<7; j++)
  {
    if(j == 0)
      motores.setSpeeds(35,-35);
    else if(j%2 == 0)
      motores.setSpeeds(70,-70);
    else
      motores.setSpeeds(-70,70);
    for (int i = 0; i < 20; i++)
      qtra.calibrate();
  }
  motores.setSpeeds(-35,35);
  delay(550);
  motores.setSpeeds(0,0); //frenado de motores
  delay(100);
}


void CalculoControlador()
{
	posicionActual = qtra.readLine(sensorValues);
	if (ejecutarControl)
	{
    /*Inicia el cálculo del PID*/
		error = setpoint - posicionActual;
		integral += ki * error;
		
		// anti-windup (ajuste excesivo)
		if(integral > maxOutput)
			integral = maxOutput;
		else if(integral < minOutput)
			integral = minOutput;
		
		dError = lastPosicion - posicionActual; // dError = -dInput
		
		controlOutput = kp * error + integral + kd * dError;
		
		// saturación de la salida
		if(controlOutput > maxOutput)
			controlOutput = maxOutput;
		else if(controlOutput < minOutput)
			controlOutput = minOutput;
		
		lastPosicion = posicionActual;
    /*Finaliza el cálculo del PID*/

    /*Actualización del control de los motores*/
    motorSpeedLeft  = MOTOR_MIN_SPEED + controlOutput;
    motorSpeedRight = MOTOR_MIN_SPEED - controlOutput;

    // Verifica que los valores de salida estén dentro de los rangos numéricos aceptados por los motores
    if(motorSpeedLeft > MOTOR_MAX_SPEED)
      motorSpeedLeft = MOTOR_MAX_SPEED;
    if(motorSpeedRight > MOTOR_MAX_SPEED)
      motorSpeedRight = MOTOR_MAX_SPEED;
    if(motorSpeedLeft < MOTOR_MIN_SPEED)
      motorSpeedLeft = MOTOR_MIN_SPEED;
    if(motorSpeedRight < MOTOR_MIN_SPEED)
      motorSpeedRight = MOTOR_MIN_SPEED;
      
    motores.setSpeeds(motorSpeedLeft,motorSpeedRight);
    if(posicionActual <= 2500)// && posicionActual >= 6000)
      digitalWrite(PIN_LED_1, HIGH);
    else
      digitalWrite(PIN_LED_1, LOW);
	}
}


void Informacion(Estado state)
{
	switch (state)
	{
		case CALIBRANDO:
			digitalWrite(PIN_LED_1, HIGH);
			digitalWrite(PIN_LED_2, LOW);
			break;
		case BRECHA_SUPERADA:
			digitalWrite(PIN_LED_1, LOW);
			digitalWrite(PIN_LED_2, HIGH);
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

