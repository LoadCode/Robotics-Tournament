## Proyecto: Halcón Milenario

Este proyecto corresponde al desarrollo del software del robot denominado **Halcón Milenario**, que participará en el marco del torneo de robótica de la Universidad del Quindío en la categoría de velocidad (entiendes por qué el nombre?). En este repositorio buscamos mantener la historia tanto de la solución de los problemas a los que nos enfrentamos como de los recursos que utilizamos.


### Desarrolladores
*	**Isabel Cristina Cardona S.**
*	**Julio César Echeverri M.**


### Desarrollo
La aproximación que estamos utilizando para implementar el seguidor de línea consiste en la lista de materiales que se presenta en la siguiente lista.

* Baby Orangutan de Pololu
* Motores Pololu para velocista
* Array de sensores QTR-8A (para detección de la línea)
* Batería LiPo de 7.4V
* Convertidor Step-Down (para alimentación de los sensores)

El algoritmo utilizado para superar los 3 obstáculos planteados en el torneo (cuya reglas y categorías se pueden **~~ver en este enlace~~**), puede ser analizado en el PDF donde a través de un diagrama de flujo se muestran las etapas por las que atravesará la lógica del **Halcón Milenario** ([**Click Aquí**](Diagrama_de_Flujo.pdf))

### Controlador PID
Entre las múltiples opciones que se tienen para hacer que un robot siga una línea (bien diferenciada con respecto a una superficie uniforme), por cuestiones de exploración, hemos decidido implementar un control PID con el fin de obtener toda la ventaja posible con respecto a la velocidad y precisión que nos puede aportar una estrategia de este tipo. Aunque existen controladores más robustos, en nuestro contexto, la selección parece ser adecuada.

Una implementación del controlador PID (entre muchas opciones) se encuentra en el documento de Brett Beauregard y cuya traducción fue realizada por Jonathan Ezequiel, el documento se puede encontrar [**en el siguiente enlace**](http://brettbeauregard.com/blog/wp-content/uploads/2012/07/Gu%C3%ADa-de-uso-PID-para-Arduino.pdf) y es importante leerlo para continuar (y comprender) el desarrollo del proyecto.

#### Ejecución del Controlador (Periodo de Muestreo)
Según se determinó en la bibliografía consultada al respecto de la implementación del controlador PID (entre otros), un aspecto fundamental es que el intervalo de ejecución de un controlador debe ser regular (constante), para cumplir con dicho requerimiento el autor **Brett Beauregard** utiliza un sistema de *polling* o de pregunta constante para determinar si el tiempo necesario ya ha transcurrido para iniciar la próxima iteración de cálculo del controlador.

La opción de *polling* puede utilizarse, sin embargo, no tiene la misma precisión que brinda el sistema de interrupciones de los microcontroladores, que cuentan con *timers* (temporizadores) dedicados a la gestón ótima del tiempo, abordar el tema del periodo de ejecución a través del uso de *interrupciones por cumplimiento de un periodo de tiempo* permite liberar la CPU de la pregunta constante para verificar si el tiempo necesario ha transcurrido (*polling*), permitiendo que aprovechemos la CPU para hacer otros cálculos, como por ejemplo procesar la información que viene de un sensor de color entre otras cosas.

La aproximación elegida para el control de tiempo fue la utilización de la biblioteca de *Arduino* **TimerOne**, el siguiente código ilustra a través de un ejemplo la menra sencilla de enecender y apagar un LED cada 80 milisegundos, es importante hacer notar que el periodo de tiempo debe indicarse en **microsegundos** dada la alta resolución en el manejo de tiempo.

```cpp
#include <TimerOne.h>

#define PIN_LED 7
#define PERIODO 80000 // en microsegundos

void setup()
{
    pinMode(PIN_LED,OUTPUT);
    Timer1.initialize(PERIODO);
    Timer1.attachInterrupt(CambiaLuz);
}

void loop()
{
	// Código para ejecutar cuando la CPU está libre
}


void CambiaLuz()
{
	digitalWrite(PIN_LED,!digitalRead(PIN_LED));
}

```
Teniendo en cuenta el esquema de funcionamiento de la biblioteca **TimerOne**, se plantea indicar como *callback* (la rutina que se ejecuta cada vez que se cumple el tiempo establecido), la función que estará encargada del cálculo del controlador PID.

#### Tiempo de Cálculo del Controlador
Un factor importante en los sistemas dinámicos que cambian con rapidez (por ejemplo, un vehículo de carreras) es el tiempo que se tardará en calcularse la salida del controlador, lo que puede incurrir en el peor de los casos en problemas de *aliasing* y finalmente en situaciones problemáticas para la estabilidad del sistema que se intenta controlar.

A continuación, se propone un esquema de medición para determinar el tiempo aproximado que se tardará el microcontrolador (con un reloj de 20MHz) en realizar el cálculo de la salida del controlador.

```cpp

unsigned long startTime;
unsigned long endTime;
unsigned long delta;

/*Declaración de constantes del controlador*/
const int MOTOR_MIN_SPEED = 25;
const int setpoint = 3200;
const double kp = 1.3;
const double ki = 0.66;
const double kd = 0.1;
const double maxOutput = 255;
const double minOutput = 20;

/*Variables para el controlador*/
volatile double posicionActual = 0;
volatile double error          = 0;
volatile double lastPosicion   = 0;
volatile double dError         = 0;
volatile double errSum         = 0;
volatile double integral       = 0;
volatile double controlOutput  = 0;


void setup()
{
	Serial.begin(9600);
}


void loop()
{
    posicionActual = 3300; //valor que simula la entrada

    startTime = micros();

    /*Cálculo del PID*/
    error = setpoint - posicionActual;
    integral += ki * error;

    // anti-windup
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
    /*Finalización del cálculo del controlador*/

    endTime = micros();
    delta = endTime - startTime;
    Serial.println(delta);
    delay(400);
}
```
Un aspecto muy importante consiste en tener en cuenta que la resolución mínima de la función `micros()` es de 4 microsegundos, sin embargo al conectar por puerto serial y observar los valores de `delta` que son enviados, se puede apreciar que el cálculo de esta implementación del PID se tarda en promedio 64 microsegundos.

**Nota:** El tiempo estimado en el código anterior no tiene en cuenta el tiempo que se invierte en la lectura de los sensores y que es determinante para conocer el periodo mínimo de muestreo del sistema.


### To Do List
[C] Determinar el funcionamiento de las interrupciones de *timer*
[IC] Determinar el tiempo de ejecución del controlador PID
[IC] Verificar calibración y funcionamiento de sensores QTR-8A (determinar umbrales)
[IC] Analizar el funcionamiento del sensor de color
