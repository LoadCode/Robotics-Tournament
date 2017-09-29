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

### Controlador
Entre las múltiples opciones que se tienen para hacer que un robot siga una línea (bien diferenciada con respecto a una superficie uniforme), por cuestiones de exploración, hemos decidido implementar un control PID con el fin de obtener toda la ventaja posible con respecto a la velocidad y precisión que nos puede aportar una estrategia de este tipo. Aunque existen controladores más robustos, en nuestro contexto, la selección parece ser adecuada.

Una implementación del controlador PID (entre muchas opciones) se encuentra en el documento de Brett Beauregard y cuya traducción fue realizada por Jonathan Ezequiel, el documento se puede encontrar [**en el siguiente enlace**](http://brettbeauregard.com/blog/wp-content/uploads/2012/07/Gu%C3%ADa-de-uso-PID-para-Arduino.pdf) y es importante leerlo para continuar (y comprender) el desarrollo del proyecto.