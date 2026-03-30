# Solar Tracker CLP5130-1
## Descripción de la página
Repositorio de los contenidos para las cátedras Proyecto 3 y 4. Sistema de seguimiento solar de dos ejes para la estación paraguaya de la red e-CALLISTO.

## Descripción del proyecto
La propuesta consiste en diseñar e implementar un sistema de seguimiento solar de dos ejes para orientar automáticamente la antena log-periódica de la estación paraguaya de la red e-CALLISTO.  
El sistema tendrá una configuración acimut–elevación, donde un eje controlará la orientación horizontal y el segundo controlará la inclinación vertical de la antena.  
La motivación principal del proyecto surge de la necesidad de mejorar la orientación efectiva de la antena durante la ventana diaria de observación solar. 

## Objetivos específicos
- Definir la arquitectura general del sistema, incluyendo subsistemas mecánico, electrónico, de potencia y sensado.
- Diseñar la placa electrónica de control basada en un microcontrolador ESP32-S3, reloj en tiempo real y entradas para encoders.
- Diseñar la estructura mecánica de dos ejes, identificando piezas, uniones, accionamientos, rodamientos y soportes.
- Seleccionar y justificar los componentes principales de accionamiento, transmisión y alimentación.
-  Establecer la lógica general de operación del sistema, incluyendo referencia temporal, cálculo de posición solar, movimiento por ejes y funciones de seguridad.

## Arquitectura general
- **Subsistema mecánico:** estructura portante, ejes de movimiento, soportes, acoples, transmisiones y montaje de antena.
- **Subsistema de control:** placa electrónica con ESP32-S3, RTC y lógica de control embebido.
- **Subsistema de potencia:** fuente principal, distribución de tensiones y driver de motores.
- **Subsistema de sensado:** encoders incrementales y elementos de referencia de posición.
- **Carga final:** antena LPDA de la estación e-CALLISTO.
