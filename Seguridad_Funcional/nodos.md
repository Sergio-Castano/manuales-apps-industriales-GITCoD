# Proyecto: Sistema de Control Distribuido con Seguridad Funcional sobre Raspberry PLC 19R+ para Centelsa

**Desarrollado por:** Luis Miguel Porras Gaviria  
**Universidad Autónoma de Occidente**  
**Contacto:** lmporras@uao.edu.co

## Descripción del Proyecto

Este proyecto se centra en la implementación de un sistema de control distribuido utilizando ROS 2, enfocado en **seguridad funcional** y ejecutado en un **Raspberry PLC 19R+**. El sistema gestiona nodos de sensores, actuadores y módulos de referencia adaptativa (MOD), los cuales ajustan dinámicamente las señales de control basándose en los **factores de riesgo** del entorno.

El objetivo principal es tener un sistema capaz de tomar decisiones operativas de forma segura frente a situaciones de riesgo, aplicando el protocolo **LOTO (Lock-Out/Tag-Out)** para validación y seguridad. Además, el proyecto utiliza **Singularity** para ejecutar múltiples contenedores, garantizando una correcta **sincronización en tiempo real** y **gestión de prioridades**.

### Características Claves:
- **ROS 2** para control distribuido.
- **Seguridad Funcional** mediante gestión de riesgos y control en tiempo real.
- **MQTT** para publicaciones desde el nodo LOTO.
- Implementación sobre **Raspberry PLC 19R+** con contenedores Singularity para optimización.
- Arquitectura distribuida para manejo de sensores y actuadores.

## Objetivos

- Diseñar un sistema que integre **estrategias de seguridad funcional**.
- Mejorar la **eficiencia** en la gestión de seguridad en entornos industriales como el de **Centelsa**.
- Desarrollar un sistema **dinámico y proactivo** frente a factores de riesgo.

## Metodología

El sistema fue desarrollado sobre un **Raspberry PLC 19R+**, elegido por su flexibilidad y soporte de código abierto, lo cual elimina la necesidad de licencias propietarias. Se utilizó **Singularity** para crear contenedores y gestionar la ejecución de procesos, lo que ayuda a cumplir con los estrictos requisitos de **seguridad funcional**.

El sistema se estructura de la siguiente manera:

- **Nodos de Sensores**: Monitorean parámetros de riesgo y otros factores de seguridad.
- **Nodo MOD**: Regula la referencia de control en función de los datos de riesgo.
- **Nodo LOTO**: Controla la activación y validación del sistema según el estado de seguridad.

Además, se configura **sincronización precisa** y **gestión de prioridades en tiempo real** utilizando técnicas como `SCHED_FIFO`, `mlockall`, y asignación explícita de **núcleos de CPU**.

## Diagrama de Nodos

A continuación, se presenta el diagrama de nodos que ilustra cómo interactúan los diferentes módulos del sistema:

![Diagrama de Nodos](imagenes/image.png)

Este diagrama muestra cómo los nodos de sensores, actuadores, y el nodo de referencia se comunican y se gestionan en tiempo real dentro del sistema. 

## Resultados y Códigos 

El sistema fue probado en ejecución simultánea de múltiples nodos dentro de contenedores Singularity, manteniendo tiempos de **muestra estables** y latencias mínimas, incluso en condiciones críticas. Se realizaron pruebas con **sensores** y **actuadores**, mostrando una **sincronización confiable** y un control eficiente bajo diversas condiciones de carga.

### Ejemplos de Resultados:
- **Tiempos de Muestreo**: Los nodos de sensores mantuvieron un periodo constante de 50 ms, con una **dispersión mínima**.
- **Tiempos de Ejecución**: Los nodos de control, como `actuator_node`, mostraron tiempos de ejecución promedio entre 11 y 36 ms, sin generar latencias críticas.
- **Publicación LOTO**: El nodo `loto_publisher` logró tiempos de ejecución por debajo de 0.3 ms, sin afectar la reactividad del sistema.

A continuación están los códigos junto a la ejecución de los mismos mediante singularity:

## Conclusiones

El sistema distribuido basado en **ROS 2** y ejecutado sobre **Raspberry PLC 19R+** con **Singularity** se mostró como una solución robusta y eficiente para la implementación de **seguridad funcional** en entornos industriales. La **precisión en la temporización** y la **estabilidad en el control de riesgos** permiten una **gestión dinámica y segura** de las operaciones, alineándose con los principios de seguridad funcional y los estándares de la industria.
