# Sistema de Nodos en ROS 2 con Seguridad Funcional

Este documento describe la arquitectura de un sistema de nodos implementado en ROS 2, diseñado para operar bajo los principios de seguridad funcional. El sistema consta de varios nodos que interactúan entre sí mediante temas de comunicación, lo que permite el monitoreo y control de procesos en tiempo real.

## Arquitectura General

La arquitectura del sistema se muestra en el siguiente diagrama, que ilustra cómo los nodos interactúan entre sí:

![Diagrama de Nodos](./path_to_image/0bc96cbe-3a7b-4e19-a824-c1e18f0266f1.png)

### Descripción de Nodos

1. **loto_publisher**:
   - Publica el mensaje de seguridad del sistema.
   - Tema: `/loto`.

2. **mod_seg_node**:
   - Nodo encargado de recibir la información del riesgo y procesar el modelo de seguridad.
   - Temas: `/riesgo1`, `/riesgo2`, `/referencia`.

3. **sensor_riesgo y sensor_riesgo2**:
   - Nodos de sensores encargados de monitorear diferentes parámetros de riesgo.
   - Temas: `/riesgo1`, `/riesgo2`.

4. **sensor_node y sensor_node2**:
   - Nodos de sensores que se encargan de medir diferentes variables en el sistema.
   - Temas: `/sensor1`, `/sensor2`.

5. **actuator_node y actuator_node2**:
   - Nodos actuadores encargados de realizar ajustes en el sistema según la información recibida.
   - Responden a los mensajes de los sensores y modifican el comportamiento del sistema.

## Seguridad Funcional

### Principios

La seguridad funcional es esencial para garantizar que el sistema sea capaz de detectar y manejar fallos que puedan comprometer su operación. Los nodos están diseñados para garantizar la disponibilidad y confiabilidad del sistema, mediante un enfoque de monitoreo constante y una adecuada validación de la información.

### Monitoreo de Riesgo

El sistema emplea un modelo de monitoreo basado en el procesamiento de los datos de los sensores de riesgo, lo que permite realizar un análisis en tiempo real de la seguridad del sistema. Los nodos de sensores (`sensor_riesgo` y `sensor_riesgo2`) proporcionan información sobre los riesgos, mientras que el nodo `mod_seg_node` procesa estos datos y genera una referencia de seguridad.

### Redundancia

Actualmente, el sistema **no implementa redundancia** en los nodos de sensores ni actuadores. Esta es una característica que se considera para futuras mejoras, con el objetivo de aumentar la fiabilidad y seguridad en el sistema. La redundancia es un mecanismo que permite que si un componente falla, otro componente de respaldo asuma su función sin comprometer el rendimiento del sistema en su totalidad.

### Detección de Fallos

El sistema está diseñado para detectar fallos en tiempo real mediante el uso de mensajes de estado, que son monitoreados y validados por nodos de control. Aunque no se cuenta con redundancia, la detección temprana de fallos permite realizar acciones correctivas antes de que el sistema falle completamente.

## Conclusiones

El sistema ROS 2 descrito en este documento está orientado a la seguridad funcional y a la integración de nodos de monitoreo y control para garantizar su confiabilidad. Aunque la redundancia aún no se encuentra implementada, el sistema está diseñado para detectar y manejar fallos de manera eficiente, asegurando que se pueda operar de forma segura en entornos críticos.

