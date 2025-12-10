# ANEXO – Descripción del funcionamiento del sistema ESP32 con FreeRTOS, MQTT y filtrado digital

Este documento describe el funcionamiento del código implementado en el ESP32, el cual genera una señal senoidal discreta, aplica un filtro digital tipo Hanning y envía los datos procesados hacia un broker MQTT utilizando tareas en tiempo real bajo FreeRTOS.
El sistema está organizado en tres tareas independientes, distribuidas entre los dos núcleos del ESP32 para garantizar estabilidad temporal y separación funcional.

## 1. Arquitectura general del sistema

El sistema está compuesto por los siguientes módulos principales:

Generación de señal discreta.

Filtrado digital mediante un filtro FIR.

Empaquetamiento y transmisión de datos usando MQTT.

Colas FreeRTOS para comunicación entre tareas.

Tareas distribuidas entre ambos núcleos del ESP32.

Cada módulo se ejecuta en una tarea propia dentro de FreeRTOS, lo que permite mantener frecuencias de muestreo estables sin interferencias por procesos de comunicación.

## 2. Tarea 1 – Generación de señal (TaskSenal)

La primera tarea del sistema se encarga de generar una señal senoidal discreta compuesta por 20 valores.
Esta señal se recorre de manera cíclica, enviándose una muestra cada 50 ms.

### 2.1 Funciones principales

Produce una muestra tomada del arreglo que contiene los 20 valores de la onda senoidal.

Envía la muestra mediante la cola colaSenal.

Mantiene el intervalo de ejecución fijo utilizando vTaskDelayUntil.

Cuando el índice llega al final del arreglo, reinicia la secuencia.

Esta tarea se ejecuta en el núcleo 1 con prioridad media.

## 3. Tarea 2 – Filtrado digital (TaskFiltro)

La segunda tarea del sistema recibe cada muestra enviada desde la tarea de generación y aplica un filtro FIR pasa bajos equivalente a un filtro Hanning de tres coeficientes.

### 3.1 Funcionamiento del filtro

El cálculo del filtro corresponde a la fórmula:
```bash
y[k] = (x[k] + 2*x[k-1] + x[k-2]) / 4
```
