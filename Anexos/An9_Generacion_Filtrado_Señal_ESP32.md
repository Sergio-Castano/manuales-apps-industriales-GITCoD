# Generación y Filtrado de Señal en ESP32 usando FreeRTOS, DAC y ADC

Este proyecto implementa un sistema de generación y filtrado digital de señales en un ESP32 utilizando tareas de FreeRTOS.
El sistema produce una onda senoidal de 100 muestras (10 Hz), la envía por DAC, la lee nuevamente mediante ADC y aplica un filtro FIR simple antes de mostrar los resultados por Serial.

## 1. Arquitectura general del sistema

El sistema está compuesto por dos tareas independientes ejecutándose en el núcleo 1 del ESP32:

Tarea 1 – Generador de señal (1 ms por muestra → 10 Hz)

Tarea 2 – Filtro digital FIR (20 ms → 50 Hz)

Además incluye:

DAC1 (GPIO 25) para generar la señal de salida.

ADC (GPIO 34) para leer la señal filtrada.

Un mutex para proteger acceso concurrente al puerto Serial.

El diseño permite mantener frecuencias precisas gracias al uso de vTaskDelayUntil() en ambas tareas.

## 2. Tabla de 100 muestras de seno

El sistema utiliza una tabla precalculada de 100 valores normales entre -1 y 1, permitiendo generar una onda senoidal suave y estable.

100 muestras → 1 ms/muestra → período = 100 ms → 10 Hz

Esta tabla se recorre de forma cíclica dentro de la tarea generadora.

## 3. Tarea 1 – Generación de señal (10 Hz)

La tarea TaskGenerador escribe muestras en el DAC1 (GPIO 25) cada 1 ms, utilizando la tabla de seno.

### 3.1 Funciones principales

Leer el siguiente valor en la tabla seno de 100 puntos.

Convertirlo al rango del DAC:

dacVal = (seno + 1) * 127.5   // 0 a 255


Enviar el valor al DAC mediante dacWrite().

Mantener un periodo exacto de 1 ms usando vTaskDelayUntil.

Esta tarea define la frecuencia principal del sistema: 10 Hz.

## 4. Tarea 2 – Filtrado digital FIR (50 Hz)

La tarea TaskFiltro toma muestras del ADC cada 20 ms y aplica un filtro FIR suave basado en tres coeficientes.

### 4.1 Estructura del filtro FIR (Hanning simple)

El filtro utilizado corresponde a:

y[k] = (x[k] + 2·x[k−1] + x[k−2]) / 4


Donde:

x[k] = nueva lectura del ADC

x[k-1], x[k-2] = valores previos

y[k] = señal filtrada

### 4.2 Funciones principales

Leer el ADC (resolución 12 bits).

Ejecutar la operación FIR.

Mantener los registros internos del filtro (x1, x2).

Proteger la salida Serial utilizando un mutex.

Mantener un periodo estable de 20 ms mediante vTaskDelayUntil.

## 5. Sincronización y protección del Serial

El código emplea:

SemaphoreHandle_t mutexSerial;


La tarea de filtrado usa el mutex para evitar que múltiples hilos escriban en Serial al mismo tiempo, garantizando salidas limpias:

xSemaphoreTake(mutexSerial, portMAX_DELAY);
Serial.print(...);
xSemaphoreGive(mutexSerial);

## 6. Configuración del sistema y creación de tareas FreeRTOS

En setup() se inicializa:

Resolución del ADC

Comunicación Serial

Mutex

Creación de tareas en el core 1

Las tareas se crean así:

xTaskCreatePinnedToCore(TaskGenerador, "Generador", 4096, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(TaskFiltro, "FiltroFIR", 4096, NULL, 1, NULL, 1);

## 7. Código completo del sistema
