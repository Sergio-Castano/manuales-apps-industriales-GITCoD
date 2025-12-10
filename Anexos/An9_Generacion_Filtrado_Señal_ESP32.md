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
```bash
y[k] = (x[k] + 2*x[k-1] + x[k-2]) / 4
```
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

```bash
SemaphoreHandle_t mutexSerial;
```

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

```bash
xTaskCreatePinnedToCore(TaskGenerador, "Generador", 4096, NULL, 2, NULL, 1);
xTaskCreatePinnedToCore(TaskFiltro, "FiltroFIR", 4096, NULL, 1, NULL, 1);
```
## 7. Código completo del sistema
```bash
#include <Arduino.h>

const int dacPin = 25;   // DAC1 del ESP32
const int adcPin = 34;   // ADC para leer señal filtrada

// =====================================================
// TABLA DE 100 MUESTRAS DE SENO [-1, 1]
// =====================================================
#define N 100
float seno100[N] = {
  0.0000,0.0634,0.1266,0.1893,0.2511,0.3120,0.3717,0.4298,0.4862,0.5406,
  0.5929,0.6428,0.6901,0.7346,0.7761,0.8146,0.8497,0.8815,0.9096,0.9341,
  0.9549,0.9718,0.9848,0.9938,0.9989,0.9999,0.9969,0.9898,0.9788,0.9638,
  0.9450,0.9224,0.8960,0.8660,0.8326,0.7958,0.7557,0.7127,0.6668,0.6182,
  0.5671,0.5137,0.4582,0.4009,0.3420,0.2817,0.2203,0.1580,0.0951,0.0317,
  -0.0317,-0.0951,-0.1580,-0.2203,-0.2817,-0.3420,-0.4009,-0.4582,-0.5137,-0.5671,
  -0.6182,-0.6668,-0.7127,-0.7557,-0.7958,-0.8326,-0.8660,-0.8960,-0.9224,-0.9450,
  -0.9638,-0.9788,-0.9898,-0.9969,-0.9999,-0.9989,-0.9938,-0.9848,-0.9718,-0.9549,
  -0.9341,-0.9096,-0.8815,-0.8497,-0.8146,-0.7761,-0.7346,-0.6901,-0.6428,-0.5929,
  -0.5406,-0.4862,-0.4298,-0.3717,-0.3120,-0.2511,-0.1893,-0.1266,-0.0634,-0.0000
};

// =====================================================
// MUTEX PARA PROTEGER EL SERIAL
// =====================================================
SemaphoreHandle_t mutexSerial;

// ===================== TAREA 1 — GENERADOR (10 Hz) =====================
// Ts = 1 ms → 100 muestras → 100 ms → 10 Hz
void TaskGenerador(void *pvParameters)
{
    TickType_t LastWakeTime = xTaskGetTickCount();
    const TickType_t Ts = pdMS_TO_TICKS(1);  // 1 ms por muestra = 10 Hz

    int idx = 0;

    for (;;)
    {
        int dacVal = (int)((seno100[idx] + 1.0f) * 127.5f);
        dacWrite(dacPin, dacVal);

        idx++;
        if (idx >= N) idx = 0;

        vTaskDelayUntil(&LastWakeTime, Ts);
    }
}

/*
    ============================================================
        TAREA 2 — FILTRO FIR (cada 20 ms)
    ============================================================
*/
void TaskFiltro(void *pvParameters)
{
    float xk = 0, xk_1 = 0, xk_2 = 0;
    float yk = 0;

    TickType_t LastWakeTime = xTaskGetTickCount();
    const TickType_t Ts = pdMS_TO_TICKS(20);  // 20 ms = 50 Hz

    for (;;)
    {
        xk = analogRead(adcPin);

        // FIR: y = (x + 2x1 + x2) / 4
        yk = (xk + 2*xk_1 + xk_2) / 4.0f;

        xk_2 = xk_1;
        xk_1 = xk;

        // Mutex para Serial
        xSemaphoreTake(mutexSerial, portMAX_DELAY);
        Serial.print("Entrada: ");
        Serial.print(xk);
        Serial.print("\t Filtrada: ");
        Serial.println(yk);
        xSemaphoreGive(mutexSerial);

        vTaskDelayUntil(&LastWakeTime, Ts);
    }
}

/*
    ============================================================
                        SETUP
    ============================================================
*/
void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);

    mutexSerial = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        TaskGenerador,
        "Generador",
        4096,
        NULL,
        2,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        TaskFiltro,
        "FiltroFIR",
        4096,
        NULL,
        1,
        NULL,
        1
    );
}

void loop() {}
```
