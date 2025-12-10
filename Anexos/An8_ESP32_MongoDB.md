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
Donde:

x[k] es la entrada actual.

x[k-1] y x[k-2] corresponden a muestras previas.

y[k] es la señal filtrada.

### 3.2 Funciones principales

Recibir la muestra mediante colaSenal.

Aplicar el filtro a la señal.

Empaquetar la muestra original y la filtrada dentro de una estructura Paquete.

Enviar el paquete resultante mediante colaPaquete.

Imprimir ambas señales por Serial en formato compatible con Serial Plotter.

Esta tarea se ejecuta en el núcleo 1 con prioridad alta, para asegurar que el filtrado mantenga su periodo de 50 ms.

## 4. Tarea 3 – Envío de datos mediante MQTT (TaskMQTT)

La última tarea se encarga de recibir los paquetes generados por el filtro, agruparlos en bloques de 20 muestras y transmitirlos mediante un mensaje JSON hacia un broker MQTT.

### 4.1 Formato del JSON enviado

El paquete publicado en MQTT posee la siguiente estructura:
```bash
{
  "nombre": "Luis Miguel Porras",
  "cedula": "1006109948",
  "funcion": "Filtro Hanning",
  "senal": [20 valores],
  "filtrada": [20 valores]
}

```
### 4.2 Funciones principales

Recibir estructuras Paquete mediante colaPaquete.

Llenar dos buffers locales: uno para la señal original y otro para la señal filtrada.

Cuando se acumulan 20 muestras:

Generar un documento JSON con ArduinoJson.

Publicar el mensaje en el topic configurado del broker MQTT.

Reiniciar el contador de muestras.

Ejecutar periódicamente client.loop() para mantener la conexión MQTT.

Esta tarea se ejecuta en el núcleo 0 con prioridad baja, ya que no interviene en el procesamiento en tiempo real.

## 5. Flujo general de operación

El sistema completo opera siguiendo la cadena:

TaskSenal genera la muestra.

La muestra viaja por colaSenal.

TaskFiltro la recibe, la filtra y crea un paquete.

El paquete viaja por colaPaquete.

TaskMQTT acumula 20 paquetes y envía el bloque por MQTT.

Este flujo garantiza procesamiento continuo, determinístico y desacoplado entre tareas.
