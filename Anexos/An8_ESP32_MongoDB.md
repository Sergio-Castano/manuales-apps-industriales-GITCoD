Descripción general

Este proyecto implementa un sistema IIoT basado en ESP32 utilizando FreeRTOS, MQTT y filtrado digital. El dispositivo genera una señal senoidal discreta, aplica un filtro pasa bajos tipo Hanning y envía los datos procesados en bloques de 20 muestras hacia un broker MQTT para su posterior análisis o almacenamiento.

El sistema está diseñado para ejecutar tareas concurrentes utilizando los dos núcleos del ESP32 para garantizar estabilidad temporal, separación funcional y envío confiable de datos.

Arquitectura del sistema
1. Generación de señal (TaskSenal)

Produce muestras de una onda senoidal discreta de 20 puntos.

Envía una muestra cada 50 ms mediante una cola FreeRTOS (colaSenal).

Funciona de manera periódica utilizando vTaskDelayUntil.

2. Filtrado digital (TaskFiltro)

Recibe cada muestra desde colaSenal.

Aplica un filtro FIR sencillo equivalente a un filtro Hanning:

y[k]=x[k]+2x[k−1]+x[k−2]4
y[k]=
4
x[k]+2x[k−1]+x[k−2]
	​


Empaqueta la señal original y la filtrada en una estructura Paquete.

Envía el paquete a través de una segunda cola (colaPaquete).

Imprime ambas señales en formato compatible con Serial Plotter.

3. Envío por MQTT (TaskMQTT)

Recibe cada Paquete desde colaPaquete.

Acumula 20 muestras de señal original y 20 filtradas.

Cuando el buffer está lleno:

Construye un documento JSON con campos de identificación y dos arrays numéricos.

Publica el mensaje en el topic MQTT configurado.

Mantiene la conexión con el broker usando client.loop().
