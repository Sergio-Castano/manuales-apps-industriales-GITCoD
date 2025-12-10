# ANEXO – Descripción del funcionamiento del sistema ESP32 con FreeRTOS, MQTT y filtrado digital

Este documento describe el funcionamiento del código implementado en el ESP32, el cual genera una señal senoidal discreta, aplica un filtro digital tipo Hanning y envía los datos procesados hacia un broker MQTT utilizando tareas en tiempo real bajo FreeRTOS.
El sistema está organizado en tres tareas independientes, distribuidas entre los dos núcleos del ESP32 para garantizar estabilidad temporal y separación funcional.
