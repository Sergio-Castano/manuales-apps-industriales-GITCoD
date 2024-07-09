# ANEXO 3 - Programación de tares de tiempo real en Raspberry Pi 4 con PREEMPT_RT

La programación de tareas con requisitos de tiempo real en un sistema operativo (SO) Linux (los mas típicamente soportados por las tarjetas Raspberry Pi) se puede lograr utilizando versiones especializadas del kernel. Específicamente empleando un parche nombrado **"PREEMPT_RT"**, el cual convierte el kernel estándar de Linux en un kernel de tiempo real "blando" (soft real-time), adecuado para muchas aplicaciones industriales y de investigación.

PREEMPT_RT logra la capacidad de tiempo real en Linux a través de una serie de modificaciones y mejoras específicas al kernel. Las dos mas relevantes son:

- **Preemptibilidad Completa:** En un kernel estándar de Linux, ciertas secciones de código no pueden ser interrumpidas, lo que puede causar retrasos impredecibles en la respuesta del sistema a eventos de tiempo real. PREEMPT_RT permite que tareas de mayor prioridad interrumpan casi cualquier proceso del SO. Esto reduce significativamente la latencia.
- **Ajustes en el Planificador:** El planificador de tareas en PREEMPT_RT está mejorado para manejar las políticas de planificación SCHED_FIFO y SCHED_RR de manera más eficiente, garantizando que las tareas de alta prioridad obtengan el tiempo de CPU necesario.

## 3.1 Instalación del parche PREEMPT_RT para Raspberry Pi OS

En primera instancia se debe abordar el método para modificar el comportamiento del kernel, mediante la inclusión del parche "PREEMPT_RT". A continuación se detallarán los pasos para conseguirlo, tenga en cuenta que los componente empleados durante esta explicación son:

- Una Raspberry Pi 4 
- Una PC con sistema operativo Linux (Ubuntu 22.04 LTS) 
- Una tarjeta microSD (Con capacidad para 16 Gb o superior)

**Nota: La capacidad de cómputo de la PC repercute directamente en la duración del proceso de compilación cruzada del Kernel, por ende, trate de emplear una PC con un procesador multi-nucleo de alta potencia.* 

### Instalar Raspberry Pi OS (64 bits) en la tarjeta SD

Existen diversos métodos para instalar una imagen del sistema Raspberry Pi OS en una tarjeta SD, en esta guía se empleará la herramienta **Raspberry Pi Imager**.

#### 1) Instalar y ejecutar Raspberry Pi Imager en la PC
En una PC con sistema operativo Ubuntu 22.04, abra una terminal y ejecute los siguiente comandos:

```bash
sudo apt update
sudo apt install rpi-imager
```
Tras completar la instalaación, ejecute el programa mediante el siguiente comando:
```bash
rpi-imager
```
Esto desplegará la siguiente ventana:

![Raspberry pi imager](imgs/ESP32/img_5.png)

#### 3) Conecte la tarjeta micro SD a la 
