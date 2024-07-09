# ANEXO 3 - Programación de tares de tiempo real en Raspberry Pi 4 con PREEMPT_RT

La programación de tareas con requisitos de tiempo real en un sistema operativo (SO) Linux (los mas típicamente soportados por las tarjetas Raspberry Pi) se puede lograr utilizando versiones especializadas del kernel. Específicamente empleando un parche nombrado **"PREEMPT_RT"**, el cual convierte el kernel estándar de Linux en un kernel de tiempo real "blando" (soft real-time), adecuado para muchas aplicaciones industriales y de investigación.

PREEMPT_RT logra la capacidad de tiempo real en Linux a través de una serie de modificaciones y mejoras específicas al kernel. Las dos mas relevantes son:

- **Preemptibilidad Completa:** En un kernel estándar de Linux, ciertas secciones de código no pueden ser interrumpidas, lo que puede causar retrasos impredecibles en la respuesta del sistema a eventos de tiempo real. PREEMPT_RT permite que tareas de mayor prioridad interrumpan casi cualquier proceso del SO. Esto reduce significativamente la latencia.
- **Ajustes en el Planificador:** El planificador de tareas en PREEMPT_RT está mejorado para manejar las políticas de planificación SCHED_FIFO y SCHED_RR de manera más eficiente, garantizando que las tareas de alta prioridad obtengan el tiempo de CPU necesario.

## 3.1 Instalación del parche PREEMPT_RT en la distribución de Linux para Raspberry Pi 4

En primera instancia se debe abordar el método para modificar el comportamiento del kernel, mediante la inclusión del parche "PREEMPT_RT". Los componente requeridos para son:

-1) Una Raspberry Pi 4
-2) Una PC con sistema operativo Linux
-3) Una tarjeta microSD
