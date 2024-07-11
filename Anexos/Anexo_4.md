# ANEXO 4 -  Programación de tareas de tiempo real en Raspberry Pi usando kernel con PREEMPT_RT

A diferencia de FREERTOS, Linux no es un sistemas operativo de tiempo real, sin embargo, cuenta con algunas políticas para el planificador de tareas que favorecen la ejecución de tareas con requisitos de tiempo real (SCHED_FIFO, SCHED_RR y SCHED_DEADLINE). Mediante la aplicación del parche de tiempo real, como se comentó previamente, el programador de tareas se optimiza para manejar este tipo de políticas.

En esta guía nos enfocaremos en la política **"SCHED_FIFO"** (First In, First Out), la cual es una política de planificación en tiempo real que asigna una prioridad fija a cada tarea. Los niveles de prioridad para una tarea estan en el rango entre 1 (más baja) a 99 (más alta). Además las tareas con SCHED_FIFO siempre tienen una prioridad mayor que las tareas con políticas de planificación normales (SCHED_OTHER). 

Una tarea SCHED_FIFO continuará ejecutándose hasta que se bloquee, explícitamente ceda el control del CPU (mediante sched_yield()) o sea adelantada por una tarea de mayor prioridad. De ese modo la tarea de mayor prioridad no puede ser interrumpida por ninguna otra tarea de igual o menor prioridad. Dentro del mismo nivel de prioridad, las tareas se ejecutan en el orden en que se registraron como listas para ejecutarse.

# 4.1 - Tareas periódicas

Vamos a ver entonces, mediante un ejemplo, cómo programar una tarea periódica con políticas SCHED_FIFO en una Raspberry Pi. El lenguaje de programación a usar será C y para la gestión de tareas se emplearán funciones de programación de hilos, siendo por lo tanto requerida la biblioteca **pthread**. Para este ejemplo específico, en el cual se pretende replicar el  previamente realizado con FREERTOS en la ESP32 [(Anexo 1)](Anexo_1.md), donde mediante una tarea periódica se modificaba el estado de uno de los pines digitales consiguiendo el parpadeo de un led, se requerirá adicionalmente la librería **"wiringPi"**, la cual permite controlar de forma sencilla el estado de los pines GPIO incorporados en las placas Raspberry Pi.

## 1) Instalación del compilador y librerías a usar

Antes de pasar al ejemplo, es pertinente verificar que dispongamos en el sistema con las herramientas y bibliotecas para conseguir su implementación. 

### Verificar la instalación del compilador de código C y bibliotecas estandar
Abra una terminal en la Raspberry Pi 4 y ejecute los siguientes comandos:

```bash
sudo apt update
sudo apt upgrade
sudo apt install -y build-essential
```
### Instalar wiringPi
La biblioteca wiringPi no se encuentra en los repositorios oficiales de Raspbian, por lo tanto debe ser instalada manualmente siguiendo estos pasos:

- Abra una terminal y ejecute el siguiente comando para clonar el repositorio de wiringPi:
```bash
git clone https://github.com/WiringPi/WiringPi.git
```

- Ingrese a la carpeta producto de clonar el repositorio
```bash
cd WiringPi
```

- Compile e instale la biblioteca
```bash
./build
```

## 2) Creación del código



## 3) Compilación del código

## 4) Ejecución del código y verificaciones
