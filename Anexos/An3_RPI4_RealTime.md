# ANEXO 3 -  Programación de tareas de tiempo real en Raspberry Pi usando kernel con PREEMPT_RT

A diferencia de FREERTOS, Linux no es un sistemas operativo de tiempo real, sin embargo, cuenta con algunas políticas para el planificador de tareas que favorecen la ejecución de tareas con requisitos de tiempo real (SCHED_FIFO, SCHED_RR y SCHED_DEADLINE). Mediante la aplicación del parche de tiempo real, como se comentó previamente, el programador de tareas se optimiza para manejar este tipo de políticas.

En esta guía nos enfocaremos en la política **"SCHED_FIFO"** (First In, First Out), la cual es una política de planificación en tiempo real que asigna una prioridad fija a cada tarea. Los niveles de prioridad para una tarea estan en el rango entre 1 (más baja) a 99 (más alta). Además las tareas con SCHED_FIFO siempre tienen una prioridad mayor que las tareas con políticas de planificación normales (SCHED_OTHER). 

Una tarea SCHED_FIFO continuará ejecutándose hasta que se bloquee, explícitamente ceda el control del CPU (mediante sched_yield()) o sea adelantada por una tarea de mayor prioridad. De ese modo la tarea de mayor prioridad no puede ser interrumpida por ninguna otra tarea de igual o menor prioridad. Dentro del mismo nivel de prioridad, las tareas se ejecutan en el orden en que se registraron como listas para ejecutarse.

# 1 - Tareas periódicas

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

En el directorio de su elección, cree un archivo con el nombre "blink_rt.c", para ello puede usar el siguiente comando:

```bash
touch blink_rt.c
```

Con el editor de texto de su preferencia, abra el archivo y pegue el siguiente código:

```c
#include <wiringPi.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <signal.h>

int ledPin = 29; // GPIO 21 - Pin 40

// Variable global para indicar si el programa debe seguir corriendo
volatile sig_atomic_t running = 1;

// Prototipo del manejador de señales
void handle_signal(int sig);

// Estructura de parámetros para la tarea de parpadeo
typedef struct {
    int DuracionBlinkMS;
} TaskBlinkParametros;

// Función para sumar un periodo en milisegundos a una estructura timespec
struct timespec SumarPeriodoMS(struct timespec ts, int periodo) {
    ts.tv_nsec += periodo * 1e6;

    while (ts.tv_nsec >= 1000000000L) {
        ts.tv_nsec -= 1000000000L;
        ts.tv_sec++;
    }

    return ts;
}

// Función de la tarea de parpadeo
void *TaskBlink(void *pvParameters) {
    TaskBlinkParametros *parametros = (TaskBlinkParametros *)pvParameters;


    if (wiringPiSetup() == -1) {
        fprintf(stderr, "Error al inicializar wiringPi\n");
        pthread_exit(NULL);
    }

    pinMode(ledPin, OUTPUT);

    struct timespec next_activation;
    int led_estado = LOW;

    clock_gettime(CLOCK_MONOTONIC, &next_activation);

    while (running) {
        led_estado = !led_estado;
        digitalWrite(ledPin, led_estado);
        printf("Estado del LED: %d\n", led_estado); // Imprime el estado del LED
        next_activation = SumarPeriodoMS(next_activation, parametros->DuracionBlinkMS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);
    }

    return NULL;
}
void endRoutine(){
    digitalWrite(ledPin, LOW);
}

int main() {
    // Registrar el manejador de señales para SIGINT y SIGTERM
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    TaskBlinkParametros Mis_Parametros;
    Mis_Parametros.DuracionBlinkMS = 1000; // 1000 milisegundos -> 1 segundo

    pthread_t Task1;
    struct sched_param param;
    pthread_attr_t attr;

    // Inicializa los atributos del hilo
    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // Asigna una prioridad alta al hilo
    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);

    if (pthread_create(&Task1, &attr, TaskBlink, &Mis_Parametros) != 0) {
        fprintf(stderr, "Error al crear el hilo\n");
        return 1;
    }

    pthread_join(Task1, NULL);

    endRoutine();

    printf("Programa terminado\n");
    return 0;
}
// Función manejadora de señales
void handle_signal(int sig) {
    if (sig == SIGINT) {
        printf("\nSIGINT recibido! Apagando LED y terminando el programa...\n");
    } else if (sig == SIGTERM) {
        printf("\nSIGTERM recibido! Apagando LED y terminando el programa...\n");
    }
    running = 0; // Establece running a 0 para terminar el bucle en TaskBlink
}
```

Guarde y cierre el archivo.

**Escribir explicaciones del código*

## 3) Compilación del código

```sh
gcc -o blink_rt.exe blink_rt.c -lwiringPi -lpthread
```

## 4) Ejecución del código y verificaciones

```sh
sudo ./blink_rt.exe
```

# 4.2 - Uso de semaforos y colas 

```c
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <sched.h>
#include <mqueue.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>    // Para O_CREAT y O_RDWR
#include <sys/stat.h> // Para los permisos 0666
#include <math.h>

// Constantes
#define NUM_TASKS 3

// Variables globales

// Mutex
pthread_mutex_t print_mutex;
pthread_mutex_t start_mutex;

// Semáforos
sem_t ready_sem;
sem_t start_sem;

// Colas
#define QUEUE_T3_T2 "/queue_t3_t2"
#define QUEUE_T2_T3 "/queue_t2_t3"
mqd_t mqd_T3_T2, mqd_T2_T3;

#define QUEUE_T3_T1 "/queue_t3_t1"
#define QUEUE_T1_T3 "/queue_t1_t3"
mqd_t mqd_T3_T1, mqd_T1_T3;

pthread_t thread[NUM_TASKS];

struct timespec SumarPeriodoMS(struct timespec ts, int periodo)
{
    ts.tv_nsec += periodo * 1e6;

    // Condición para conservar la structura del timespec y prevenir errores
    while (ts.tv_nsec >= 1000000000L)
    {
        ts.tv_nsec -= 1000000000L;
        ts.tv_sec++;
    }

    return ts;
}

void *task1(void *arg)
{
    float number;

    // Incrementa el semáforo contador protegido por el mutex
    pthread_mutex_lock(&start_mutex);
    sem_post(&ready_sem);
    pthread_mutex_unlock(&start_mutex);

    // Esperar hasta que todas las tareas estén listas
    sem_wait(&start_sem);

    while (1)
    {
        // Recibir el número de la tarea 3
        mq_receive(mqd_T3_T1, (char *)&number, sizeof(number), NULL);

        number = pow(cos(number), 2);

        // Sección crítica protegida por mutex
        pthread_mutex_lock(&print_mutex);
        printf("Tarea 1 - Resultado de cos^2(n): %.2f\n", number);
        pthread_mutex_unlock(&print_mutex);

        // Enviar el número a Tarea 2
        if (mq_send(mqd_T1_T3, (const char *)&number, sizeof(float), 0) == -1)
        {
            perror("mq_send task3");
            exit(1);
        }
    }
    return NULL;
}

void *task2(void *arg)
{
    float number;

    // Incrementa el semáforo contador protegido por el mutex
    pthread_mutex_lock(&start_mutex);
    sem_post(&ready_sem);
    pthread_mutex_unlock(&start_mutex);

    // Esperar hasta que todas las tareas estén listas
    sem_wait(&start_sem);

    while (1)
    {
        // Recibir el número de la tarea 3
        mq_receive(mqd_T3_T2, (char *)&number, sizeof(number), NULL);

        number = pow(sin(number), 2);

        // Sección crítica protegida por mutex
        pthread_mutex_lock(&print_mutex);
        printf("Tarea 2 - Resultado de sin^2(n): %.2f\n", number);
        pthread_mutex_unlock(&print_mutex);

        // Enviar el número a Tarea 2
        if (mq_send(mqd_T2_T3, (const char *)&number, sizeof(float), 0) == -1)
        {
            perror("mq_send task3");
            exit(1);
        }
    }
    return NULL;
}

void *task3(void *arg)
{
    struct timespec next_activation;
    float number;
    float resultado_T1, resultado_T2, resultado_T3;

    // Incrementar el contador protegido por el mutex
    pthread_mutex_lock(&start_mutex);
    sem_post(&ready_sem);
    pthread_mutex_unlock(&start_mutex);

    // Esperar hasta que todas las tareas estén listas
    sem_wait(&start_sem);

    clock_gettime(CLOCK_MONOTONIC, &next_activation);

    while (1)
    {
        // Generar número aleatorio
        number = (float)(rand() % 100) / 10.0;

        // Enviar el número a Tarea 2
        if (mq_send(mqd_T3_T2, (const char *)&number, sizeof(float), 0) == -1)
        {
            perror("mq_send task3");
            exit(1);
        }

        // Enviar el número a Tarea 2
        if (mq_send(mqd_T3_T1, (const char *)&number, sizeof(float), 0) == -1)
        {
            perror("mq_send task3");
            exit(1);
        }

        if (mq_receive(mqd_T1_T3, (char *)&resultado_T1, sizeof(resultado_T1), NULL) == -1)
        {
            perror("mq_send task3");
            exit(1);
        }

        if (mq_receive(mqd_T2_T3, (char *)&resultado_T2, sizeof(resultado_T2), NULL) == -1)
        {
            perror("mq_send task3");
            exit(1);
        }

        resultado_T3 = resultado_T1 + resultado_T2;

        // Sección crítica protegida por mutex
        pthread_mutex_lock(&print_mutex);
        printf("Tarea 3 recibió el número: %.2f\n\n", resultado_T3);
        pthread_mutex_unlock(&print_mutex);

        next_activation.tv_sec += 1;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_activation, NULL);
    }

    return NULL;
}

int main()
{
    struct mq_attr attr;
    pthread_attr_t attr_thread;
    struct sched_param param;
    cpu_set_t cpuset;

    // Inicializar mutexes
    pthread_mutex_init(&print_mutex, NULL);
    pthread_mutex_init(&start_mutex, NULL);

    // Inicializar semáforos
    sem_init(&ready_sem, 0, 0);
    sem_init(&start_sem, 0, 0);

    // Configurar atributos de la cola de mensajes
    attr.mq_flags = 0;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(float);
    attr.mq_curmsgs = 0;

    // Crear las colas de mensajes
    mq_unlink(QUEUE_T3_T2);
    mq_unlink(QUEUE_T2_T3);
    mqd_T3_T2 = mq_open(QUEUE_T3_T2, O_CREAT | O_RDWR, 0666, &attr);
    mqd_T2_T3 = mq_open(QUEUE_T2_T3, O_CREAT | O_RDWR, 0666, &attr);

    mq_unlink(QUEUE_T3_T1);
    mq_unlink(QUEUE_T1_T3);
    mqd_T3_T1 = mq_open(QUEUE_T3_T1, O_CREAT | O_RDWR, 0666, &attr);
    mqd_T1_T3 = mq_open(QUEUE_T1_T3, O_CREAT | O_RDWR, 0666, &attr);

    // Inicializar estructura de atributos para los hilos
    pthread_attr_init(&attr_thread);
    pthread_attr_setinheritsched(&attr_thread, PTHREAD_EXPLICIT_SCHED); // Configuración de herencia de política
    pthread_attr_setschedpolicy(&attr_thread, SCHED_FIFO);              // Configuración de la politica de planificación

    // Fijar los hilos al mismo núcleo
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // Raspberry Pi tiene 4 nucleos -> 0 al 3
    pthread_attr_setaffinity_np(&attr_thread, sizeof(cpu_set_t), &cpuset);

    // Establecer la política SCHED_FIFO y prioridad para el hilo principal
    param.sched_priority = 90;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    // Crear las tareas con diferentes prioridades
    param.sched_priority = 81;
    pthread_attr_setschedparam(&attr_thread, &param);
    pthread_create(&thread[0], &attr_thread, task1, NULL);

    param.sched_priority = 82;
    pthread_attr_setschedparam(&attr_thread, &param);
    pthread_create(&thread[1], &attr_thread, task2, NULL);

    param.sched_priority = 83;
    pthread_attr_setschedparam(&attr_thread, &param);
    pthread_create(&thread[2], &attr_thread, task3, NULL);

    // Esperar hasta que todas las tareas estén listas
    for (int i = 0; i < NUM_TASKS; i++)
    {
        sem_wait(&ready_sem);
    }

    // Liberar las tareas para que inicien al mismo tiempo
    pthread_mutex_lock(&start_mutex);
    for (int i = 0; i < NUM_TASKS; i++)
    {
        sem_post(&start_sem);
    }
    pthread_mutex_unlock(&start_mutex);

    // Esperar a que todas las tareas terminen
    for (int i = 0; i < NUM_TASKS; i++)
    {
        pthread_join(thread[i], NULL);
    }

    // Destruir mutexes y semáforos
    pthread_mutex_destroy(&print_mutex);
    pthread_mutex_destroy(&start_mutex);
    sem_destroy(&ready_sem);
    sem_destroy(&start_sem);

    // Cerrar y eliminar las colas de mensajes
    mq_close(mqd_T3_T2);
    mq_close(mqd_T2_T3);
    mq_close(mqd_T3_T1);
    mq_close(mqd_T1_T3);

    mq_unlink(QUEUE_T3_T2);
    mq_unlink(QUEUE_T2_T3);
    mq_unlink(QUEUE_T3_T1);
    mq_unlink(QUEUE_T1_T3);

    return 0;
}
```

```sh
gcc -o tasks_rt 3tasks_rt.c -lpthread -lm
```

