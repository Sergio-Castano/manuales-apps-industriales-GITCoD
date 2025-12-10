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

## 5. Requisitos del Sistema

ESP32 compatible con FreeRTOS incluido.

Biblioteca PubSubClient para MQTT.

Biblioteca ArduinoJson para construcción del paquete JSON.

Acceso a un broker MQTT (por defecto: broker.emqx.io).

Node-RED y MongoDB para el almacenamiento de los datos enviados.

## 6. Código ESP32
```bash
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ================== WiFi ==================
const char *ssid = "MY_ROUTER";
const char *password = "Autonoma2023";

// ================== MQTT ==================
const char *mqtt_broker = "broker.emqx.io";
const char *topic = "emqx/prueba";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ================== Señal ==================
float valor[20] = {
  0, 0.3247, 0.6142, 0.8372, 0.9694, 0.9966, 0.9158, 0.7357, 0.4759, 0.1646,
  -0.1646, -0.4759, -0.7357, -0.9158, -0.9966, -0.9694, -0.8372, -0.6142, -0.3247, -0.0000
};
int i = 0;

// Buffers para enviar en bloques de 20
float senal_buffer[20];
float filtrada_buffer[20];
int buf_index = 0;

// ================== FreeRTOS ==================
struct Paquete {
  float senal;
  float filtrada;
};

QueueHandle_t colaSenal;
QueueHandle_t colaPaquete;

// Prototipos
void TaskSenal(void *pvParameters);
void TaskFiltro(void *pvParameters);
void TaskMQTT(void *pvParameters);

// Variables de referencia de tiempo
TickType_t lastWakeTime1;
TickType_t lastWakeTime2;
TickType_t lastWakeTime3;

void setup() {
  Serial.begin(115200);

  // Conexión WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");

  // Conexión MQTT
  client.setServer(mqtt_broker, mqtt_port);
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("MQTT broker connected");
    } else {
      Serial.print("Failed MQTT, state ");
      Serial.println(client.state());
      delay(2000);
    }
  }

  // Crear colas
  colaSenal = xQueueCreate(10, sizeof(float));
  colaPaquete = xQueueCreate(10, sizeof(Paquete));

  // Inicializar referencias de tiempo
  lastWakeTime1 = xTaskGetTickCount();
  lastWakeTime2 = xTaskGetTickCount();
  lastWakeTime3 = xTaskGetTickCount();

  // Crear tareas
  xTaskCreatePinnedToCore(TaskSenal,  "Senal",  4096, NULL, 2, NULL, 1);  // Core 1
  xTaskCreatePinnedToCore(TaskFiltro, "Filtro", 4096, NULL, 3, NULL, 1);  // Core 1
  xTaskCreatePinnedToCore(TaskMQTT,   "MQTT",   6144, NULL, 1, NULL, 0);  // Core 0
}

void loop() {
  vTaskSuspend(NULL);
}

// ================== Tarea Señal ==================
void TaskSenal(void *pvParameters) {
  while (true) {
    float muestra = valor[i];
    i++;
    if (i >= 20) i = 0;

    xQueueSend(colaSenal, &muestra, portMAX_DELAY);

    vTaskDelayUntil(&lastWakeTime1, pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}

// ================== Tarea Filtro ==================
void TaskFiltro(void *pvParameters) {
  // Variables locales del filtro → evitan interferencias
  float xk = 0, xk_1 = 0, xk_2 = 0, yk = 0;

  while (true) {
    float entrada;
    if (xQueueReceive(colaSenal, &entrada, portMAX_DELAY)) {
      xk   = entrada;
      yk   = (xk + (2 * xk_1) + xk_2) / 4;
      xk_2 = xk_1;
      xk_1 = xk;

      Paquete pkt;
      pkt.senal = entrada;
      pkt.filtrada = yk;

      xQueueSend(colaPaquete, &pkt, portMAX_DELAY);

      // Para Serial Plotter: solo 2 columnas (señal y filtrada)
      Serial.print(pkt.senal);
      Serial.print("\t");
      Serial.println(pkt.filtrada);
    }
    vTaskDelayUntil(&lastWakeTime2, pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}

// ================== Tarea MQTT ==================
void TaskMQTT(void *pvParameters) {
  Paquete pkt;

  while (true) {
    if (xQueueReceive(colaPaquete, &pkt, portMAX_DELAY)) {
      senal_buffer[buf_index] = pkt.senal;
      filtrada_buffer[buf_index] = pkt.filtrada;
      buf_index++;

      if (buf_index >= 20) {

        StaticJsonDocument<1024> doc;

        // Datos fijos de identificación
        doc["nombre"] = "Luis Miguel Porras";
        doc["cedula"] = "1006109948";
        doc["funcion"] = "Filtro Hanning";

        // Arrays de datos
        JsonArray senalArr = doc.createNestedArray("senal");
        JsonArray filtradaArr = doc.createNestedArray("filtrada");

        for (int j = 0; j < 20; j++) {
          senalArr.add(senal_buffer[j]);
          filtradaArr.add(filtrada_buffer[j]);
        }

        char buffer[1024];
        serializeJson(doc, buffer);
        client.publish(topic, buffer);

        buf_index = 0;
      }
    }

    // Mantener conexión MQTT
    client.loop();

    vTaskDelayUntil(&lastWakeTime3, pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}


```
