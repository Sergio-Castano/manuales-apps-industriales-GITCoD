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
## 7. Flujos Node-RED para MQTT, Dashboard y MongoDB

Este anexo describe los flujos implementados en Node-RED para visualizar en tiempo real las señales enviadas por la ESP32 mediante MQTT, así como para almacenar y consultar los datos en MongoDB. Se incluyen ambos flujos y sus archivos JSON listos para importar directamente en Node-RED.

### 1. Flujo 1 – Recepción MQTT, visualización y almacenamiento en MongoDB

Este flujo recibe los mensajes publicados en el tópico emqx/prueba, procesa las señales, las grafica de forma progresiva y genera un documento que se almacena en la colección Filtro_Hanning de MongoDB.

#### 1.1 Funciones principales

Suscripción al tópico MQTT definido.

Construcción de un buffer temporal con las muestras recibidas.

Reproducción secuencial de las señales hacia un gráfico del dashboard cada 50 ms.

Preparación del documento para MongoDB con los campos enviados desde la ESP32.

Inserción del registro en la base de datos.

#### 1.2 JSON del flujo (importar en Node-RED)
```bash
[
    {
        "id": "57b446b45948e3b0",
        "type": "tab",
        "label": "Flow 4",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "659aacadc2cee203",
        "type": "mqtt in",
        "z": "57b446b45948e3b0",
        "name": "",
        "topic": "emqx/prueba",
        "qos": "2",
        "datatype": "auto-detect",
        "broker": "697fb5d7d66c5b98",
        "nl": false,
        "rap": true,
        "rh": 0,
        "inputs": 0,
        "x": 230,
        "y": 240,
        "wires": [
            [
                "31129e04d4e8235b",
                "0c923ca2c20a55eb"
            ]
        ]
    },
    {
        "id": "31129e04d4e8235b",
        "type": "function",
        "z": "57b446b45948e3b0",
        "name": "MOSTRAR SEÑALES",
        "func": "// Guardar el bloque recibido en contexto\nlet buffer = context.get(\"buffer\") || [];\n\n// Agregar las 10 muestras al buffer\nfor (let j = 0; j < msg.payload.senal.length; j++) {\n    buffer.push({\n        senal: msg.payload.senal[j],\n        filtrada: msg.payload.filtrada[j]\n    });\n}\ncontext.set(\"buffer\", buffer);\n\n// Revisar si ya hay un temporizador corriendo\nlet timer = context.get(\"timer\");\nif (!timer) {\n    timer = setInterval(() => {\n        let buf = context.get(\"buffer\") || [];\n        if (buf.length > 0) {\n            let dato = buf.shift();  // sacar el primero\n            context.set(\"buffer\", buf);\n\n            let out1 = { topic: \"Señal\", payload: dato.senal };\n            let out2 = { topic: \"Filtrada\", payload: dato.filtrada };\n\n            node.send([out1, out2]); // enviamos al chart\n        }\n    }, 50);  // cada 50 ms\n    context.set(\"timer\", timer);\n}\n\n// No enviamos nada directo, solo desde el timer\nreturn null;\n",
        "outputs": 2,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 580,
        "y": 400,
        "wires": [
            [
                "20cdbc28e1c5e174"
            ],
            [
                "20cdbc28e1c5e174"
            ]
        ]
    },
    {
        "id": "20cdbc28e1c5e174",
        "type": "ui_chart",
        "z": "57b446b45948e3b0",
        "name": "",
        "group": "4c5e22ad4d89d6d4",
        "order": 0,
        "width": "12",
        "height": "8",
        "label": "chart",
        "chartType": "line",
        "legend": "false",
        "xformat": "HH:mm:ss",
        "interpolate": "linear",
        "nodata": "",
        "dot": false,
        "ymin": "",
        "ymax": "",
        "removeOlder": 1,
        "removeOlderPoints": "50",
        "removeOlderUnit": "3600",
        "cutout": 0,
        "useOneColor": false,
        "useUTC": false,
        "colors": [
            "#1f77b4",
            "#aec7e8",
            "#ff7f0e",
            "#2ca02c",
            "#98df8a",
            "#d62728",
            "#ff9896",
            "#9467bd",
            "#c5b0d5"
        ],
        "outputs": 1,
        "useDifferentColor": false,
        "className": "",
        "x": 910,
        "y": 400,
        "wires": [
            []
        ]
    },
    {
        "id": "0c923ca2c20a55eb",
        "type": "function",
        "z": "57b446b45948e3b0",
        "name": "FUNCION_MONGO",
        "func": "let payload = msg.payload;\n\n// Documento para MongoDB\nlet doc = {\n    timestamp: new Date().toISOString(),   // hora de recepción en el servidor\n    nombre: payload.nombre,\n    cedula: payload.cedula,\n    funcion: payload.funcion,\n    senal: payload.senal,\n    filtrada: payload.filtrada\n};\n\nmsg.payload = doc;\nreturn msg;\n",
        "outputs": 1,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 540,
        "y": 460,
        "wires": [
            [
                "17d179ee57aff50c",
                "406ac776a9bc6e8a"
            ]
        ]
    },
    {
        "id": "17d179ee57aff50c",
        "type": "mongodb4",
        "z": "57b446b45948e3b0",
        "clientNode": "835a518ada87116b",
        "mode": "collection",
        "collection": "Filtro_Hanning",
        "operation": "insertOne",
        "output": "toArray",
        "maxTimeMS": "0",
        "handleDocId": false,
        "name": "",
        "x": 780,
        "y": 460,
        "wires": [
            []
        ]
    },
    {
        "id": "406ac776a9bc6e8a",
        "type": "debug",
        "z": "57b446b45948e3b0",
        "name": "debug 17",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "false",
        "statusVal": "",
        "statusType": "auto",
        "x": 780,
        "y": 560,
        "wires": []
    },
    {
        "id": "697fb5d7d66c5b98",
        "type": "mqtt-broker",
        "name": "",
        "broker": "broker.emqx.io",
        "port": "1883",
        "clientid": "",
        "autoConnect": true,
        "usetls": false,
        "protocolVersion": "4",
        "keepalive": "60",
        "cleansession": true,
        "birthTopic": "",
        "birthQos": "0",
        "birthPayload": "",
        "birthMsg": {},
        "closeTopic": "",
        "closeQos": "0",
        "closePayload": "",
        "closeMsg": {},
        "willTopic": "",
        "willQos": "0",
        "willPayload": "",
        "willMsg": {},
        "userProps": "",
        "sessionExpiry": ""
    },
    {
        "id": "4c5e22ad4d89d6d4",
        "type": "ui_group",
        "name": "Default",
        "tab": "28a4da8d11fa289c",
        "order": 1,
        "disp": true,
        "width": "12",
        "collapse": false,
        "className": ""
    },
    {
        "id": "835a518ada87116b",
        "type": "mongodb4-client",
        "name": "",
        "protocol": "mongodb",
        "hostname": "localhost",
        "port": "27017",
        "dbName": "MQTT_NODERED",
        "appName": "optional",
        "authSource": "optional",
        "authMechanism": "DEFAULT",
        "tls": false,
        "tlsCAFile": "",
        "tlsCertificateKeyFile": "",
        "tlsInsecure": false,
        "connectTimeoutMS": "30000",
        "socketTimeoutMS": "0",
        "minPoolSize": "0",
        "maxPoolSize": "100",
        "maxIdleTimeMS": "0",
        "uri": "",
        "advanced": "{}",
        "uriTabActive": "tab-uri-simple"
    },
    {
        "id": "28a4da8d11fa289c",
        "type": "ui_tab",
        "name": "EMQX",
        "icon": "dashboard",
        "order": 4,
        "disabled": false,
        "hidden": false
    }
]
```

### 2. Flujo 2 – Consulta de datos almacenados en MongoDB y graficación

Este flujo permite consultar la base de datos MQTT_NODERED, extraer los registros previamente almacenados y reconstruir las señales para graficarlas nuevamente con Node-RED Dashboard.

#### 2.1 Funciones principales

Activación manual de la consulta mediante un nodo Inject.

Ejecución de una búsqueda en MongoDB filtrando por nombre.

Ordenamiento de los resultados desde el más reciente.

Reconstrucción de todos los vectores de señal.

Envío secuencial hacia el dashboard para simular transmisión en vivo.

Detención del temporizador cuando no quedan muestras.

#### 2.2 JSON del flujo (importar en Node-RED)
```bash
[
    {
        "id": "e0ee9f0249bc7867",
        "type": "tab",
        "label": "Flow 5",
        "disabled": false,
        "info": "",
        "env": []
    },
    {
        "id": "a1f1f0e5c5e2f8b3",
        "type": "inject",
        "z": "e0ee9f0249bc7867",
        "name": "Disparar consulta",
        "props": [
            {
                "p": "payload"
            }
        ],
        "repeat": "",
        "crontab": "",
        "once": false,
        "onceDelay": 0.1,
        "topic": "",
        "payload": "",
        "payloadType": "date",
        "x": 100,
        "y": 280,
        "wires": [
            [
                "d2f1b45f9a6d2d7b"
            ]
        ]
    },
    {
        "id": "b4c46fbf.b5b62",
        "type": "debug",
        "z": "e0ee9f0249bc7867",
        "name": "Resultado",
        "active": true,
        "tosidebar": true,
        "console": false,
        "tostatus": false,
        "complete": "true",
        "targetType": "full",
        "statusVal": "",
        "statusType": "auto",
        "x": 900,
        "y": 280,
        "wires": []
    },
    {
        "id": "d2f1b45f9a6d2d7b",
        "type": "function",
        "z": "e0ee9f0249bc7867",
        "name": "Query Mongo",
        "func": "msg.payload = {};\nmsg.payload = { nombre: \"Luis Miguel Porras\" };\nmsg.sort = { \"timestamp\": -1 }; // más recientes primero\nreturn msg;\n\n",
        "outputs": 1,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 320,
        "y": 280,
        "wires": [
            [
                "deea49a56ac5c59b"
            ]
        ]
    },
    {
        "id": "deea49a56ac5c59b",
        "type": "mongodb in",
        "z": "e0ee9f0249bc7867",
        "mongodb": "d5b357bf6170f41d",
        "name": "",
        "collection": "Filtro_Hanning",
        "operation": "find",
        "x": 610,
        "y": 280,
        "wires": [
            [
                "b4c46fbf.b5b62",
                "44334a400a98a1b5"
            ]
        ]
    },
    {
        "id": "44334a400a98a1b5",
        "type": "function",
        "z": "e0ee9f0249bc7867",
        "name": "function 7",
        "func": "// Tomamos todos los documentos de la consulta\nlet docs = msg.payload;\n\n// Creamos o recuperamos el buffer\nlet buffer = context.get(\"buffer\") || [];\n\n// Recorremos cada documento (ya ordenado) y agregamos sus datos\nfor (let d of docs) {\n    for (let j = 0; j < d.senal.length; j++) {\n        buffer.push({\n            senal: d.senal[j],\n            filtrada: d.filtrada[j]\n        });\n    }\n}\ncontext.set(\"buffer\", buffer);\n\n// Revisamos si ya hay un temporizador corriendo\nlet timer = context.get(\"timer\");\nif (!timer) {\n    timer = setInterval(() => {\n        let buf = context.get(\"buffer\") || [];\n        if (buf.length > 0) {\n            let dato = buf.shift();  // sacar el primero\n            context.set(\"buffer\", buf);\n\n            let out1 = { topic: \"Señal\", payload: dato.senal };\n            let out2 = { topic: \"Filtrada\", payload: dato.filtrada };\n\n            node.send([out1, out2]); // enviamos al chart (dos salidas)\n        } else {\n            // si ya no hay datos, paramos el timer\n            clearInterval(timer);\n            context.set(\"timer\", null);\n        }\n    }, 50);  // cada 50 ms\n    context.set(\"timer\", timer);\n}\n\n// No enviamos nada directo aquí\nreturn null;\n",
        "outputs": 2,
        "noerr": 0,
        "initialize": "",
        "finalize": "",
        "libs": [],
        "x": 900,
        "y": 360,
        "wires": [
            [
                "57c29be939fe9f7a"
            ],
            [
                "57c29be939fe9f7a"
            ]
        ]
    },
    {
        "id": "57c29be939fe9f7a",
        "type": "ui_chart",
        "z": "e0ee9f0249bc7867",
        "name": "",
        "group": "4c5e22ad4d89d6d4",
        "order": 1,
        "width": 0,
        "height": 0,
        "label": "chart",
        "chartType": "line",
        "legend": "false",
        "xformat": "HH:mm:ss",
        "interpolate": "linear",
        "nodata": "",
        "dot": false,
        "ymin": "",
        "ymax": "",
        "removeOlder": 1,
        "removeOlderPoints": "50",
        "removeOlderUnit": "3600",
        "cutout": 0,
        "useOneColor": false,
        "useUTC": false,
        "colors": [
            "#1f77b4",
            "#aec7e8",
            "#ff7f0e",
            "#2ca02c",
            "#98df8a",
            "#d62728",
            "#ff9896",
            "#9467bd",
            "#c5b0d5"
        ],
        "outputs": 1,
        "useDifferentColor": false,
        "className": "",
        "x": 1050,
        "y": 360,
        "wires": [
            []
        ]
    },
    {
        "id": "d5b357bf6170f41d",
        "type": "mongodb",
        "hostname": "localhost",
        "topology": "direct",
        "connectOptions": "",
        "port": "27017",
        "db": "MQTT_NODERED",
        "name": ""
    },
    {
        "id": "4c5e22ad4d89d6d4",
        "type": "ui_group",
        "name": "Default",
        "tab": "28a4da8d11fa289c",
        "order": 1,
        "disp": true,
        "width": "12",
        "collapse": false,
        "className": ""
    },
    {
        "id": "28a4da8d11fa289c",
        "type": "ui_tab",
        "name": "EMQX",
        "icon": "dashboard",
        "order": 4,
        "disabled": false,
        "hidden": false
    }
]
```
