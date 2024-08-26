# Caso 2: Control distribuido con requisitos de Hard Real-Time empleando nodos compatibles con protocolo CAN

Para garantizar el cumplimiento de requisitos "Hard Real-Time" se propone el uso de dispositivos dedicados que soporten una red de comunicación determinista y con baja latencia. El protocolo de comunicación seleccionado es CAN (Controller Area Network), entre los dispositivos compatibles con este protocolo, es decir, que cuentan con un controlador CAN integrado se encuentran el Arduino DUE y el ESP32. En este ejemplo de aplicación se van a utilizar placas Arduino DUE para realizar control sobre una planta emulada cuya dinámica pertenece de la velocidad de rotación de un motor DC (Quanser). Se plantea un ambiente distribuido para los dispositivos involucrados en el lazo de control. 

## 1) Nodo Sensor
El nodo sensor se encarga de tomar muestras periodicamente de la salida de la planta, en este caso corresponde a la revoluciones del motor determinadas por un tacómetro cuya salida se expresa en tensión (Voltios). Tras realizar la lectura de la señal analógica, este valor se transmite mediante la red CAN como el mensaje de mas alta prioridad, de modo que se asegure un cumplimiento estricto de la aplicación de la acción de control, la cual depende de la recepción de la lectura del sensor. 

Se programa una placa Arduino Due con el siguiente código:

```c
#include "variant.h"
#include <due_can.h>
#include <DueTimer.h>

#define MAX_CAN_FRAME_DATA_LEN   2
#define PERIODO_uSEG 50000 //50 ms
#define in_pin A0

bool state = false;

CAN_FRAME outgoing;

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, state);
  Can0.begin(CAN_BPS_250K);

  Timer3.attachInterrupt(Capturar_dato);
  analogReadResolution(12);
  pinMode(in_pin, INPUT);

  
  outgoing.id = 10;
  outgoing.extended = false;
  outgoing.priority = 1; //0-15 lower is higher priority
  outgoing.length = MAX_CAN_FRAME_DATA_LEN;
  
  delay(5000);

  Timer3.start(PERIODO_uSEG);

}

void Capturar_dato() {
  state = !state;
  digitalWrite(13, state);
  int sensor_read = analogRead(in_pin);
  outgoing.data.s0 = sensor_read;
  Can0.sendFrame(outgoing);
}


void loop() {
}
```

## 2) Nodo Controlador

## 3) Nodo Puente 

## 4) Conexión de los nodos
