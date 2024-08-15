# Caso 1: Control distribuido usando Raspberry Pi y DDS (ROS2) 
A continuación se presentan los pasos para implementar un lazo de control distribuido usando **dos (2)** Raspberry Pi 4 como nodos de hardware, las cuales han de contrar con una versión de kernel parcheado con PREEMPT_RT y contenedores Singularity para utilizar el middleware DDS integrado en ROS2 (Versión Humble). Con lo cual deberá haber seguido los procedimientos presentados en los anexos para tener parcheado el SO, haber instalado el paquete WiringPi, tener instalado Singularity y saber cómo crear un contenedor con ROS2.

## Nodo para Lectura de Variable (Nodo Sensor)

### Crear el código para el nodo
1) En una de las Raspberry Pi, dentro de una carpeta de trabajo de su elección, cree un sandbox de Singularity con ROS2 y nómbrelo como "ROS2_sensor_node", para ello abra una terminal, navegue hasta su carpeta de trabajo y use el siguiente comando:
```sh
sudo singularity build --sandbox --arch arm64 ROS2_sensor_node/ docker://ros:humble-ros-base
```

2) En la misma terminal inicie un contenedor para editar la imagen usando el siguiente comando:
```sh
sudo singularity shell -w ROS2_sensor_node/
```

3) Cree el espacio de trabajo:
```sh
mkdir -p /ros2_ws/src
```

4) Ingrese al espacio de trabajo, inicie el entorno de ROS y cree un paquete llamado "sensor_package", el cual tendrá un nodo llamado "sensor_node". Para ello use en orden los siguientes comandos:
```sh
cd /ros2_ws/src
source /opt/ros/humble/setup.bash
ros2 pkg create --build-type ament_cmake --node-name sensor_node sensor_package
```

5) Navegue hasta la carpeta donde se encuentra el archivo del código fuente del nodo:
```sh
cd /ros2_ws/src/sensor_package/src/
```

6) Abra el archivo "sensor_node.cpp" con algun editor de texto (recuerde que pude editarlo desde la terminal o accediendo al archivo desde el entorno gráfico del host) y reemplace su contenido con el siguiente código:
```cpp
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <cstring>        // necessary for strerror
#include <pthread.h>      //necessary for thread and RT
#include <wiringSerial.h> //necessary for serial port com
#include <unistd.h>       // necesary for usleep

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

//Definición de variables globales
int sp;                     // Serial port
const char *test_str = "s"; // Char para indicar al micro leer entrada analógica
float sensor_value = 0.0;

const float alpha = 0.25;
float Sensor_value_filtered = 0.0;

int periodo = 200e6;

struct timespec AddTimespecByNs(struct timespec ts, int64_t ns)
{
  ts.tv_nsec += ns;

  while (ts.tv_nsec >= 1000000000)
  {
    ++ts.tv_sec;
    ts.tv_nsec -= 1000000000;
  }

  while (ts.tv_nsec < 0)
  {
    --ts.tv_sec;
    ts.tv_nsec += 1000000000;
  }

  return ts;
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("sensor_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sensor_topic", 10);
  }

  void publicar(float value)
  {
    auto message = std_msgs::msg::Float64();
    message.data = value;
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{  
  //Inicialización del sistema ROS 2
  rclcpp::init(argc, argv);
  
  //Creación del nodo a partir de la clase declarada
  std::shared_ptr<MinimalPublisher> my_ros_node = std::make_shared<MinimalPublisher>();
  
  //Se inicializa el puerto Serial al cual está conectado la interfáz analógica
  sp = serialOpen("/dev/ttyACM0", 115200); //Con una taza de 115200 Bps
  usleep(50000); // Esperar 50ms a que la conexión con el puerto se establezca correctamente
  serialFlush(sp);
  
  if (sp == -1)
  {
    std::cout << "PORT FAIL TO OPEN" << std::endl;
    exit(1);
  }
  
  //Declaración de variables
  char inChar; //Variable para almacenar el caracter leído del puerto Serial 
  std::string response = ""; //Variable donde se almacenará la cadena de caracteres recibidos por el Serial
  bool response_complete = false; //Bandera para determinar cuando se ha terminado de leer la cadena  
  
  //Se asigna una prioridad de 80 para la tarea
  struct sched_param param;
  param.sched_priority = 80; 
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    
  //La tarea se ancla a un núcleo específico
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(3, &cpuset);
  pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
  
  std::cout << "INICIA EL NODO" << std::endl;

  //Inicio de la tarea periódica
  struct timespec next_wakeup_time_;
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup_time_);
  
  //Se mantiene activa hasta que se genere una interrupción del sistema ROS
  while (rclcpp::ok())
  {
    next_wakeup_time_ = AddTimespecByNs(next_wakeup_time_, periodo);
    try
    {
      //Se envía un caracter a traves del serial para solicitar la lectura del sensor a la interfaz analógica 
      serialPuts(sp, test_str);
      
      //Se espera hasta que hayan caracteres para iniciar la lectura   
      while (serialDataAvail(sp) < 2)
      {
      }
      
      //Se leen uno a uno los caracteres y se concatenan en un string hasta que se detecta el caracter de final de línea
      while (response_complete == false)
      {
        inChar = serialGetchar(sp);
        if (inChar == '\n')
        {
          response_complete = true;
        }
        else
        {
          response += inChar;
      }
      
      //El dato recibido se trasnforma y almacena en una variable numérica de tipo flotante y se escala 
      sensor_value = std::stof(response) * 20; 
      //Se aplica un filtro EMA
      Sensor_value_filtered = alpha * sensor_value + (1 - alpha) * Sensor_value_filtered; 
      //Se redondea a un decimal
      Sensor_value_filtered = roundf(Sensor_value_filtered * 10) / 10; 
        
      response_complete = false; //Se reinicia la bandera
        
      std::cout << Sensor_value_filtered << std::endl; //Se imprime en la terminal el valor a transmitir por DDS
        
      //Se publica la lectura del sensor por DDS
      my_ros_node->publicar(Sensor_value_filtered);
      rclcpp::spin_some(my_ros_node);
      }
      //Se limpia el String para el siguiente ciclo
      response = "";
    }
    catch (...)
    {
      std::cout << "Fallo en comunicación serial" << std::endl;
    }
    
    //Se suspende la tarea hasta el siguiente ciclo
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup_time_, NULL);
  }

  rclcpp::shutdown();

  // exit the current thread
  pthread_exit(NULL);

  return 0;
}
```

7) Compilar el paquete

Recuerde que antes de poder compilar el paquete, debe modificar los archivos Cmake
   

### Programar la interfaz analógica (Arduino Due)

Código para la interfaz analógica:

```cpp
int ADC_read_value = 0;
float sensorValue = 0.0;

int DAC_output_value = 0;
float DAC_output_voltage = 0.0;

String AccionControlString = "";
bool AccionControlStringComplete = false;

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);

  Serial.begin(115200);
  analogWrite(DAC0, DAC_output_value);
}

void loop() {
}

/*
  serialEvent ocurre cada vez que llegan nuevos datos al RX serial del hardware.
*/
void serialEvent() {
  if (Serial.available()) {
    //Se captura el primer caracter
    char opcionChar = (char)Serial.read();

    switch (opcionChar) {

      case 's':
        //Si el primer caracter es una 's' se realiza una lectura del valor analógico en el pin A0
        ADC_read_value = analogRead(A0);
        sensorValue = (ADC_read_value / 4095.0) * 3.3; //Se escala para obtener el voltaje
        //Se envía el valor mediante el puerto Serial
        Serial.println(sensorValue, 3 ); 
        break;

      case 'a': 
        //Si el primer caracter es una 'a' indica que se recibió una acción de control en el formato aX.XX
        AccionControlStringComplete = false;
        //Se leen el resto de caracteres posteriores a la 'a', que corresponden a un voltaje
        while (AccionControlStringComplete == false) {
          if (Serial.available()) {
            char inChar = (char)Serial.read();
            AccionControlString += inChar;
            if (inChar == '\n') {
              AccionControlStringComplete = true;
              DAC_output_voltage = AccionControlString.toFloat();
            }
          }
        }
        DAC_output_value = (4095 * DAC_output_voltage) / 3.30;
        
        if (DAC_output_value < 0) {
          DAC_output_value = 0;
        }
        else if (DAC_output_value > 4095) {
          DAC_output_value = 4095;
        }
        
        analogWrite(DAC0, DAC_output_value);
        AccionControlString = "";
        break;
    }
  }
}
```

8) Programar la interfaz analógica

8) Probar el nodo:
- Conecte e



## Nodo para calcular y aplicar la acción de control (Nodo Controlador)



