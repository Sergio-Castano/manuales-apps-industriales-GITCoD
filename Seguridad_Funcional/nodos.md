# Proyecto: Sistema de Control Distribuido con Seguridad Funcional sobre Raspberry PLC 19R+ para Centelsa

**Desarrollado por:** Luis Miguel Porras Gaviria  
**Universidad Autónoma de Occidente**  
**Contacto:** lmporras@uao.edu.co

## Descripción del Proyecto

Este proyecto se centra en la implementación de un sistema de control distribuido utilizando ROS 2, enfocado en **seguridad funcional** y ejecutado en un **Raspberry PLC 19R+**. El sistema gestiona nodos de sensores, actuadores y módulos de referencia adaptativa (MOD), los cuales ajustan dinámicamente las señales de control basándose en los **factores de riesgo** del entorno.

El objetivo principal es tener un sistema capaz de tomar decisiones operativas de forma segura frente a situaciones de riesgo, aplicando el protocolo **LOTO (Lock-Out/Tag-Out)** para validación y seguridad. Además, el proyecto utiliza **Singularity** para ejecutar múltiples contenedores, garantizando una correcta **sincronización en tiempo real** y **gestión de prioridades**.

### Características Claves:
- **ROS 2** para control distribuido.
- **Seguridad Funcional** mediante gestión de riesgos y control en tiempo real.
- **MQTT** para publicaciones desde el nodo LOTO.
- Implementación sobre **Raspberry PLC 19R+** con contenedores Singularity para optimización.
- Arquitectura distribuida para manejo de sensores y actuadores.

## Objetivos

- Diseñar un sistema que integre **estrategias de seguridad funcional**.
- Mejorar la **eficiencia** en la gestión de seguridad en entornos industriales como el de **Centelsa**.
- Desarrollar un sistema **dinámico y proactivo** frente a factores de riesgo.

## Metodología

El sistema fue desarrollado sobre un **Raspberry PLC 19R+**, elegido por su flexibilidad y soporte de código abierto, lo cual elimina la necesidad de licencias propietarias. Se utilizó **Singularity** para crear contenedores y gestionar la ejecución de procesos, lo que ayuda a cumplir con los estrictos requisitos de **seguridad funcional**.

El sistema se estructura de la siguiente manera:

- **Nodos de Sensores**: Monitorean parámetros de riesgo y otros factores de seguridad.
- **Nodo MOD**: Regula la referencia de control en función de los datos de riesgo.
- **Nodo LOTO**: Controla la activación y validación del sistema según el estado de seguridad.

Además, se configura **sincronización precisa** y **gestión de prioridades en tiempo real** utilizando técnicas como `SCHED_FIFO`, `mlockall`, y asignación explícita de **núcleos de CPU**.

## Diagrama de Nodos

A continuación, se presenta el diagrama de nodos que ilustra cómo interactúan los diferentes módulos del sistema:

![Diagrama de Nodos](imagenes/image.png)

Este diagrama muestra cómo los nodos de sensores, actuadores, y el nodo de referencia se comunican y se gestionan en tiempo real dentro del sistema. 

## Resultados y Códigos 

El sistema fue probado en ejecución simultánea de múltiples nodos dentro de contenedores Singularity, manteniendo tiempos de **muestra estables** y latencias mínimas, incluso en condiciones críticas. Se realizaron pruebas con **sensores** y **actuadores**, mostrando una **sincronización confiable** y un control eficiente bajo diversas condiciones de carga.

### Ejemplos de Resultados:
- **Tiempos de Muestreo**: Los nodos de sensores mantuvieron un periodo constante de 50 ms, con una **dispersión mínima**.
- **Tiempos de Ejecución**: Los nodos de control, como `actuator_node`, mostraron tiempos de ejecución promedio entre 11 y 36 ms, sin generar latencias críticas.
- **Publicación LOTO**: El nodo `loto_publisher` logró tiempos de ejecución por debajo de 0.3 ms, sin afectar la reactividad del sistema.

A continuación están los códigos junto a la ejecución de los mismos mediante singularity:
1. Ingresar al PLC mediante SSH o una pantalla con micro HDMI
2. Ingresar al contenedor de Singularity con los siguientes comandos
   
   ```bash
   cd singularity_containers/
   sudo singularity shell -w ROS2/
   ```
3. Dentro de Singularity debemos ejecutar estos comandos

    ```bash
   cd ..
   cd ros2_ws/
   source install/setup.bash
   source /opt/ros/humble/setup.bash
   ```
   En caso de no encontrar los ejecutables a momento de usarlos, utilizar: 

    ```bash
   colcon build --packages-select loto_sender_pkg sensor_node2 sensor_node 
   ```
    
    Y después ejecutar

   ```bash
   source install/setup.bash
   source /opt/ros/humble/setup.bash
   ```
   
5. Repetir este proceso 9 veces en total con cada uno de los siguientes códigos
   A. Nodo del Actuador 1
   ```bash
   #include <rclcpp/rclcpp.hpp>
   #include "std_msgs/msg/float64.hpp"
   #include <cstdlib>
   #include <string>
   #include <memory>
   #include <sstream>
   #include <algorithm>
   #include <vector>
   #include <ctime>
   #include <csignal>
   #include <iostream>
   #include <pthread.h>
   #include <sched.h>
   #include <unistd.h>
   #include <sys/mman.h>
   
   bool continuar = true;
   
   void signal_handler(int signum) {
       continuar = false;
   }
   
   class ActuatorNode : public rclcpp::Node
   {
   public:
       ActuatorNode() : Node("actuator_node"), reference_(0.0), error_prev_(0.0), integral_(0.0),
                        Kp_(1.0), Ki_(0.1), Kd_(0.05)
       {
           sensor_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
               "sensor_data", 10,
               std::bind(&ActuatorNode::sensor_callback, this, std::placeholders::_1));
   
           reference_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
               "reference", 10,
               std::bind(&ActuatorNode::reference_callback, this, std::placeholders::_1));
   
           rclcpp::on_shutdown([this]() {
               stop_motor();
           });
   
           last_activation_time_ = {0, 0};
           last_write_time_ = {0, 0};
       }
   
       std::vector<double> tiempos_control;
       std::vector<double> periodos;
   
   private:
       void sensor_callback(const std_msgs::msg::Float64::SharedPtr msg)
       {
           struct timespec t0, t1;
           clock_gettime(CLOCK_MONOTONIC, &t0);
   
           if (last_activation_time_.tv_sec != 0 || last_activation_time_.tv_nsec != 0) {
               double periodo = (t0.tv_sec - last_activation_time_.tv_sec) +
                                (t0.tv_nsec - last_activation_time_.tv_nsec) / 1e9;
               periodos.push_back(periodo);
           }
           last_activation_time_ = t0;
   
           double sensor = msg->data;
           double error = reference_ - sensor;
           integral_ += error;
           integral_ = std::clamp(integral_, -50.0, 50.0);
           double derivative = error - error_prev_;
           double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
           output = std::clamp(output, 0.0, 5.0);
   
           int dac_value = static_cast<int>((output / 10.0) * 4095.0);
           dac_value = std::clamp(dac_value, 0, 4095);
   
           double dt = (t0.tv_sec - last_write_time_.tv_sec) +
                       (t0.tv_nsec - last_write_time_.tv_nsec) / 1e9;
   
           if (dt >= 0.050) {
               std::ostringstream cmd;
               cmd << "/set-analog-output A0.2 " << dac_value;
   
               FILE *pipe = popen(cmd.str().c_str(), "r");
               if (pipe) pclose(pipe);
   
               last_write_time_ = t0;
           }
   
           error_prev_ = error;
   
           clock_gettime(CLOCK_MONOTONIC, &t1);
           double elapsed = (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1e9;
           tiempos_control.push_back(elapsed);
       }
   
       void reference_callback(const std_msgs::msg::Float64::SharedPtr msg)
       {
           reference_ = msg->data;
       }
   
       void stop_motor()
       {
           std::system("/set-analog-output A0.2 0");
           std::cout << "⚠️  Nodo detenido: salida A0.2 puesta en 0." << std::endl;
       }
   
       rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sensor_subscription_;
       rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscription_;
       double reference_, error_prev_, integral_;
       double Kp_, Ki_, Kd_;
       struct timespec last_activation_time_;
       struct timespec last_write_time_;
   };
   
   int main(int argc, char *argv[])
   {
       std::signal(SIGINT, signal_handler);
       rclcpp::init(argc, argv);
   
       // ✅ Evita page faults
       if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
           std::cerr << "⚠️  No se pudo bloquear la memoria RAM." << std::endl;
       }
   
       // ✅ Prioridad tiempo real 81
       struct sched_param param;
       param.sched_priority = 81;
       if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
           std::cerr << "⚠️  No se pudo asignar prioridad tiempo real." << std::endl;
       }
   
       // ✅ Afinidad a núcleo 2
       cpu_set_t cpuset;
       CPU_ZERO(&cpuset);
       CPU_SET(2, &cpuset);
       if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
           std::cerr << "⚠️  No se pudo fijar la afinidad al CPU 2." << std::endl;
       }
   
       auto node = std::make_shared<ActuatorNode>();
   
       struct timespec sleep_time = {0, 500};  // 0.5 microsegundos
   
       while (rclcpp::ok() && continuar) {
           rclcpp::spin_some(node);
           clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, nullptr);
       }
   
       rclcpp::shutdown();
   
       std::cout << "\n📈 Tiempos de ejecución del control (s):\n[";
       for (size_t i = 0; i < node->tiempos_control.size(); ++i) {
           std::cout << node->tiempos_control[i];
           if (i != node->tiempos_control.size() - 1) std::cout << ", ";
       }
       std::cout << "]" << std::endl;
   
       std::cout << "\n📊 Períodos entre tareas de control (s):\n[";
       for (size_t i = 0; i < node->periodos.size(); ++i) {
           std::cout << node->periodos[i];
           if (i != node->periodos.size() - 1) std::cout << ", ";
       }
       std::cout << "]" << std::endl;
   
       return 0;
   }
   ```
   
   Ejecución del código: 
   ```bash
   ros2 run sensor_node sensor_node
   ```
   

## Conclusiones

El sistema distribuido basado en **ROS 2** y ejecutado sobre **Raspberry PLC 19R+** con **Singularity** se mostró como una solución robusta y eficiente para la implementación de **seguridad funcional** en entornos industriales. La **precisión en la temporización** y la **estabilidad en el control de riesgos** permiten una **gestión dinámica y segura** de las operaciones, alineándose con los principios de seguridad funcional y los estándares de la industria.
