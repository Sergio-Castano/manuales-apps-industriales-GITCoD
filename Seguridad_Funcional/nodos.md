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
   
4. Repetir este proceso 9 veces en total con cada uno de los siguientes códigos
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
   
   Ejecución y nombre del código: 
   ```bash
   ros2 run sensor_node actuator_node
   ```
   B. Nodo del sensor 1

   ```bash
   #include <rclcpp/rclcpp.hpp>
   #include "std_msgs/msg/float64.hpp"
   #include <cstdlib>
   #include <string>
   #include <memory>
   #include <array>
   #include <sstream>
   #include <vector>
   #include <ctime>
   #include <csignal>
   #include <iostream>
   #include <pthread.h>
   #include <sched.h>
   #include <unistd.h>
   #include <sys/mman.h>
   
   using std::placeholders::_1;
   
   bool continuar = true;
   bool ignorar_delta = false;
   
   void signal_handler(int) {
       std::cout << "🔴 Señal SIGINT recibida. Cerrando nodo sensor..." << std::endl;
       continuar = false;  // 🚫 Detener el ciclo, pero no hacer shutdown aquí
   }
   
   struct timespec AddTimespecByNs(struct timespec ts, int64_t ns)
   {
       ts.tv_nsec += ns;
       while (ts.tv_nsec >= 1000000000) {
           ++ts.tv_sec;
           ts.tv_nsec -= 1000000000;
       }
       while (ts.tv_nsec < 0) {
           --ts.tv_sec;
           ts.tv_nsec += 1000000000;
       }
       return ts;
   }
   
   class SensorNode : public rclcpp::Node
   {
   public:
       SensorNode() : Node("sensor_node")
       {
           publisher_ = this->create_publisher<std_msgs::msg::Float64>("sensor_data", 10);
           sub_loto_ = this->create_subscription<std_msgs::msg::Float64>(
               "loto", 10, std::bind(&SensorNode::loto_callback, this, _1));
           last_time = {0, 0};
           std::cout << "✅ Nodo sensor_node iniciado correctamente." << std::endl;
       }
   
       void read_sensor()
       {
           struct timespec now;
           clock_gettime(CLOCK_MONOTONIC, &now);
   
           if (!ignorar_delta && (last_time.tv_sec != 0 || last_time.tv_nsec != 0)) {
               double delta = (now.tv_sec - last_time.tv_sec) +
                              (now.tv_nsec - last_time.tv_nsec) / 1e9;
               tiempos.push_back(delta);
           }
           ignorar_delta = false;
           last_time = now;
   
           std::string output = exec("/get-analog-input I0.4");
           try {
               double bin = std::stod(output);
               double voltage = (bin / 4095.0) * 10.0;
   
               auto msg = std_msgs::msg::Float64();
               msg.data = voltage;
               publisher_->publish(msg);
   
               RCLCPP_INFO(this->get_logger(), "Sensor I0.4: %f V", voltage);
           }
           catch (const std::exception &e) {
               RCLCPP_ERROR(this->get_logger(), "Error leyendo o convirtiendo: '%s'", output.c_str());
           }
       }
   
       std::vector<double> tiempos;
       struct timespec next_wakeup;
   
   private:
       std::string exec(const char* cmd)
       {
           std::array<char, 128> buffer;
           std::string result;
           std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
           if (!pipe) throw std::runtime_error("popen() falló");
           while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
               result += buffer.data();
           }
           return result;
       }
   
       void loto_callback(const std_msgs::msg::Float64::SharedPtr msg) {
           double nuevo = msg->data;
           if (loto_anterior_ == 0.0 && nuevo == 1.0) {
               std::cout << "🔄 LOTO flanco 0→1 detectado. Reiniciando tiempo de muestreo." << std::endl;
               clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
               ignorar_delta = true;
           }
           loto_anterior_ = nuevo;
       }
   
       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
       rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_loto_;
       struct timespec last_time;
       double loto_anterior_ = 1.0;
   };
   
   int main(int argc, char ** argv)
   {
       std::signal(SIGINT, signal_handler);
       rclcpp::init(argc, argv);
   
       if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
           std::cerr << "⚠️  No se pudo bloquear memoria con mlockall." << std::endl;
       }
   
       struct sched_param param;
       param.sched_priority = 85;
       cpu_set_t cpuset;
       CPU_ZERO(&cpuset);
       CPU_SET(2, &cpuset);  // Core 2
   
       pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
       pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
   
       auto node = std::make_shared<SensorNode>();
   
       clock_gettime(CLOCK_MONOTONIC, &node->next_wakeup);
       const int64_t periodo_ns = 50 * 1e6;  // 50 ms
   
       while (rclcpp::ok() && continuar) {
           node->read_sensor();
           rclcpp::spin_some(node);
           node->next_wakeup = AddTimespecByNs(node->next_wakeup, periodo_ns);
           clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &node->next_wakeup, nullptr);
       }
   
       std::cout << "\n📈 Tiempos entre llamadas a read_sensor (s):\n[";
       for (size_t i = 0; i < node->tiempos.size(); ++i) {
           std::cout << node->tiempos[i];
           if (i != node->tiempos.size() - 1)
               std::cout << ", ";
       }
       std::cout << "]" << std::endl;
   
       rclcpp::shutdown();  // ✅ shutdown seguro al final
       return 0;
   }
   ```

   Ejecución y nombre del código: 
   ```bash
   ros2 run sensor_node sensor_node
   ```
   C. Nodo del sensor de riesgo 1 


   ```bash                                                                                                                                                                                                                                                                
   #include <rclcpp/rclcpp.hpp>
   #include "std_msgs/msg/float64.hpp"
   #include <cstdlib>
   #include <string>
   #include <memory>
   #include <iostream>
   #include <vector>
   #include <ctime>
   #include <csignal>
   #include <pthread.h>
   #include <sched.h>
   #include <unistd.h>
   #include <sys/mman.h>
   
   bool continuar = true;
   
   void signal_handler(int signum) {
       std::cout << "🔴 Señal SIGINT recibida. Cerrando nodo sensor_riesgo..." << std::endl;
       continuar = false;
       rclcpp::shutdown();
   }
   
   class SensorRiesgo : public rclcpp::Node
   {
   public:
       SensorRiesgo() : Node("sensor_riesgo")
       {
           publisher_ = this->create_publisher<std_msgs::msg::Float64>("riesgo_1", 10);
           last_time_ = {0, 0};
           std::cout << "✅ Nodo sensor_riesgo iniciado correctamente." << std::endl;
       }
   
       void read_sensor()
       {
           struct timespec now;
           clock_gettime(CLOCK_MONOTONIC, &now);
   
           if (last_time_.tv_sec != 0 || last_time_.tv_nsec != 0) {
               double delta = (now.tv_sec - last_time_.tv_sec) +
                              (now.tv_nsec - last_time_.tv_nsec) / 1e9;
               tiempos.push_back(delta);
           }
   
           last_time_ = now;
   
           std::string output = exec("/get-analog-input I0.2");
   
           try {
               double bin = std::stod(output);
               double voltage = (bin / 4095.0) * 10.0;
   
               auto msg = std_msgs::msg::Float64();
               msg.data = voltage;
               publisher_->publish(msg);
   
               std::cout << "[riesgo_1] I0.2 = " << voltage << " V" << std::endl;
           } catch (const std::exception &e) {
               std::cerr << "❌ Error leyendo I0.2: '" << output << "'" << std::endl;
           }
       }
   
       std::vector<double> tiempos;
   
   private:
       std::string exec(const char *cmd)
       {
           char buffer[128];
           std::string result;
           FILE *fp = popen(cmd, "r");
           if (!fp) return "0";
           while (fgets(buffer, sizeof(buffer), fp)) result += buffer;
           fclose(fp);
           return result;
       }
   
       rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
       struct timespec last_time_;
   };
   
   int main(int argc, char **argv)
   {
       std::signal(SIGINT, signal_handler);
       rclcpp::init(argc, argv);
   
       // ✅ Evitar fallos de página
       if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
           std::cerr << "⚠️  No se pudo bloquear la memoria RAM." << std::endl;
       }
   
       // ✅ Prioridad tiempo real 83
       struct sched_param param;
       param.sched_priority = 83;
       if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
           std::cerr << "⚠️  No se pudo asignar prioridad tiempo real." << std::endl;
       }
   
       // ✅ Afinidad al core 2
       cpu_set_t cpuset;
       CPU_ZERO(&cpuset);
       CPU_SET(2, &cpuset);
       if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
           std::cerr << "⚠️  No se pudo fijar la afinidad al CPU 2." << std::endl;
       }
   
       auto node = std::make_shared<SensorRiesgo>();
   
       // ✅ Temporización precisa (wake-up absoluto cada 50 ms)
       struct timespec next_wakeup;
       clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
   
       while (rclcpp::ok() && continuar) {
           rclcpp::spin_some(node);
           node->read_sensor();
   
           // Siguiente wake-up: +50 ms
           next_wakeup.tv_nsec += 50 * 1e6;
           while (next_wakeup.tv_nsec >= 1e9) {
               next_wakeup.tv_nsec -= 1e9;
               next_wakeup.tv_sec++;
           }
           clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
       }
   
       std::cout << "\n📈 Tiempos entre llamadas a read_sensor (s):\n[";
       for (size_t i = 0; i < node->tiempos.size(); ++i) {
           std::cout << node->tiempos[i];
           if (i != node->tiempos.size() - 1)
               std::cout << ", ";
       }
       std::cout << "]" << std::endl;
   
       return 0;
   }
   ```

   Ejecución y nombre del código: 
   ```bash
   ros2 run sensor_node sensor_riesgo
   ```

## Conclusiones

El sistema distribuido basado en **ROS 2** y ejecutado sobre **Raspberry PLC 19R+** con **Singularity** se mostró como una solución robusta y eficiente para la implementación de **seguridad funcional** en entornos industriales. La **precisión en la temporización** y la **estabilidad en el control de riesgos** permiten una **gestión dinámica y segura** de las operaciones, alineándose con los principios de seguridad funcional y los estándares de la industria.
