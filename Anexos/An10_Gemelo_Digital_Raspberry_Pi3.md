# Gemelo Digital del Motor DC Quanser en ROS2 Jazzy (Tiempo Real, SCHED_FIFO)

Este proyecto implementa un Gemelo Digital del motor DC Quanser utilizando ROS2 Jazzy dentro de un entorno Linux con capacidades de tiempo real (RT-PREEMPT). El sistema integra:

Un modelo discreto experimental (motor_sim_node.cpp)

Un modelo teórico discretizado vía ZOH (motor_teorico_zoh.cpp)

Dos controladores PID discretos (dual_pid_node2.cpp y dual_pid_node3.cpp)

Afinidad de CPU, planificación SCHED_FIFO y bloqueo de memoria

Registro del tiempo de ejecución y sample time para evaluar determinismo

El propósito es validar dinámica, estabilidad temporal y confiabilidad del sistema corriendo en tiempo real.

## 1. Dependencias
```bash
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-rmw-fastrtps*
sudo apt install build-essential
sudo apt install rt-tests
```
## 2. Compilación

Estructura recomendada:
```bash
ros2_ws/src/motor_digital_twin/
└── src/
    ├── motor_sim_node.cpp
    ├── motor_teorico_zoh.cpp
    ├── dual_pid_node2.cpp
    ├── dual_pid_node3.cpp
```

Compilar:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Arquitectura de Tópicos
### 3.1 Planta simulada (modelo identificado)

#### motor_sim_node

Publica: /motor_speed_sim_disc

Se suscribe: /cmd_voltage_disc

#### dual_pid_node2

Publica: /cmd_voltage_disc

Se suscribe:

/ref_speed_sim (REFERENCIA EN rad/s)

/motor_speed_sim_disc

### 3.2 Planta teórica ZOH

#### motor_teorico_zoh_node

Publica: /motor_speed_zoh_output

Se suscribe: /cmd_voltage_disc

#### dual_pid_node3

Publica: /cmd_voltage_disc

Se suscribe:

/ref_speed_sim (REFERENCIA EN rad/s)

/motor_speed_zoh_output

## 4. Diagrama general del sistema
```bash
           +----------------------+
           |   /ref_speed_sim     |  <-- referencia en rad/s
           +----------+-----------+
                      |
                      v
         +-------------------------+
         |        PID NODE        |
         | (dual_pid_node2/3)     |
         +-----------+-------------+
                     |
                     v
           /cmd_voltage_disc  (voltaje hacia la planta)
                     |
     ---------------------------------
     |                               |
     v                               v
+------------+                 +------------+
| motor_sim  |                 | motor_zoh  |
| (modelo ID)|                 | (modelo ZOH)|
+------+------+                 +------+------+
       |                               |
       v                               v
 /motor_speed_sim_disc      /motor_speed_zoh_output
```

## 5. Ejecución de los nodos

No olvides:

```bash
source ~/ros2_ws/install/setup.bash
```

### 5.1 Ejecutar el modelo identificado

```bash
ros2 run motor_digital_twin motor_sim_node
```

### 5.2 Ejecutar el PID para el modelo identificado

```bash
ros2 run motor_digital_twin pid_node2
```

### 5.3 Ejecutar el modelo teórico ZOH

```bash
ros2 run motor_digital_twin motor_teorico_zoh_node
```
### 5.4 Ejecutar el PID para el modelo ZOH
```bash
ros2 run motor_digital_twin pid_node3
```

## 6. Cómo enviar una referencia al PID (en rad/s)

El PID recibe la referencia en el tópico:

/ref_speed_sim


Enviar una referencia de 20 rad/s:

```bash
ros2 topic pub /ref_speed_sim std_msgs/msg/Float64 "{data: 20.0}"
```

Enviar una referencia de 50 rad/s de forma continua a 100 Hz:

```bash
ros2 topic pub -r 100 /ref_speed_sim std_msgs/msg/Float64 "{data: 50.0}"
```

Enviar un escalón a cero (parar el motor):

```bash
ros2 topic pub /ref_speed_sim std_msgs/msg/Float64 "{data: 0.0}"
```
## 7. Modo manual: enviar voltaje directo al motor

Ambas plantas reciben el voltaje por:

/cmd_voltage_disc


Ejemplo: enviar 3 V:

```bash
ros2 topic pub /cmd_voltage_disc std_msgs/msg/Float64 "{data: 3.0}"
```

Ejemplo: saturación negativa:

```bash
ros2 topic pub /cmd_voltage_disc std_msgs/msg/Float64 "{data: -12.0}"
```

## 8. Monitoreo

Listar tópicos:

```bash
ros2 topic list
```

Ver velocidad en planta simulada:

```bash
ros2 topic echo /motor_speed_sim_disc
```

Ver velocidad en planta ZOH:

```bash
ros2 topic echo /motor_speed_zoh_output
```
9. Archivos de registro

Los nodos generan automáticamente:

Nodo	Archivo
motor_sim_node	motor_disc_exec_times.txt
motor_teorico_zoh_node	motor_teorico_zoh_exec_times.txt
dual_pid_node2	pid2_sample_times.txt
dual_pid_node3	pid3_sample_times.txt
10. Ejecución completa del Gemelo Digital
Caso 1: modelo identificado

Terminal 1:

ros2 run motor_digital_twin motor_sim_node


Terminal 2:

ros2 run motor_digital_twin pid_node2


Terminal 3 (referencia en rad/s):

ros2 topic pub -r 100 /ref_speed_sim std_msgs/msg/Float64 "{data: 40.0}"

Caso 2: modelo teórico ZOH

Terminal 1:

ros2 run motor_digital_twin motor_teorico_zoh_node


Terminal 2:

ros2 run motor_digital_twin pid_node3


Terminal 3:

ros2 topic pub -r 100 /ref_speed_sim std_msgs/msg/Float64 "{data: 40.0}"
