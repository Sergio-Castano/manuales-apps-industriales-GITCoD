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


En este código encontrará 

7) Compilar el paquete


### Programar la interfaz analógica (Arduino Due)

8) Programar la interfaz analógica

8) Probar el nodo:
- Conecte e



## Nodo para calcular y aplicar la acción de control (Nodo Controlador)



