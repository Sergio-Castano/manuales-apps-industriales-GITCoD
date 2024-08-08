# Caso 1: Control distribuido usando Raspberry Pi y DDS (ROS2) 
A continuación se presentan los pasos para implementar un lazo de control distribuido usando **dos (2)** Raspberry Pi 4 como nodos de hardware, las cuales han de contrar con una versión de kernel parcheado con PREEMPT_RT y contenedores Singularity para utilizar el middleware DDS integrado en ROS2 (Versión Humble). Con lo cual deberá haber seguido los procedimientos presentados en los anexos para tener parcheado el SO, haber instalado el paquete WiringPi, tener instalado Singularity y saber cómo crear un contenedor con ROS2.

## Nodo para Lectura de Variable (Nodo Sensor)

1) En una de las Raspberry Pi, cree un sandbox de Singularity con ROS2 y nómbrelo como ROS2_sensor_node.


3) Inicie un contenedor para editar la ngrese a la imagen  

## Nodo para calcular y aplicar la acción de control (Nodo Controlador)



