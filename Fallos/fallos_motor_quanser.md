
# Quanser Digital Twin

Este repositorio contiene un gemelo digital de un motor **Quanser** implementado en **ROS 2 Humble**. El proyecto incluye simulación, control PID, inyección de fallos (sensor y actuador) y monitoreo de salud.

## Requisitos

1. **ROS 2 Humble** (a partir de la versión **Humble Hawksbill**)
2. **Python 3.10**
3. **Gazebo** (opcional si quieres correr la simulación completa)
4. **Dependencias adicionales:**
   - `ros-humble-robot-state-publisher`
   - `ros-humble-joint-state-publisher`
   - `ros-humble-gazebo-ros-pkgs` (si usas Gazebo)
   - `ros-humble-ros2-control` (si quieres usar controladores)

## Instalación

1. Clona este repositorio:

   ```bash
   git clone https://github.com/tu_usuario/quanser_digital_twin.git
   cd quanser_digital_twin
   ```

2. Configura el espacio de trabajo de ROS 2

   ```bash
   mkdir -p ~/ros_ws/src
   cd ~/ros_ws/src
   git clone https://github.com/tu_usuario/quanser_digital_twin.git
   cd ~/ros_ws
   colcon build
   source install/setup.bash
   ```
   

