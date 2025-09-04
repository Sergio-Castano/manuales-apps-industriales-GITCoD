
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

3. Asegúrate de que todos los archivos URDF, parámetros, y launch estén correctamente configurados.

## Simulación y Monitoreo

1. Lanza el sistema base de simulación y monitoreo:

   ```bash
   ros2 launch quanser_digital_twin motor_rviz.launch.py
   ```

2. Activa el controlador PID del motor

   ```bash
   ros2 run quanser_digital_twin pid_controller_node
   ```

3. Publica las referencias:

    ```bash
   ros2 topic pub /ref_voltage std_msgs/Float64 "data: 1.5" -r 20
   ros2 topic pub /ref_voltage_disc std_msgs/Float64 "data: 1.5" -r 20
   ```
    
4. Ver el valor de las plantas:

   ```bash
   # Ver velocidad de la planta teórica
   ros2 topic echo /motor_speed
   
   # Ver velocidad de la planta discreta
   ros2 topic echo /motor_speed_sim_disc
   
   # Ver posición de la planta teórica
   ros2 topic echo /motor_pos
   
   # Ver posición de la planta discreta
   ros2 topic echo /motor_pos_sim_disc
   ```
   
5. Para activar un fallo de sensor
   
   A. Sesgo se sensor
   ```bash
   ros2 param set /fault_injector_node fault_type_meas sensor_bias
   ros2 param set /fault_injector_node bias_w 0.5
   ```

   Volver a la medición sana
   ```bash
   ros2 param set /fault_injector_node fault_type_meas none
   ```

   B. Sensor inmovilizado

   ```bash
   ros2 param set /fault_injector_node fault_type_meas none
   ros2 param set /fault_injector_node stuck_enable true
   ros2 param set /fault_injector_node stuck_after_s 0.5
   ```

   Volver a la medición sana
   
   ```bash  
   ros2 param set /fault_injector_node fault_type_meas none
   ros2 param set /fault_injector_node stuck_enable false
   ```

   C. Ráfagas de ruido impulsivo en el sensor
   
   ```bash
   ros2 param set /fault_injector_node fault_type_meas noise_burst
   ros2 param set /fault_injector_node burst_std 0.4
   ros2 param set /fault_injector_node burst_prob 0.2
   ```

   Volver a estado sano

   ```bash
   ros2 param set /fault_injector_node fault_type_meas none
   ```
   D. Ver estado de salud del sistema

   ```bash
   ros2 topic echo /health/residual/diagnostics
   ```

6. Para activar fallo de actuador (Debemos duplicar la planta)

   A. Activar fallo de retardo de actuador

   ```bash
   ros2 param set /fault_injector_node actuator_delay_ms 150
   ```

   B. Duplicar plantas para el retardo de actuador

   ```bash
      ros2 run quanser_digital_twin motor_sim_theoretical_node --ros-args -r /cmd_voltage:=/cmd_voltage_delayed -r /motor_speed:=/motor_speed_actd -r /motor_current:=/motor_current_actd -r /motor_pos:=/motor_pos_actd -p dt:=0.0001
   ```

   Ejecutar la planta discreta con retardo de mando
   
   ```bash
      ros2 run quanser_digital_twin motor_sim_node --ros-args -r /cmd_voltage_disc:=/cmd_voltage_disc_delayed -r /motor_speed_sim_disc:=/motor_speed_sim_disc_actd -p dt:=0.01
   ```

   C. Health mux, remapear entradas y salidas de la planta espejo para monitoreo del retardo de actuador: 

   ```bash
      ros2 run quanser_digital_twin health_mux_node --ros-args -r /motor_speed:=/motor_speed_actd -r /motor_speed_sim_disc:=/motor_speed_sim_disc_actd -r /health/ref_speed_rads:=/ahealth/ref_speed_rads -r /health/ref_speed_disc_rds:=/ahealth/ref_speed_disc_rds -r /health/meas_speed_theoretical_rads:=/ahealth/meas_speed_theoretical_rads -r /health/meas_speed_discrete_rads:=/ahealth/meas_speed_discrete_rads
   ```
   D. Residual monitor (espejo) con retardo de actuador, monitorear los residuales en la planta con retardo del actuador
   
   ```bash
      ros2 run quanser_digital_twin residual_monitor_node --ros-args -r /health/ref_speed_rads:=/ahealth/ref_speed_rads -r /health/ref_speed_disc_rads:=/ahealth/ref_speed_disc_rds -r /health/meas_speed_theoretical_rads:=/ahealth/meas_speed_theoretical_rads -r /health/meas_speed_discrete_rads:=/ahealth/meas_speed_discrete_rads -r /health/residual/track_theoretical:=/ahealth/residual/track_theoretical -r /health/residual/track_discrete:=/ahealth/residual/track_discrete -r /health/residual/parity:=/ahealth/residual/parity -r /health/residual/diagnostics:=/ahealth/residual/diagnostics -p dt:=0.01 -p alpha:=0.2 -p warn_sigma:=1.5 -p alarm_sigma:=3.0
   ```
   
   E. Ver los diagnósticos de la planta con retardeo de actuador
   
   ```bash
   ros2 topic echo /ahealth/residual/diagnostics
   ```

   F. Volver a sano (retorno sin retardo)

   
   ```bash
   ros2 param set /fault_injector_node actuator_delay_ms 0
   ```


   
