# combined_launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # --- Args compartidos ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    use_gazebo_classic = LaunchConfiguration('use_gazebo_classic')
    headless = LaunchConfiguration('headless')

    declare_use_gazebo_classic = DeclareLaunchArgument(
        'use_gazebo_classic', default_value='True',
        description='Lanzar Gazebo Classic (gzserver/gzclient)'
    )
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='False',
        description='Si True, no abre gzclient'
    )

    urdf_path = 'src/quanser_digital_twin/urdf/motor.urdf'
    rviz_config = 'src/quanser_digital_twin/urdf/motor.rviz'

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    robot_description = {'robot_description': robot_desc}

    # --- Nodos tuyos (Launch 1) ---
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    joint_pub = Node(
        package='quanser_digital_twin',
        executable='joint_publisher_node',
        name='joint_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    gazebo_joint_pub = Node(
        package='quanser_digital_twin',
        executable='gazebo_joint_publisher_node',
        name='gazebo_joint_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # --- Gazebo Classic (Launch 1) ---
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose',
             '-slibgazebo_ros_init.so',
             '-slibgazebo_ros_factory.so'],
        output='screen',
        condition=IfCondition(use_gazebo_classic)
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(PythonExpression([use_gazebo_classic, ' and not ', headless]))
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_motor_robot',
        arguments=[
            '-entity', 'motor_robot',
            '-file', urdf_path,
            '-x', '0', '-y', '0', '-z', '0.3', '-Y', '0'
        ],
        output='screen',
        condition=IfCondition(use_gazebo_classic)
    )

    # --- Controladores y puente (Launch 1) ---
    load_jsb = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    load_vel = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )
    speed_bridge = Node(
        package='quanser_digital_twin',
        executable='speed_to_cmd',
        name='speed_to_cmd',
        output='screen'
    )

    # --- Nodos de simulación (Launch 2 y Launch 3, sin cambios) ---
    motor_sim_node = Node(
        package='quanser_digital_twin',
        executable='motor_sim_node',
        name='motor_sim_node',
        output='screen',
        parameters=[{'dt': 0.01}]
    )

    motor_sim_theoretical_node = Node(
        package='quanser_digital_twin',
        executable='motor_sim_theoretical_node',
        name='motor_sim_theoretical_node',
        output='screen',
        parameters=[{'dt': 0.0001}]
    )

    health_mux_node = Node(
        package='quanser_digital_twin',
        executable='health_mux_node',
        name='health_mux_node',
        output='screen'
    )

    residual_monitor = Node(
        package='quanser_digital_twin',
        executable='residual_monitor_node',
        name='residual_monitor',
        output='screen',
        parameters=[{
            'dt': 0.01,        # igual a tu loop
            'alpha': 0.02,     # suavizado del z-score
            'warn_sigma': 3.0,
            'alarm_sigma': 5.0,
        }],
    )
    # … y agrégalo en actions junto con health_mux_node

    # --- NUEVO: Inyector de fallos (agregado sin tocar nada más) ---
    fault_injector = Node(
        package='quanser_digital_twin',
        executable='fault_injector_node',
        name='fault_injector_node',
        output='screen'
        # Parámetros opcionales los puedes pasar por ros2 param set en runtime
    )

    # --- Orden original base: Gazebo -> (0.3s) RSP -> (1.0s) spawn + controladores + resto ---
    after_rsp = TimerAction(period=0.3, actions=[state_pub])
    after_spawn = TimerAction(
        period=1.0,
        actions=[
            spawn_entity,
            load_jsb,
            load_vel,
            joint_pub,
            gazebo_joint_pub,
            rviz,
            speed_bridge,
            # Añadidos de Launch 2 y Launch 3:
            motor_sim_node,
            motor_sim_theoretical_node
        ]
    )

    # --- LaunchDescription ---
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(declare_use_gazebo_classic)
    ld.add_action(declare_headless)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(health_mux_node)
    ld.add_action(residual_monitor)
    ld.add_action(fault_injector)   # <--- añadido
    ld.add_action(after_rsp)
    ld.add_action(after_spawn)
    
    return ld
