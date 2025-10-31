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

    # --- NUEVOS args de comodidad ---
    use_faults = LaunchConfiguration('use_faults')
    declare_use_faults = DeclareLaunchArgument(
        'use_faults', default_value='False',
        description='Si True, activa el fault_injector y remapea el PID a señales faulty'
    )

    use_refgen = LaunchConfiguration('use_refgen')
    declare_use_refgen = DeclareLaunchArgument(
        'use_refgen', default_value='True',
        description='Si True, arranca un generador de referencias para ambos lazos'
    )

    urdf_path = 'src/quanser_digital_twin/urdf/motor.urdf'
    rviz_config = 'src/quanser_digital_twin/urdf/motor.rviz'

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    robot_description = {'robot_description': robot_desc}

    # --- Nodos base de robot / visualización ---
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

    # --- Gazebo Classic ---
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

    # --- Controladores y puente ---
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

    # --- Simulaciones (teórica y discreta) ---
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

    # --- Health Mux y Residuales ---
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
            'dt': 0.01,
            'alpha': 0.02,
            'warn_sigma': 3.0,
            'alarm_sigma': 5.0,
        }],
    )

    # --- (Opcional) Generador de referencias ---
    ref_generator = Node(
        package='quanser_digital_twin',
        executable='ref_generator_node',
        name='ref_generator',
        output='screen',
        parameters=[{
            'mode': 'step',      # 'step' o 'sine'
            'amp': 1.0,          # V_equiv
            'bias': 0.0,
            'freq': 0.2,         # Hz si 'sine'
            'republish': True    # publica /ref_voltage y /ref_voltage_disc
        }],
        condition=IfCondition(use_refgen)
    )

    # --- Inyector de fallos ---
    fault_injector = Node(
        package='quanser_digital_twin',
        executable='fault_injector_node',
        name='fault_injector',
        output='screen',
        condition=IfCondition(use_faults)
    )

    # --- PID dual (adentro del launch) ---
    # Si use_faults=True, remapeamos mediciones a *_faulty. Si False, a originales.
    pid_remaps_when_faults = [
        ('/motor_speed', '/motor_speed_faulty'),
        ('/motor_speed_sim_disc', '/motor_speed_sim_disc_faulty'),
        # Si quisieras retardo de actuador, descomenta y activa actuator_delay_ms en el inyector:
        # ('/cmd_voltage', '/cmd_voltage_delayed'),
        # ('/cmd_voltage_disc', '/cmd_voltage_disc_delayed'),
    ]
    pid_remaps_normal = []  # sin remaps: consume /motor_speed y /motor_speed_sim_disc normales

    dual_pid = Node(
        package='quanser_digital_twin',
        executable='pid_controller_node',
        name='dual_pid_node',
        output='screen',
        remappings=pid_remaps_when_faults if use_faults.perform(None) == 'True' else pid_remaps_normal
    )

    # --- Orden: Gazebo -> (0.3s) RSP -> (1.0s) resto ---
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
            motor_sim_node,
            motor_sim_theoretical_node,
            health_mux_node,
            residual_monitor,
            ref_generator,
            fault_injector,
            dual_pid,
        ]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(declare_use_gazebo_classic)
    ld.add_action(declare_headless)
    ld.add_action(declare_use_faults)
    ld.add_action(declare_use_refgen)

    ld.add_action(gzserver)
    ld.add_action(gzclient)

    ld.add_action(after_rsp)
    ld.add_action(after_spawn)
    return ld
