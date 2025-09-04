# launch/motor_pid_rviz.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('quanser_digital_twin').find('quanser_digital_twin')
    pkg_gz = FindPackageShare('gazebo_ros').find('gazebo_ros')

    urdf = os.path.join(pkg, 'urdf', 'motor.urdf')
    rviz_cfg = os.path.join(pkg, 'urdf', 'motor.rviz')

    use_theoretical = LaunchConfiguration('use_theoretical')
    use_gazebo      = LaunchConfiguration('use_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('use_theoretical', default_value='true',
                              description='true=planta teórica, false=planta discreta'),
        DeclareLaunchArgument('use_gazebo', default_value='true',
                              description='Lanzar Gazebo solo como visualizador'),

        # robot_state_publisher (necesita /joint_states)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf).read()}],
            output='screen'
        ),

        # ⛔️ NO lanzar joint_state_publisher: tu nodo ya publica /joint_states

        # Nodo que integra /motor_speed y publica /joint_states
        Node(
            package='quanser_digital_twin',
            executable='joint_publisher_node',  # tu MotorJointPublisher
            name='joint_publisher_from_pid',
            output='screen'
        ),

        # PID
        Node(
            package='quanser_digital_twin',
            executable='pid_controller_node',
            name='pid_controller',
            output='screen',
            parameters=[{'kp': 5.0, 'ki': 1.0, 'kd': 0.0, 'dt': 0.01}]
        ),

        # Planta teórica  (publica /motor_speed)
        Node(
            package='quanser_digital_twin',
            executable='motor_sim_theoretical_node',
            name='motor_sim_theoretical',
            output='screen',
            parameters=[{'dt': 0.001}],
            condition=lambda context: use_theoretical.perform(context) == 'true'
        ),

        # Planta discreta (publica /motor_speed_sim) -> la remapeamos a /motor_speed
        Node(
            package='quanser_digital_twin',
            executable='motor_sim_node',
            name='motor_sim_discrete',
            output='screen',
            parameters=[{'dt': 0.01}],
            remappings=[('/motor_speed_sim', '/motor_speed')],
            condition=lambda context: use_theoretical.perform(context) == 'false'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen'
        ),

        # Gazebo clásico solo para ver el cilindro (SIN controladores)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gzserver.launch.py')),
            condition=lambda ctx: use_gazebo.perform(ctx) == 'true'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gz, 'launch', 'gzclient.launch.py')),
            condition=lambda ctx: use_gazebo.perform(ctx) == 'true'
        ),
    ])
