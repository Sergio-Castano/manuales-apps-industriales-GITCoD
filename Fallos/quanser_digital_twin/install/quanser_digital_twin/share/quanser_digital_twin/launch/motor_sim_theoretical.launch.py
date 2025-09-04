from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quanser_digital_twin',
            executable='motor_sim_theoretical_node',
            name='motor_sim_theoretical_node',
            output='screen',
            parameters=[{'dt': 0.0001}]
        )
    ])
