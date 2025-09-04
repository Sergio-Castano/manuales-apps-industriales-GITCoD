import os
from glob import glob
from setuptools import setup

package_name = 'quanser_digital_twin'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),  # <- NUEVA LÍNEA
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TuNombre',
    maintainer_email='tu@email.com',
    description='Gemelo digital del motor Quanser',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_sim_node = quanser_digital_twin.motor_sim_node:main',
            'motor_sim_theoretical_node = quanser_digital_twin.motor_sim_theoretical:main',
            'joint_publisher_node = quanser_digital_twin.motor_joint_publisher:main',
            'pid_controller_node = quanser_digital_twin.pid_controller_node:main',
            'gazebo_joint_publisher_node = quanser_digital_twin.gazebo_joint_publisher:main',  # <- Añadido
        ],
    },
)
