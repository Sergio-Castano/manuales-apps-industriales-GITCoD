import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/investigacion/ros2_ws/src/quanser_digital_twin/install/quanser_digital_twin'
