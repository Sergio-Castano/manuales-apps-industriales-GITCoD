import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # Parámetros del controlador
        self.declare_parameter('kp', 5.0)
        self.declare_parameter('ki', 1.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('dt', 0.01)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # Estados del PID
        self.integral = 0.0
        self.prev_error = 0.0
        self.ref_voltage = 0.0
        self.meas_voltage = 0.0

        # Subscripciones
        self.sub_ref = self.create_subscription(Float64, '/ref_voltage', self.ref_callback, 10)
        self.sub_meas = self.create_subscription(Float64, '/motor_speed', self.meas_callback, 10)

        # Publicador de control
        self.pub_cmd = self.create_publisher(Float64, '/cmd_voltage', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.update_pid)

    def ref_callback(self, msg):
        self.ref_voltage = msg.data

    def meas_callback(self, msg):
        self.meas_voltage = msg.data

    def update_pid(self):
        error = self.ref_voltage - self.meas_voltage
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        msg_out = Float64()
        msg_out.data = control
        self.pub_cmd.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
