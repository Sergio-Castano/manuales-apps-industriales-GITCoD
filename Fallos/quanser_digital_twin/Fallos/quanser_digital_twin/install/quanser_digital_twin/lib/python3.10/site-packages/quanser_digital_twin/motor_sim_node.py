import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorSimNode(Node):
    def __init__(self):
        super().__init__('motor_sim_node')
        self.declare_parameter('dt', 0.01)
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        self.sub = self.create_subscription(Float64, '/cmd_voltage', self.cmd_callback, 10)
        self.pub = self.create_publisher(Float64, '/motor_speed_sim', 10)
        self.timer = self.create_timer(self.dt, self.update)

        # Variables del modelo discretizado
        self.u = 0.0
        self.y = [0.0, 0.0]       # Salida actual y pasada
        self.u_hist = [0.0, 0.0]  # Entrada actual y pasada

        # Discretización por ZOH (tm = 0.01 s)
        # P(z) = (0.03829 z + 0.01635) / (z^2 - 1.003 z + 0.06946)
        self.a1 = -1.003     # Cuidado: negativos en la implementación
        self.a2 = 0.06946
        self.b0 = 0.03829
        self.b1 = 0.01635

    def cmd_callback(self, msg):
        self.u = msg.data

    def update(self):
        # y[k] = -a1*y[k-1] - a2*y[k-2] + b0*u[k] + b1*u[k-1]
        y_new = -self.a1 * self.y[0] - self.a2 * self.y[1] + self.b0 * self.u + self.b1 * self.u_hist[0]

        self.y[1] = self.y[0]
        self.y[0] = y_new

        self.u_hist[1] = self.u_hist[0]
        self.u_hist[0] = self.u

        msg_out = Float64()
        msg_out.data = y_new
        self.pub.publish(msg_out)

        self.get_logger().debug(f'u={self.u:.3f}, y={y_new:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorSimNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
