# dual_pid_node.py — Doble PID con la misma estructura de tu ejemplo
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class DualPIDControllerNode(Node):
    def __init__(self):
        super().__init__('dual_pid_node')

        # Parámetros comunes
        self.declare_parameter('dt', 0.01)
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # Ganancias lazo 1 (motor teórico)
        self.declare_parameter('kp1', 5.0)
        self.declare_parameter('ki1', 1.0)
        self.declare_parameter('kd1', 0.0)
        self.kp1 = self.get_parameter('kp1').get_parameter_value().double_value
        self.ki1 = self.get_parameter('ki1').get_parameter_value().double_value
        self.kd1 = self.get_parameter('kd1').get_parameter_value().double_value

        # Ganancias lazo 2 (motor discreto)
        self.declare_parameter('kp2', 5.0)
        self.declare_parameter('ki2', 1.0)
        self.declare_parameter('kd2', 0.0)
        self.kp2 = self.get_parameter('kp2').get_parameter_value().double_value
        self.ki2 = self.get_parameter('ki2').get_parameter_value().double_value
        self.kd2 = self.get_parameter('kd2').get_parameter_value().double_value

        # Estados PID lazo 1
        self.integral1 = 0.0
        self.prev_error1 = 0.0
        self.ref1 = 0.0
        self.meas1 = 0.0

        # Estados PID lazo 2
        self.integral2 = 0.0
        self.prev_error2 = 0.0
        self.ref2 = 0.0
        self.meas2 = 0.0

        # Subscripciones lazo 1
        self.sub_ref1 = self.create_subscription(Float64, '/ref_voltage', self.cb_ref1, 10)
        self.sub_meas1 = self.create_subscription(Float64, '/motor_speed', self.cb_meas1, 10)

        # Subscripciones lazo 2
        self.sub_ref2 = self.create_subscription(Float64, '/ref_voltage_disc', self.cb_ref2, 10)
        self.sub_meas2 = self.create_subscription(Float64, '/motor_speed_sim_disc', self.cb_meas2, 10)

        # Publicadores
        self.pub_u1 = self.create_publisher(Float64, '/cmd_voltage', 10)
        self.pub_u2 = self.create_publisher(Float64, '/cmd_voltage_disc', 10)

        # Timer único (misma dt para ambos lazos, igual que tu ejemplo)
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            'dual_pid_node listo:\n'
            '  L1: /ref_voltage & /motor_speed -> /cmd_voltage\n'
            '  L2: /ref_voltage_disc & /motor_speed_sim_disc -> /cmd_voltage_disc'
        )

    # --- Callbacks de referencias y mediciones ---
    def cb_ref1(self, msg):  self.ref1 = msg.data
    def cb_meas1(self, msg): self.meas1 = msg.data
    def cb_ref2(self, msg):  self.ref2 = msg.data
    def cb_meas2(self, msg): self.meas2 = msg.data

    def update(self):
        # Actualiza gains por si los modificas en runtime con ros2 param set
        self.kp1 = self.get_parameter('kp1').get_parameter_value().double_value
        self.ki1 = self.get_parameter('ki1').get_parameter_value().double_value
        self.kd1 = self.get_parameter('kd1').get_parameter_value().double_value

        self.kp2 = self.get_parameter('kp2').get_parameter_value().double_value
        self.ki2 = self.get_parameter('ki2').get_parameter_value().double_value
        self.kd2 = self.get_parameter('kd2').get_parameter_value().double_value

        dt = self.dt

        # ====== Lazo 1 ======
        error1 = self.ref1 - self.meas1
        self.integral1 += error1 * dt
        derivative1 = (error1 - self.prev_error1) / dt if dt > 0.0 else 0.0
        control1 = self.kp1 * error1 + self.ki1 * self.integral1 + self.kd1 * derivative1
        self.prev_error1 = error1
        self.pub_u1.publish(Float64(data=float(control1)))

        # ====== Lazo 2 ======
        error2 = self.ref2 - self.meas2
        self.integral2 += error2 * dt
        derivative2 = (error2 - self.prev_error2) / dt if dt > 0.0 else 0.0
        control2 = self.kp2 * error2 + self.ki2 * self.integral2 + self.kd2 * derivative2
        self.prev_error2 = error2
        self.pub_u2.publish(Float64(data=float(control2)))

def main(args=None):
    rclpy.init(args=args)
    node = DualPIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
