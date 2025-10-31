import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class HealthMuxNode(Node):
    def __init__(self):
        super().__init__('health_mux_node')

        # Ganancias de conversión (V_eq -> rad/s)
        self.declare_parameter('theoretical_gain_to_rads', 69.81)
        self.k_theo_to_rads = float(self.get_parameter('theoretical_gain_to_rads').value)

        self.declare_parameter('disc_is_v_eq', True)
        self.disc_is_v_eq = bool(self.get_parameter('disc_is_v_eq').value)

        self.declare_parameter('disc_gain_to_rads', 69.81)
        self.k_disc_to_rads = float(self.get_parameter('disc_gain_to_rads').value)

        # --- Subscripciones originales ---
        self.create_subscription(Float64, '/ref_voltage', self.cb_ref_theo, 10)
        self.create_subscription(Float64, '/ref_voltage_disc', self.cb_ref_disc, 10)
        self.create_subscription(Float64, '/motor_speed', self.cb_meas_theo, 10)            # V_eq
        self.create_subscription(Float64, '/motor_speed_sim_disc', self.cb_meas_disc, 10)   # V_eq
        self.create_subscription(Float64, '/cmd_voltage', self.cb_cmd_theo, 10)
        self.create_subscription(Float64, '/cmd_voltage_disc', self.cb_cmd_disc, 10)
        self.create_subscription(Float64, '/motor_current', self.cb_current, 10)

        # --- Publicadores /health en V_eq (compatibilidad) ---
        self.pub_ref_speed       = self.create_publisher(Float64, '/health/ref_speed', 10)
        self.pub_ref_speed_disc  = self.create_publisher(Float64, '/health/ref_speed_disc', 10)
        self.pub_meas_theo       = self.create_publisher(Float64, '/health/meas_speed_theoretical', 10)
        self.pub_meas_disc       = self.create_publisher(Float64, '/health/meas_speed_discrete', 10)
        self.pub_cmd_theo        = self.create_publisher(Float64, '/health/cmd_voltage_theoretical', 10)
        self.pub_cmd_disc        = self.create_publisher(Float64, '/health/cmd_voltage_discrete', 10)
        self.pub_current         = self.create_publisher(Float64, '/health/motor_current', 10)

        # --- NUEVO: referencias canónicas (rad/s y rpm) ---
        self.pub_ref_rads        = self.create_publisher(Float64, '/health/ref_speed_rads', 10)
        self.pub_ref_rpm         = self.create_publisher(Float64, '/health/ref_speed_rpm', 10)
        self.pub_ref_disc_rads   = self.create_publisher(Float64, '/health/ref_speed_disc_rads', 10)
        self.pub_ref_disc_rpm    = self.create_publisher(Float64, '/health/ref_speed_disc_rpm', 10)

        # --- Mediciones canónicas (rad/s y rpm) ---
        self.pub_meas_theo_rads  = self.create_publisher(Float64, '/health/meas_speed_theoretical_rads', 10)
        self.pub_meas_disc_rads  = self.create_publisher(Float64, '/health/meas_speed_discrete_rads', 10)
        self.pub_meas_theo_rpm   = self.create_publisher(Float64, '/health/meas_speed_theoretical_rpm', 10)
        self.pub_meas_disc_rpm   = self.create_publisher(Float64, '/health/meas_speed_discrete_rpm', 10)

        self.get_logger().info("HealthMuxNode listo: refs y medidas en V_eq + rad/s (+ rpm).")

    @staticmethod
    def rads_to_rpm(w_rads: float) -> float:
        return w_rads * (60.0 / (2.0 * math.pi))  # ≈ 9.549296

    # --- Callbacks de paso/V_eq ---
    def cb_cmd_theo(self, msg): self.pub_cmd_theo.publish(msg)
    def cb_cmd_disc(self, msg): self.pub_cmd_disc.publish(msg)
    def cb_current(self, msg):  self.pub_current.publish(msg)

    # --- NUEVO: convertir y publicar las REFERENCIAS ---
    def cb_ref_theo(self, msg):
        # V_eq original (compat)
        self.pub_ref_speed.publish(msg)
        # rad/s + rpm
        w = float(msg.data) * self.k_theo_to_rads
        self.pub_ref_rads.publish(Float64(data=w))
        self.pub_ref_rpm.publish(Float64(data=self.rads_to_rpm(w)))

    def cb_ref_disc(self, msg):
        # V_eq original (compat)
        self.pub_ref_speed_disc.publish(msg)
        # rad/s + rpm
        w = float(msg.data) * (self.k_disc_to_rads if self.disc_is_v_eq else 1.0)
        self.pub_ref_disc_rads.publish(Float64(data=w))
        self.pub_ref_disc_rpm.publish(Float64(data=self.rads_to_rpm(w)))

    # --- Mediciones: convertir a rad/s + rpm ---
    def cb_meas_theo(self, msg):
        self.pub_meas_theo.publish(msg)
        w = float(msg.data) * self.k_theo_to_rads
        self.pub_meas_theo_rads.publish(Float64(data=w))
        self.pub_meas_theo_rpm.publish(Float64(data=self.rads_to_rpm(w)))

    def cb_meas_disc(self, msg):
        self.pub_meas_disc.publish(msg)
        w = float(msg.data) * (self.k_disc_to_rads if self.disc_is_v_eq else 1.0)
        self.pub_meas_disc_rads.publish(Float64(data=w))
        self.pub_meas_disc_rpm.publish(Float64(data=self.rads_to_rpm(w)))

def main(args=None):
    rclpy.init(args=args)
    node = HealthMuxNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
