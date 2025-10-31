import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MotorSimTheoreticalNode(Node):
    def __init__(self):
        super().__init__('motor_sim_theoretical_node')

        # Constantes físicas (motor Quanser)
        self.J = 1.16e-6      # kg·m²
        self.b = 6.82e-6      # N·m·s/rad
        self.Kt = 0.0069       # Nm/A
        self.Ke = 0.0069       # V·s/rad
        self.R = 10.6         # Ohmios
        self.L = 0.00082      # Henrys

        # Variables de estado
        self.i = 0.0          # Corriente (A)
        self.omega = 0.0      # Velocidad angular (rad/s)
        self.theta = 0.0      # Posición angular (rad)
        self.V = 0.0          # Voltaje aplicado (V)

        # Tiempo de muestreo
        self.declare_parameter('dt', 0.001)
        self.dt = self.get_parameter('dt').get_parameter_value().double_value

        # Subscripción al voltaje de entrada
        self.sub = self.create_subscription(Float64, '/cmd_voltage', self.voltage_callback, 10)

        # Publicadores
        self.pub_i = self.create_publisher(Float64, '/motor_current', 10)
        self.pub_omega = self.create_publisher(Float64, '/motor_speed', 10)  # ahora en voltios simulados
        self.pub_theta = self.create_publisher(Float64, '/motor_pos', 10)

        # Timer de simulación
        self.timer = self.create_timer(self.dt, self.update_model)

    def voltage_callback(self, msg):
        self.V = msg.data

    def update_model(self):
        # Derivadas diferenciales
        di_dt = (1 / self.L) * (self.V - self.R * self.i - self.Ke * self.omega)
        domega_dt = (self.Kt / self.J) * self.i - (self.b / self.J) * self.omega

        # Integración Euler
        self.i += di_dt * self.dt
        self.omega += domega_dt * self.dt
        self.theta += self.omega * self.dt

        # Publicar corriente
        msg_i = Float64()
        msg_i.data = self.i
        self.pub_i.publish(msg_i)

        # Publicar velocidad convertida a voltaje (1.5 V ↔ 1000 RPM ↔ 104.72 rad/s)
        msg_omega = Float64()
        msg_omega.data = self.omega / 69.81
        self.pub_omega.publish(msg_omega)

        # Publicar posición
        msg_theta = Float64()
        msg_theta.data = self.theta
        self.pub_theta.publish(msg_theta)

def main(args=None):
    rclpy.init(args=args)
    node = MotorSimTheoreticalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
