import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from collections import deque
import time
import random

class FaultInjectorNode(Node):
    """
    Inyecta fallos en:
      - mediciones: /motor_speed -> /motor_speed_faulty
                    /motor_speed_sim_disc -> /motor_speed_sim_disc_faulty
      - actuadores (opcional): /cmd_voltage -> /cmd_voltage_delayed
                               /cmd_voltage_disc -> /cmd_voltage_disc_delayed

    Parámetros (dinámicos con ros2 param set):
      fault_type_meas ∈ {none, sensor_bias, sensor_stuck, noise_burst}
      bias_w (float)           # rad/s equivalentes en V_eq*gain (aplicado sobre V_eq)
      burst_std (float)        # std del ruido impulsivo (V_eq)
      burst_prob (float)       # probabilidad de impulso por muestra (0..1)
      stuck_enable (bool)
      stuck_after_s (float)    # tiempo (s) tras arrancar para congelar lectura

      actuator_delay_ms (int)  # retardo FIFO en ms para cmd (0 = sin retardo)
    """
    def __init__(self):
        super().__init__('fault_injector_node')

        # ---- params medición ----
        self.declare_parameter('fault_type_meas', 'none')
        self.declare_parameter('bias_w', 0.0)
        self.declare_parameter('burst_std', 0.0)
        self.declare_parameter('burst_prob', 0.0)
        self.declare_parameter('stuck_enable', False)
        self.declare_parameter('stuck_after_s', 0.0)

        # ---- params actuador ----
        self.declare_parameter('actuator_delay_ms', 0)

        # estado interno
        self.t0 = time.time()
        self.stuck_value_theo = None
        self.stuck_value_disc = None

        # subs originales (V_eq en tus nodos)
        self.sub_meas_theo = self.create_subscription(Float64, '/motor_speed', self.cb_meas_theo, 10)
        self.sub_meas_disc = self.create_subscription(Float64, '/motor_speed_sim_disc', self.cb_meas_disc, 10)
        self.sub_cmd_theo  = self.create_subscription(Float64, '/cmd_voltage', self.cb_cmd_theo, 10)
        self.sub_cmd_disc  = self.create_subscription(Float64, '/cmd_voltage_disc', self.cb_cmd_disc, 10)

        # pubs “faulty”
        self.pub_meas_theo_faulty = self.create_publisher(Float64, '/motor_speed_faulty', 10)
        self.pub_meas_disc_faulty = self.create_publisher(Float64, '/motor_speed_sim_disc_faulty', 10)
        self.pub_cmd_theo_delayed = self.create_publisher(Float64, '/cmd_voltage_delayed', 10)
        self.pub_cmd_disc_delayed = self.create_publisher(Float64, '/cmd_voltage_disc_delayed', 10)

        # colas para retardo de actuador
        self.fifo_cmd_theo = deque()
        self.fifo_cmd_disc = deque()
        self.timer = self.create_timer(0.001, self.tick_delay)  # 1 kHz para drenar colas

        self.get_logger().info("fault_injector_node listo.")

    # -------- medición con fallos (sobre V_eq) ----------
    def apply_meas_fault(self, val, branch='theo'):
        ft = self.get_parameter('fault_type_meas').get_parameter_value().string_value
        bias = float(self.get_parameter('bias_w').value)
        burst_std = float(self.get_parameter('burst_std').value)
        burst_prob = float(self.get_parameter('burst_prob').value)
        stuck_enable = bool(self.get_parameter('stuck_enable').value)
        stuck_after_s = float(self.get_parameter('stuck_after_s').value)
        t = time.time() - self.t0

        x = float(val)

        if stuck_enable and t >= stuck_after_s:
            if branch == 'theo':
                if self.stuck_value_theo is None:
                    self.stuck_value_theo = x
                return self.stuck_value_theo
            else:
                if self.stuck_value_disc is None:
                    self.stuck_value_disc = x
                return self.stuck_value_disc

        if ft == 'sensor_bias':
            x = x + bias
        elif ft == 'noise_burst':
            if random.random() < burst_prob:
                x = x + random.gauss(0.0, burst_std)
        # ft == 'none' o 'sensor_stuck' ya manejado arriba

        return x

    def cb_meas_theo(self, msg):
        y = self.apply_meas_fault(msg.data, 'theo')
        self.pub_meas_theo_faulty.publish(Float64(data=y))

    def cb_meas_disc(self, msg):
        y = self.apply_meas_fault(msg.data, 'disc')
        self.pub_meas_disc_faulty.publish(Float64(data=y))

    # -------- actuador con retardo ----------
    def cb_cmd_theo(self, msg):
        delay_ms = int(self.get_parameter('actuator_delay_ms').value)
        if delay_ms <= 0:
            self.pub_cmd_theo_delayed.publish(msg)
        else:
            self.fifo_cmd_theo.append((time.time() + delay_ms/1000.0, float(msg.data)))

    def cb_cmd_disc(self, msg):
        delay_ms = int(self.get_parameter('actuator_delay_ms').value)
        if delay_ms <= 0:
            self.pub_cmd_disc_delayed.publish(msg)
        else:
            self.fifo_cmd_disc.append((time.time() + delay_ms/1000.0, float(msg.data)))

    def tick_delay(self):
        now = time.time()
        while self.fifo_cmd_theo and self.fifo_cmd_theo[0][0] <= now:
            _, u = self.fifo_cmd_theo.popleft()
            self.pub_cmd_theo_delayed.publish(Float64(data=u))
        while self.fifo_cmd_disc and self.fifo_cmd_disc[0][0] <= now:
            _, u = self.fifo_cmd_disc.popleft()
            self.pub_cmd_disc_delayed.publish(Float64(data=u))

def main(args=None):
    rclpy.init(args=args)
    n = FaultInjectorNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
