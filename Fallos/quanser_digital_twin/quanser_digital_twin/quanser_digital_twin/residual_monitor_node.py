import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class EWMA:
    """Promedio y varianza exponencialmente ponderados (para z-score online)."""
    def __init__(self, alpha=0.02):
        self.alpha = float(alpha)
        self.mean = 0.0
        self.var = 1e-6
        self.initialized = False

    def update(self, x):
        if not self.initialized:
            self.mean = float(x)
            self.var = 1e-6
            self.initialized = True
            return 0.0
        a = self.alpha
        prev_mean = self.mean
        self.mean = (1 - a) * self.mean + a * x
        self.var = (1 - a) * self.var + a * (x - prev_mean) ** 2
        std = math.sqrt(max(self.var, 1e-12))
        z = (x - self.mean) / std if std > 0.0 else 0.0
        return z

class ResidualMonitorNode(Node):
    """
    Calcula residuales en rad/s:
      r_track_theo = ref_speed_rads      - meas_speed_theoretical_rads
      r_track_disc = ref_speed_disc_rads - meas_speed_discrete_rads
      r_par        = meas_speed_theoretical_rads - meas_speed_discrete_rads

    Publica:
      /health/residual/track_theoretical (Float64)
      /health/residual/track_discrete    (Float64)
      /health/residual/parity            (Float64)
      /health/residual/diagnostics       (DiagnosticArray) con z-scores y niveles (OK/WARN/ERROR)

    Umbrales (en z-score): warn_sigma, alarm_sigma (params).
    """
    def __init__(self):
        super().__init__('residual_monitor_node')

        # ===== Parámetros =====
        self.declare_parameter('dt', 0.01)
        self.declare_parameter('alpha', 0.02)        # suavizado EWMA (0.01–0.1 típico)
        self.declare_parameter('warn_sigma', 3.0)    # z-score para WARN
        self.declare_parameter('alarm_sigma', 5.0)   # z-score para ERROR

        self.dt          = float(self.get_parameter('dt').value)
        self.alpha       = float(self.get_parameter('alpha').value)
        self.warn_sigma  = float(self.get_parameter('warn_sigma').value)
        self.alarm_sigma = float(self.get_parameter('alarm_sigma').value)

        # ===== Estado =====
        self.ref_rads      = None
        self.ref_disc_rads = None
        self.meas_theo     = None
        self.meas_disc     = None

        # Filtros online por residual
        self.ewma_track_theo = EWMA(self.alpha)
        self.ewma_track_disc = EWMA(self.alpha)
        self.ewma_parity     = EWMA(self.alpha)

        # ===== Subscripciones (en rad/s canónicos) =====
        self.create_subscription(Float64, '/health/ref_speed_rads', self.cb_ref_theo, 10)
        self.create_subscription(Float64, '/health/ref_speed_disc_rads', self.cb_ref_disc, 10)
        self.create_subscription(Float64, '/health/meas_speed_theoretical_rads', self.cb_meas_theo, 10)
        self.create_subscription(Float64, '/health/meas_speed_discrete_rads', self.cb_meas_disc, 10)

        # ===== Publicadores =====
        self.pub_r_track_theo = self.create_publisher(Float64, '/health/residual/track_theoretical', 10)
        self.pub_r_track_disc = self.create_publisher(Float64, '/health/residual/track_discrete', 10)
        self.pub_r_parity     = self.create_publisher(Float64, '/health/residual/parity', 10)
        self.pub_diag         = self.create_publisher(DiagnosticArray, '/health/residual/diagnostics', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('ResidualMonitor listo: calculando residuales y z-scores.')

    # ===== Callbacks =====
    def cb_ref_theo(self, msg):  self.ref_rads = float(msg.data)
    def cb_ref_disc(self, msg):  self.ref_disc_rads = float(msg.data)
    def cb_meas_theo(self, msg): self.meas_theo = float(msg.data)
    def cb_meas_disc(self, msg): self.meas_disc = float(msg.data)

    # ===== Lógica =====
    def update(self):
        # Verifica data mínima
        have_theo = (self.ref_rads is not None) and (self.meas_theo is not None)
        have_disc = (self.ref_disc_rads is not None) and (self.meas_disc is not None)
        have_both = (self.meas_theo is not None) and (self.meas_disc is not None)

        r_track_theo = None
        r_track_disc = None
        r_par        = None

        # Residuales
        if have_theo:
            r_track_theo = self.ref_rads - self.meas_theo
            self.pub_r_track_theo.publish(Float64(data=float(r_track_theo)))
        if have_disc:
            r_track_disc = self.ref_disc_rads - self.meas_disc
            self.pub_r_track_disc.publish(Float64(data=float(r_track_disc)))
        if have_both:
            r_par = self.meas_theo - self.meas_disc
            self.pub_r_parity.publish(Float64(data=float(r_par)))

        # Diagnósticos (z-score + niveles)
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        statuses = []

        if r_track_theo is not None:
            z = self.ewma_track_theo.update(r_track_theo)
            statuses.append(self._mk_status('r_track_theoretical', r_track_theo, z))

        if r_track_disc is not None:
            z = self.ewma_track_disc.update(r_track_disc)
            statuses.append(self._mk_status('r_track_discrete', r_track_disc, z))

        if r_par is not None:
            z = self.ewma_parity.update(r_par)
            statuses.append(self._mk_status('r_parity', r_par, z))

        if statuses:
            diag.status = statuses
            self.pub_diag.publish(diag)

    def _mk_status(self, name, residual_value, zscore):
        level = DiagnosticStatus.OK
        msg = 'OK'
        az = abs(zscore)
        if az >= self.alarm_sigma:
            level = DiagnosticStatus.ERROR
            msg = 'ALARM'
        elif az >= self.warn_sigma:
            level = DiagnosticStatus.WARN
            msg = 'WARN'

        st = DiagnosticStatus()
        st.name = f"Residual::{name}"
        st.level = level
        st.message = msg
        st.values = [
            KeyValue(key='residual_value_rads', value=f"{residual_value:.6f}"),
            KeyValue(key='zscore', value=f"{zscore:.3f}"),
            KeyValue(key='warn_sigma', value=f"{self.warn_sigma}"),
            KeyValue(key='alarm_sigma', value=f"{self.alarm_sigma}"),
            KeyValue(key='alpha', value=f"{self.alpha}"),
        ]
        return st

def main(args=None):
    rclpy.init(args=args)
    node = ResidualMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
