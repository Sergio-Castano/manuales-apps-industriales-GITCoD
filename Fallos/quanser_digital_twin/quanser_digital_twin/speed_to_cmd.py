import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray

class SpeedToCmd(Node):
    def __init__(self):
        super().__init__('speed_to_cmd')
        self.k_v_to_rad = 104.72 / 1.5  # 1.5 V -> 1000 rpm ~ 104.72 rad/s
        self.sub = self.create_subscription(Float64, '/motor_speed', self.cb, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
    def cb(self, msg: Float64):
        out = Float64MultiArray()
        out.data = [msg.data * self.k_v_to_rad]
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SpeedToCmd())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 