from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile
import rclpy

class GazeboJointPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_joint_publisher')
        self.position = 0.0
        self.speed = 0.0
        qos = QoSProfile(depth=10)
        self.sub = self.create_subscription(Float64, '/motor_speed', self.speed_callback, qos)
        self.pub = self.create_publisher(Float64, '/model/motor_model/joint/motor_joint/cmd_pos', qos)
        self.timer = self.create_timer(0.01, self.update_joint)

    def speed_callback(self, msg):
        self.speed = msg.data * 104.72 / 1.5

    def update_joint(self):
        self.position += self.speed * 0.01
        msg = Float64()
        msg.data = self.position
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboJointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
