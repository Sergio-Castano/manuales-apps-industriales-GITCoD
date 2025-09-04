import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import math

class MotorJointPublisher(Node):
    def __init__(self):
        super().__init__('motor_joint_publisher')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Float64, '/motor_speed', self.speed_callback, 10)
        self.timer = self.create_timer(0.01, self.update_joint)
        self.speed = 0.0
        self.position = 0.0

    def speed_callback(self, msg):
        # Convierte voltaje a rad/s: 1.5 V = 1000 rpm ≈ 104.72 rad/s
        self.speed = msg.data * 104.72 / 1.5

    def update_joint(self):
        self.position += self.speed * 0.01  # Integración simple
        joint_msg = JointState()
        joint_msg.name = ['motor_joint']
        joint_msg.position = [self.position]
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
