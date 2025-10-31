import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class GazeboJointPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_joint_publisher')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(Float64, '/motor_speed', self.speed_callback, 10)
        self.timer = self.create_timer(0.01, self.update_joint)
        self.speed = 0.0
        self.position = 0.0

    def speed_callback(self, msg):
        self.speed = msg.data * 104.72 / 1.5  # voltaje a rad/s

    def update_joint(self):
        self.position += self.speed * 0.01
        joint_msg = JointState()
        joint_msg.name = ['motor_joint']
        joint_msg.position = [self.position]
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
