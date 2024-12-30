import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TypingSpeedPublisher(Node):
    def __init__(self):
        super().__init__('typing_speed_publisher')
        self.publisher_ = self.create_publisher(String, 'typing_task', 10)
        self.timer = self.create_timer(10.0, self.publish_task)
        self.get_logger().info('Typing Speed Publisher started.')

    def publish_task(self):
        task = "The quick brown fox jumps over the lazy dog."
        msg = String()
        msg.data = task
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published typing task: "{task}"')

def main(args=None):
    rclpy.init(args=args)
    node = TypingSpeedPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

