import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TypingSpeedSubscriber(Node):
    def __init__(self):
        super().__init__('typing_speed_subscriber')
        self.subscription = self.create_subscription(
            String,
            'typing_task',
            self.task_callback,
            10
        )
        self.start_time = None
        self.current_task = None
        self.get_logger().info('Typing Speed Subscriber started.')

    def task_callback(self, msg):
        self.current_task = msg.data
        self.start_time = time.time()
        self.get_logger().info(f'Received typing task: "{self.current_task}"')
        print("\nPlease type the following sentence as fast as you can:")
        print(self.current_task)
        print("Type below and press Enter:")

        user_input = input("> ")
        self.check_typing_speed(user_input)

    def check_typing_speed(self, user_input):
        if self.current_task is None or self.start_time is None:
            self.get_logger().warn("No task received or timing not started.")
            return

        end_time = time.time()
        time_taken = end_time - self.start_time

        if user_input.strip() == self.current_task.strip():
            typing_speed = len(user_input) / time_taken
            self.get_logger().info(
                f"Correct typing! Time taken: {time_taken:.2f} seconds. "
                f"Typing speed: {typing_speed:.2f} characters per second."
            )
        else:
            self.get_logger().warn("Incorrect typing. Please try again.")

def main(args=None):
    rclpy.init(args=args)
    node = TypingSpeedSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

