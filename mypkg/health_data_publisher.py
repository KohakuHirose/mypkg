#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Kohaku Hirose 　　　　　
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class HealthDataPublisher(Node):
    def __init__(self):
        super().__init__('health_data_publisher')
        self.heart_rate_publisher = self.create_publisher(Float32, 'heart_rate', 10)
        self.body_temp_publisher = self.create_publisher(Float32, 'body_temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_health_data)

    def publish_health_data(self):
        heart_rate = random.uniform(60.0, 100.0)  # Simulated heart rate
        body_temp = random.uniform(36.0, 37.5)    # Simulated body temperature
        
        if random.random() < 0.2:  # 20% の確率で異常な心拍数
            heart_rate = random.uniform(40.0, 120.0)
        if random.random() < 0.2:  # 20% の確率で異常な体温
            body_temp = random.uniform(35.0, 39.0)

        heart_rate_msg = Float32()
        heart_rate_msg.data = heart_rate
        self.heart_rate_publisher.publish(heart_rate_msg)
        
        body_temp_msg = Float32()
        body_temp_msg.data = body_temp
        self.body_temp_publisher.publish(body_temp_msg)

        self.get_logger().info(f'Published Heart Rate: {heart_rate:.2f}, Body Temperature: {body_temp:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = HealthDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
