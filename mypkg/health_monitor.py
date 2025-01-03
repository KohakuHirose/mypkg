#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Kohaku Hirose 　　　　　
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')
        self.heart_rate_subscription = self.create_subscription(
            Float32,
            'heart_rate',
            self.heart_rate_callback,
            10
        )
        self.body_temp_subscription = self.create_subscription(
            Float32,
            'body_temperature',
            self.body_temp_callback,
            10
        )
        self.alert_publisher = self.create_publisher(String, 'health_alerts', 10)

    def heart_rate_callback(self, msg):
        heart_rate = msg.data
        if heart_rate < 60.0 or heart_rate > 100.0:
            alert_msg = String()
            alert_msg.data = f'Abnormal Heart Rate Detected: {heart_rate:.2f} bpm!'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)

    def body_temp_callback(self, msg):
        body_temp = msg.data
        if body_temp < 36.0 or body_temp > 37.5:
            alert_msg = String()
            alert_msg.data = f'Abnormal Body Temperature Detected: {body_temp:.2f} °C!'
            self.alert_publisher.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

