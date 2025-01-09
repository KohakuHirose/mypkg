#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Kohaku Hirose 　　　　　
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import random

class HealthNode(Node):
    def __init__(self):
        super().__init__('health_node')
        self.heart_rate_publisher = self.create_publisher(Float32, 'heart_rate', 10)
        self.body_temp_publisher = self.create_publisher(Float32, 'body_temperature', 10)
        self.alert_publisher = self.create_publisher(String, 'health_alerts', 10)
        self.timer = self.create_timer(1.0, self.publish_health_data)

    def publish_health_data(self):
        heart_rate = random.uniform(50.0, 110.0) 
        body_temp = random.uniform(35.0, 38.5) 

        heart_rate_msg = Float32()
        heart_rate_msg.data = heart_rate
        self.heart_rate_publisher.publish(heart_rate_msg)

        body_temp_msg = Float32()
        body_temp_msg.data = body_temp
        self.body_temp_publisher.publish(body_temp_msg)

        alert_msg = String()
        if heart_rate < 60.0 or heart_rate > 100.0:
            alert_msg.data = f'Alert: Abnormal Heart Rate: {heart_rate:.2f} bpm!'
        elif body_temp < 36.0 or body_temp > 37.5:
            alert_msg.data = f'Alert: Abnormal Body Temperature: {body_temp:.2f} °C!'
        else:
            alert_msg.data = f'Normal: Heart Rate: {heart_rate:.2f} bpm, Body Temperature: {body_temp:.2f} °C.'

        self.alert_publisher.publish(alert_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HealthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
