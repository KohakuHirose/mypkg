from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',  # Replace with your package name
            executable='health_data_publisher',  # Entry point for publisher node
            name='health_data_publisher',
            output='screen',
        ),
        Node(
            package='mypkg',  # Replace with your package name
            executable='health_monitor',  # Entry point for subscriber node
            name='health_monitor',
            output='screen',
        ),
    ])

