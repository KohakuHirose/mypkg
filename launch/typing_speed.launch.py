from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mypkg',  # 自分のパッケージ名に変更
            executable='typing_task_publisher',  # パブリッシャーのエントリポイント
            name='typing_speed_publisher',  # ノード名
        ),
        Node(
            package='mypkg',  # 自分のパッケージ名に変更
            executable='typing_speed_subscriber',  # サブスクライバーのエントリポイント
            name='typing_speed_subscriber',  # ノード名
            output='screen',  # サブスクライバーの出力をターミナルに表示
        ),
    ])

