from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_hand_ros',
            executable='tcp_leap_receiver',
            name='tcp_leap_receiver',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'host': '127.0.0.1'},
                {'port': 60002}
            ]
        ),
    ])
