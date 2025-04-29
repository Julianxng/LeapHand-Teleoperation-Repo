from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the LEAP hand control node (hardware driver)
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0}
            ]
        ),

        # Start the TCP client node that listens to Ultraleap data
        Node(
            package='leap_hand',
            executable='tcp_leap_receiver.py',
            name='tcp_leap_receiver',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'host': '127.0.0.1'},  # TCP server address
                {'port': 60002}         # TCP server port
            ]
        ),
    ])
