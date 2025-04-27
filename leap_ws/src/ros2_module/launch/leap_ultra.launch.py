from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the LEAP hand control node (your hardware driver/service)
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

        # Start your Ultraleap-based teleoperation controller
        Node(
            package='leap_hand',
            executable='leap_ultraleap_control.py',
            name='leap_ultraleap_control',
            emulate_tty=True,
            output='screen'
        ),
    ])

