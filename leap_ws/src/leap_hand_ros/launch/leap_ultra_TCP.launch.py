from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Full path to URDF file
    urdf_file = os.path.join(
        get_package_share_directory("leap_hand_ros"),
        "leap_urdf",
        "robot.urdf"
    )

    # Read the contents of the URDF file
    with open(urdf_file, 'r') as infp:
        urdf_content = infp.read()

    # TCP LEAP receiver node
    tcp_node = Node(
        package='leap_hand_ros',
        executable='tcp_leap_receiver',
        name='tcp_leap_receiver',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'host': '127.0.0.1'},
            {'port': 60002}
        ]
    )

    # Robot State Publisher with URDF contents
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content}]
    )

    # RViz2
    rviz_config_file = os.path.join(
        get_package_share_directory("leap_hand_ros"),
        "rviz",
        "leap_hand.rviz"
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        tcp_node,
        robot_state_publisher_node,
        rviz_node
    ])
