#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import Leap  # LeapC Python bindings must be installed

from calc_flexion_TCP import calculate_hand_joint_angles


class UltraleapTeleopNode(Node):
    def __init__(self):
        super().__init__('ultraleap_teleop')
        self.publisher_ = self.create_publisher(JointState, 'cmd_leap', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)  # 30 Hz

        self.controller = Leap.Controller()
        self.controller.set_policy(Leap.Controller.POLICY_IMAGES)

        self.joint_names = [
            'thumb_mcp', 'thumb_pip', 'thumb_dip',
            'index_mcp', 'index_pip', 'index_dip',
            'middle_mcp', 'middle_pip', 'middle_dip',
            'ring_mcp', 'ring_pip', 'ring_dip'
        ]

        self.get_logger().info("Ultraleap teleop node initialized and running.")

    def timer_callback(self):
        frame = self.controller.frame()
        if frame.hands.is_empty:
            self.get_logger().info("No hands detected.")
            return

        hand = frame.hands[0]  # Use the first hand detected
        hand_data = self.extract_hand_data(hand)

        try:
            joint_angles = calculate_hand_joint_angles(hand_data)
            if len(joint_angles) != len(self.joint_names):
                self.get_logger().warn("Mismatch between joint angles and joint names.")
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = joint_angles

            self.publisher_.publish(msg)
            self.get_logger().info(f"Published {len(joint_angles)} joint angles.")

        except Exception as e:
            self.get_logger().error(f"Failed to compute joint angles: {e}")

    def extract_hand_data(self, hand):
        hand_data = {}
        for finger in hand.fingers:
            if finger.type == Leap.Finger.TYPE_PINKY:
                continue  # Skip pinky

            finger_name = self.leap_finger_name(finger.type)
            bones = []
            for b in range(4):
                bone = finger.bone(b)
                bones.append({
                    'prev_joint': [bone.prev_joint.x, bone.prev_joint.y, bone.prev_joint.z],
                    'next_joint': [bone.next_joint.x, bone.next_joint.y, bone.next_joint.z]
                })
            hand_data[finger_name] = bones
        return hand_data

    @staticmethod
    def leap_finger_name(finger_type):
        return {
            Leap.Finger.TYPE_THUMB: 'thumb',
            Leap.Finger.TYPE_INDEX: 'index',
            Leap.Finger.TYPE_MIDDLE: 'middle',
            Leap.Finger.TYPE_RING: 'ring'
        }.get(finger_type, 'unknown')


def main(args=None):
    rclpy.init(args=args)
    node = UltraleapTeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

