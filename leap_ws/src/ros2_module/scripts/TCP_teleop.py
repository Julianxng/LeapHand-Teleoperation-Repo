#!/usr/bin/env python3

import socket
import numpy as np
import rclpy
from rclpy.node import Node
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

# TCP settings
HOST = '127.0.0.1'   # server IP
PORT = 60002         # server port

# Motor settings
JOINT_LIMITS = (0.0, 2.0)
ALL_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
FLEXION_MOTOR_IDS = [2, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15]

class TCPLeapReceiver(Node):
    def __init__(self):
        super().__init__('tcp_leap_receiver')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((HOST, PORT))
        self.sock.setblocking(True)

        self.kP = 800
        self.kI = 0
        self.kD = 200
        self.curr_lim = 500

        self.dxl_client = None
        self.connect_to_hand()
        self.setup_dynamixel()

        self.timer = self.create_timer(1.0 / 30.0, self.update_hand_position)

    def connect_to_hand(self):
        try:
            self.dxl_client = DynamixelClient(ALL_MOTOR_IDS, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(ALL_MOTOR_IDS, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(ALL_MOTOR_IDS, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

    def setup_dynamixel(self):
        self.dxl_client.sync_write(ALL_MOTOR_IDS, np.ones(len(ALL_MOTOR_IDS)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(ALL_MOTOR_IDS, True)
        self.dxl_client.sync_write(ALL_MOTOR_IDS, np.ones(len(ALL_MOTOR_IDS)) * self.kP, 84, 2)
        self.dxl_client.sync_write([1, 8], np.ones(2) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(ALL_MOTOR_IDS, np.ones(len(ALL_MOTOR_IDS)) * self.kI, 82, 2)
        self.dxl_client.sync_write(ALL_MOTOR_IDS, np.ones(len(ALL_MOTOR_IDS)) * self.kD, 80, 2)
        self.dxl_client.sync_write([1, 8], np.ones(2) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(ALL_MOTOR_IDS, np.ones(len(ALL_MOTOR_IDS)) * self.curr_lim, 102, 2)

    def update_hand_position(self):
        try:
            data = self.sock.recv(4096).decode('utf-8').strip()
            if not data:
                return

            landmarks = np.array([float(x) for x in data.split(',')])
            if len(landmarks) != 60:
                self.get_logger().warn(f"Received {len(landmarks)} floats, expected 60.")
                return

            # landmarks: 20 points × 3 (x, y, z)
            landmarks = landmarks.reshape((20, 3))

            flexion_angles = self.compute_flexions_from_landmarks(landmarks)

            motor_commands = np.zeros(15)
            for i, motor_id in enumerate(FLEXION_MOTOR_IDS):
                index = ALL_MOTOR_IDS.index(motor_id)
                motor_commands[index] = self.flexion_deg_to_leap_radians(flexion_angles[i])

            self.dxl_client.write_desired_pos(ALL_MOTOR_IDS, motor_commands)

        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")

    def flexion_deg_to_leap_radians(self, angle_deg):
        max_flexion_deg = 180.0
        norm = np.clip((max_flexion_deg - angle_deg) / max_flexion_deg, 0.0, 1.0)
        return norm * JOINT_LIMITS[1]

    def angle_between(self, v1, v2):
        v1_u = v1 / (np.linalg.norm(v1) + 1e-6)
        v2_u = v2 / (np.linalg.norm(v2) + 1e-6)
        dot = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
        return np.degrees(np.arccos(dot))

    def compute_flexions_from_landmarks(self, landmarks):
        angles = []

        # Example finger mappings based on landmark index:
        # You will have to adapt this based on which landmarks are thumb/index/middle/ring/pinky
        finger_indices = {
            'index': [5, 6, 7, 8],
            'middle': [9, 10, 11, 12],
            'ring': [13, 14, 15, 16],
            'thumb': [1, 2, 3, 4],
        }

        for name in ['index', 'middle', 'ring']:
            idx = finger_indices[name]
            vec1 = landmarks[idx[1]] - landmarks[idx[0]]
            vec2 = landmarks[idx[2]] - landmarks[idx[1]]
            vec3 = landmarks[idx[3]] - landmarks[idx[2]]

            mcp = self.angle_between(vec1, vec2)
            pip = self.angle_between(vec2, vec3)
            dip = 0.0

            angles.extend([mcp, pip, dip])

        # Thumb (slightly different kinematics)
        idx = finger_indices['thumb']
        vec1 = landmarks[idx[1]] - landmarks[idx[0]]
        vec2 = landmarks[idx[2]] - landmarks[idx[1]]
        vec3 = landmarks[idx[3]] - landmarks[idx[2]]

        cmc = self.angle_between(vec1, vec2)
        mcp = self.angle_between(vec2, vec3)
        ip = 0.0

        angles.extend([cmc, mcp, ip])

        return angles

def main(args=None):
    rclpy.init(args=args)
    node = TCPLeapReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
