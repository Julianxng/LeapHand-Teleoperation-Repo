#!/usr/bin/env python3

import socket
import time
import numpy as np
import rclpy
import time
from math import pi
from rclpy.node import Node
from leap_hand.leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand.leap_hand_utils.leap_hand_utils as lhu

# TCP settings
HOST = '127.0.0.1'   # Server IP
PORT = 60002         # Server port

# Motor settings
ALL_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
FLEXION_MOTOR_IDS = [2, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15]

class TCPLeapReceiver(Node):
    def __init__(self):
        super().__init__('tcp_leap_receiver')

        self.debug = False
        self.sock = None
        self.buffer = ''
        self.reconnect_interval = 5  # seconds

        self.kP = 800
        self.kI = 0
        self.kD = 200
        self.curr_lim = 500

        self.dxl_client = None
        self.connect_to_hand()
        self.setup_dynamixel()
        self.ready = False
        update_rate = 5.0 if self.debug else (1.0 / 30.0)
        self.timer = self.create_timer(update_rate, self.update_hand_position)
        self.create_timer(1.0, self.set_ready)
        #self.create_timer(2.0, self.test_motor_responses)

        


    def set_ready(self):
        self.ready = True


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
        self.dxl_client.write_desired_pos(ALL_MOTOR_IDS, np.ones(len(ALL_MOTOR_IDS)) * pi)

        print("Dynamixel setup complete")

    def test_motor_responses(self):
        if hasattr(self, '_motor_test_done'):
            return  # Don't run again
        self._motor_test_done = True

        # Define the motor IDs to exclude
        excluded_ids = {1, 8, 12}
        target_ids = [mid for mid in ALL_MOTOR_IDS if mid not in excluded_ids]

        for value in [3.0, 3.5, 4.0]:
            command = np.ones(len(target_ids)) * value
            self.dxl_client.write_desired_pos(target_ids, command)
            print(f"\n[DEBUG] Sent command {value} to motors: {target_ids}")

            time.sleep(3.0)

            pos = self.dxl_client.read_pos()
            # Show only the relevant motor positions
            target_pos = [pos[ALL_MOTOR_IDS.index(mid)] for mid in target_ids]
            print(f"[DEBUG] Read positions: {np.round(target_pos, 3)}")



    def connect_to_server(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Attempting to connect to {HOST}:{PORT}")
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((HOST, PORT))
                sock.setblocking(False)
                self.get_logger().info("Connected to TCP server.")
                return sock
            except (socket.error, ConnectionRefusedError) as e:
                self.get_logger().warn(f"Connection failed: {e}. Retrying in {self.reconnect_interval} seconds...")
                time.sleep(self.reconnect_interval)

    def update_hand_position(self):
        if self.sock is None:
            self.sock = self.connect_to_server()
            return

        try:
            data = self.sock.recv(4096).decode('utf-8')
            if self.debug:
                print("\n--- New TCP Packet ---")
                print(f"[TCP] Raw data: {data}")  

            if not data:
                self.get_logger().warn("No data received. Reconnecting...")
                self.sock.close()
                self.sock = None
                return
            self.buffer += data
            while '\n' in self.buffer:
                line, self.buffer = self.buffer.split('\n', 1)
                self.process_line(line.strip())
        except (BlockingIOError, socket.error):
            pass
        except Exception as e:
            self.get_logger().warn(f"Exception: {e}. Reconnecting...")
            if self.sock:
                self.sock.close()
            self.sock = None

    def process_line(self, line):

        try:
            landmarks = np.array([float(x) for x in line.split(',')])
            if len(landmarks) != 66:
                self.get_logger().warn(f"Received {len(landmarks)} floats, expected 60.")
                return
            landmarks = landmarks.reshape((22, 3))
            if self.debug:
                print(f"[PARSE] Parsed {len(landmarks)} floats into landmarks shape {landmarks.shape}")
                print("[PARSE] First 5 landmarks:\n", landmarks[:5])

            flexion_angles = self.compute_flexions_from_landmarks(landmarks)
            if self.debug:
                 print(f"[ANGLES] Flexion angles ({len(flexion_angles)}): {np.round(flexion_angles, 2)}")
            motor_commands = np.ones(15) * pi
            for i, motor_id in enumerate(FLEXION_MOTOR_IDS):
                index = ALL_MOTOR_IDS.index(motor_id)
                motor_commands[index] = self.flexion_deg_to_motor_command(flexion_angles[i])
                if self.debug:
                    print(f"[MOTOR MAP] motor_id={motor_id}, angle_deg={flexion_angles[i]:.2f} → motor_cmd={motor_commands[index]:.3f}")
            self.dxl_client.write_desired_pos(ALL_MOTOR_IDS, motor_commands)
            if self.debug:
                print("[COMMAND] Final motor command array:")
                print(np.round(motor_commands, 3))

        except Exception as e:
            self.get_logger().warn(f"Error processing line: {e}")

    def flexion_deg_to_motor_command(self, angle_deg):
        angle_rad = np.radians(angle_deg) + pi
        #if self.debug:
         #       print(f"[ANGLE CALC] angle_deg = {angle_deg}, angle_rad = {angle_rad}")
        return np.clip(angle_rad, 3.14, 5.14)  # Clip to your mechanical joint limit


    def angle_between(self, v1, v2):
        v1_u = v1 / (np.linalg.norm(v1) + 1e-6)
        v2_u = v2 / (np.linalg.norm(v2) + 1e-6)
        dot = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
        return np.degrees(np.arccos(dot))

    def compute_flexions_from_landmarks(self, landmarks):
        angles = []
        finger_indices = {
            'thumb': [0, 1, 2, 3],
            'index': [4, 5, 6, 7],
            'middle': [8, 9, 10, 11],
            'ring': [12, 13, 14, 15],
        }



        for name in ['index', 'middle', 'ring']:
            idx = finger_indices[name]
            vec1 = landmarks[idx[1]] - landmarks[idx[0]]
            vec2 = landmarks[idx[2]] - landmarks[idx[1]]
            vec3 = landmarks[idx[3]] - landmarks[idx[2]]
            vec4 = landmarks[idx[3]] - landmarks[idx[2]]  # Optional: can use to compare to vec3

            mcp = self.angle_between(vec1, vec2)
            pip = self.angle_between(vec2, vec3)
            dip = self.angle_between(vec3, vec4)

            if self.debug:
                print(f"\n[{name.upper()}] Landmark indices: {idx}")
                print(f"[{name.upper()}] Vectors:")
                print(f"  vec1 (MCP segment): {vec1}")
                print(f"  vec2 (PIP segment): {vec2}")
                print(f"  vec3 (DIP segment): {vec3}")
                print(f"  vec4 (DIP→TIP check): {vec4}")
                print(f"[{name.upper()}] Angles → MCP: {mcp:.2f}°, PIP: {pip:.2f}°, DIP: {dip:.2f}°")

            angles.extend([mcp, pip, dip])

        # Thumb calculation
        idx = finger_indices['thumb']
        vec1 = landmarks[idx[1]] - landmarks[idx[0]]  # CMC segment
        vec2 = landmarks[idx[2]] - landmarks[idx[1]]  # MCP segment
        vec3 = landmarks[idx[3]] - landmarks[idx[2]]  # IP segment
        vec4 = landmarks[idx[3]] - landmarks[idx[2]]  # DIP→tip (for completeness)

        cmc = self.angle_between(vec1, vec2)
        mcp = self.angle_between(vec2, vec3)
        ip = self.angle_between(vec3, vec4)

        if self.debug:
            print(f"\n[THUMB] Landmark indices: {idx}")
            print(f"[THUMB] Vectors:")
            print(f"  vec1 (CMC segment): {vec1}")
            print(f"  vec2 (MCP segment): {vec2}")
            print(f"  vec3 (IP segment): {vec3}")
            print(f"  vec4 (IP→tip check): {vec4}")
            print(f"[THUMB] Angles → CMC: {cmc:.2f}°, MCP: {mcp:.2f}°, IP: {ip:.2f}°")

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
