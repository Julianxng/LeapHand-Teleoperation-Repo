#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu
from leap_hand.srv import LeapPosition, LeapVelocity, LeapEffort, LeapPosVelEff

from leap_teleop import compute_all_flexion_angles_from_leap  # Import the real Ultraleap integration

JOINT_LIMITS = (0.0, 2.0)  # min, max radians
HOME_POSITION = lhu.allegro_to_LEAPhand(np.zeros(16))[:15]  # Use first 15 motors

# Flexion-only motor IDs (excluding abduction) and their indexes
FLEXION_MOTOR_IDS = [2, 3, 4, 5, 6, 7, 9, 10, 11, 13, 14, 15]
FLEXION_INDEXES = list(range(len(FLEXION_MOTOR_IDS)))

# Full motor list (used for writing 15 motor positions)
ALL_MOTOR_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]

# Convert Ultraleap angle to radians for the LEAP hand
def flexion_deg_to_leap_radians(angle_deg):
    max_flexion_deg = 180.0
    norm = np.clip((max_flexion_deg - angle_deg) / max_flexion_deg, 0.0, 1.0)
    return norm * JOINT_LIMITS[1]

class LeapTeleopController(Node):
    def __init__(self):
        super().__init__('leap_ultraleap_control')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.port = self.get_parameter('port').get_parameter_value().string_value

        self.dxl_client = None
        self.connect_to_hand()

        self.kP = 800
        self.kI = 0
        self.kD = 200
        self.curr_lim = 500

        self.setup_dynamixel()

        self.prev_angles = self.curr_angles = np.zeros(15)  # Store all 15 motor values
        self.send_home_position()

        self.timer = self.create_timer(1.0 / 30.0, self.update_hand_position)

    def connect_to_hand(self):
        try:
            self.dxl_client = DynamixelClient(ALL_MOTOR_IDS, self.port, 4000000)
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

    def send_home_position(self):
        self.dxl_client.write_desired_pos(ALL_MOTOR_IDS, HOME_POSITION)

    def update_hand_position(self):
        # Get real Ultraleap data
        try:
            ultraleap_angles_deg = compute_all_flexion_angles_from_leap()
        except Exception as e:
            self.get_logger().warn(f"Failed to read Ultraleap angles: {e}")
            return

        flexion_angles = np.array([flexion_deg_to_leap_radians(a) for a in ultraleap_angles_deg])
        all_motor_commands = np.zeros(15)
        for i, motor_id in enumerate(FLEXION_MOTOR_IDS):
            index = ALL_MOTOR_IDS.index(motor_id)
            all_motor_commands[index] = flexion_angles[i]

        self.prev_angles = self.curr_angles
        self.curr_angles = all_motor_commands
        self.dxl_client.write_desired_pos(ALL_MOTOR_IDS, self.curr_angles)

def main(args=None):
    rclpy.init(args=args)
    node = LeapTeleopController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
