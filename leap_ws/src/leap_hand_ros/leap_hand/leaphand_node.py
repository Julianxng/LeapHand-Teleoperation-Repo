#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from leap_hand.leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand.leap_hand_utils.leap_hand_utils as lhu
from leap_hand_ros.srv import LeapPosition, LeapVelocity, LeapEffort, LeapPosVelEff

class LeapNode(Node):
    def __init__(self):
        super().__init__('leaphand_node')
        self.kP = self.declare_parameter('kP', 600.0).get_parameter_value().double_value
        self.kI = self.declare_parameter('kI', 0.0).get_parameter_value().double_value
        self.kD = self.declare_parameter('kD', 100.0).get_parameter_value().double_value
        self.curr_lim = self.declare_parameter('curr_lim', 350.0).get_parameter_value().double_value
        self.ema_amount = 0.2

        # Motor IDs 1-15 only
        self.motors = list(range(1, 16))

        # Initial pose with dummy 0 at front
        self.prev_pos = self.pos = self.curr_pos = np.insert(
            lhu.allegro_to_LEAPhand(np.zeros(15)), 0, 0.0
        )

        self.create_subscription(JointState, 'cmd_leap', self._receive_pose, 10)
        self.create_subscription(JointState, 'cmd_allegro', self._receive_allegro, 10)
        self.create_subscription(JointState, 'cmd_ones', self._receive_ones, 10)

        self.create_service(LeapPosition, 'leap_position', self.pos_srv)
        self.create_service(LeapVelocity, 'leap_velocity', self.vel_srv)
        self.create_service(LeapEffort, 'leap_effort', self.eff_srv)
        self.create_service(LeapPosVelEff, 'leap_pos_vel_eff', self.pos_vel_eff_srv)
        self.create_service(LeapPosVelEff, 'leap_pos_vel', self.pos_vel_srv)

        try:
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception:
            try:
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
            except Exception:
                self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB2', 4000000)
                self.dxl_client.connect()

        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([1, 8], np.ones(2) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([1, 8], np.ones(2) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos[1:])

    def _receive_pose(self, msg):
        pose = np.insert(np.array(msg.position), 0, 0.0)[:16]
        self.prev_pos = self.curr_pos
        self.curr_pos = pose
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos[1:])

    def _receive_allegro(self, msg):
        pose = np.insert(lhu.allegro_to_LEAPhand(msg.position, zeros=False), 0, 0.0)[:16]
        self.prev_pos = self.curr_pos
        self.curr_pos = pose
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos[1:])

    def _receive_ones(self, msg):
        pose = np.insert(lhu.sim_ones_to_LEAPhand(np.array(msg.position)), 0, 0.0)[:16]
        self.prev_pos = self.curr_pos
        self.curr_pos = pose
        self.dxl_client.write_desired_pos(self.motors, self.curr_pos[1:])

    def pos_srv(self, request, response):
        response.position = self.dxl_client.read_pos().tolist()
        return response

    def vel_srv(self, request, response):
        response.velocity = self.dxl_client.read_vel().tolist()
        return response

    def eff_srv(self, request, response):
        response.effort = self.dxl_client.read_cur().tolist()
        return response

    def pos_vel_srv(self, request, response):
        output = self.dxl_client.read_pos_vel()
        response.position = output[0].tolist()
        response.velocity = output[1].tolist()
        response.effort = np.zeros_like(output[1]).tolist()
        return response

    def pos_vel_eff_srv(self, request, response):
        output = self.dxl_client.read_pos_vel_cur()
        response.position = output[0].tolist()
        response.velocity = output[1].tolist()
        response.effort = output[2].tolist()
        return response

def main(args=None):
    rclpy.init(args=args)
    leaphand_node = LeapNode()
    rclpy.spin(leaphand_node)
    leaphand_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
