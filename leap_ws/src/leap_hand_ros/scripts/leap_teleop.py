#!/usr/bin/env python3

import leap
import numpy as np

# Computes angle in degrees between two vectors
def angle_between(v1, v2):
    v1_u = v1 / (np.linalg.norm(v1) + 1e-6)
    v2_u = v2 / (np.linalg.norm(v2) + 1e-6)
    dot = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.degrees(np.arccos(dot))

def compute_finger_flexions(finger):
    joints = [finger.bones[i].next_joint.to_float_array() for i in range(4)]
    vec1 = np.array(joints[1]) - np.array(joints[0])  # Bone 1
    vec2 = np.array(joints[2]) - np.array(joints[1])  # Bone 2
    vec3 = np.array(joints[3]) - np.array(joints[2])  # Bone 3

    mcp_angle = angle_between(vec1, vec2)
    pip_angle = angle_between(vec2, vec3)
    dip_angle = 0.0  # Thumb doesn’t have a DIP

    return mcp_angle, pip_angle, dip_angle

def compute_thumb_flexions(finger):
    joints = [finger.bones[i].next_joint.to_float_array() for i in range(4)]
    vec1 = np.array(joints[1]) - np.array(joints[0])
    vec2 = np.array(joints[2]) - np.array(joints[1])
    vec3 = np.array(joints[3]) - np.array(joints[2])

    cmc_angle = angle_between(vec1, vec2)
    mcp_angle = angle_between(vec2, vec3)
    ip_angle = 0.0

    return cmc_angle, mcp_angle, ip_angle

def compute_all_flexion_angles_from_leap():
    controller = Leap.Controller()
    frame = controller.frame()
    hand = frame.hands[0] if frame.hands else None

    if hand is None:
        raise RuntimeError("No hand detected")

    angles = []
    for finger_type in [Leap.Finger.TYPE_INDEX, Leap.Finger.TYPE_MIDDLE, Leap.Finger.TYPE_RING]:
        finger = next(f for f in hand.fingers if f.type == finger_type)
        mcp, pip, dip = compute_finger_flexions(finger)
        angles.extend([mcp, pip, dip])

    # Thumb
    thumb = next(f for f in hand.fingers if f.type == Leap.Finger.TYPE_THUMB)
    cmc, mcp, ip = compute_thumb_flexions(thumb)
    angles.extend([cmc, mcp, ip])

    return angles
