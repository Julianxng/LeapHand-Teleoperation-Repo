import numpy as np

def calculate_finger_flexion_angles(finger_bones):
    """
    Calculates the flexion angles for MCP, PIP, and DIP joints of a single finger.

    Parameters:
        finger_bones (list of dict): Each bone must have 'prev_joint' and 'next_joint',
                                     a 3-element list [x, y, z].
                                     Must be length 4 (metacarpal, proximal, intermediate, distal).

    Returns:
        angles (list): [MCP_flexion, PIP_flexion, DIP_flexion] in degrees.
    """
    if len(finger_bones) != 4:
        raise ValueError("Each finger must have 4 bones (metacarpal, proximal, intermediate, distal)")

    angles = []
    for i in range(1, 4):
        prev_bone = finger_bones[i - 1]
        curr_bone = finger_bones[i]

        vec1 = np.array(curr_bone['next_joint']) - np.array(curr_bone['prev_joint'])
        vec2 = np.array(prev_bone['next_joint']) - np.array(prev_bone['prev_joint'])

        # Normalize vectors
        vec1 /= np.linalg.norm(vec1)
        vec2 /= np.linalg.norm(vec2)

        # Calculate angle using dot product
        dot_product = np.clip(np.dot(vec1, vec2), -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)
        angles.append(angle_deg)

    return angles

def calculate_hand_joint_angles(hand_data):
    """
    Calculates flexion angles for all fingers (excluding the pinky).

    Parameters:
        hand_data (dict): Dictionary with keys 'thumb', 'index', 'middle', 'ring'.
                          Each value is a list of 4 bones with 'prev_joint' and 'next_joint'.

    Returns:
        list: 12 flexion angles (3 per finger, excluding pinky)
    """
    joint_angles = []
    for finger_name in ['thumb', 'index', 'middle', 'ring']:
        bones = hand_data.get(finger_name, [])
        if bones:
            angles = calculate_finger_flexion_angles(bones)
            joint_angles.extend(angles)
    return joint_angles

# === EXAMPLE USAGE ===
if __name__ == '__main__':
    # Simulated example finger bones
    example_finger = [
        {'prev_joint': [0.0, 0.0, 0.0], 'next_joint': [0.0, 1.0, 0.0]},  # Metacarpal
        {'prev_joint': [0.0, 1.0, 0.0], 'next_joint': [0.0, 2.0, 0.0]},  # Proximal
        {'prev_joint': [0.0, 2.0, 0.0], 'next_joint': [0.0, 3.0, 0.0]},  # Intermediate
        {'prev_joint': [0.0, 3.0, 0.0], 'next_joint': [0.0, 4.0, 0.0]}   # Distal
    ]

    # Simulated full hand (4 fingers)
    simulated_hand = {
        'thumb': example_finger,
        'index': example_finger,
        'middle': example_finger,
        'ring': example_finger
    }

    all_angles = calculate_hand_joint_angles(simulated_hand)
    print("All Joint Angles (Thumb to Ring, MCP->DIP):", all_angles)
