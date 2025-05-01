import numpy as np
import socket

def calculate_finger_flexion_angles(finger_bones):
    if len(finger_bones) != 4:
        raise ValueError("Each finger must have 4 bones (metacarpal, proximal, intermediate, distal)")

    angles = []
    for i in range(1, 4):
        prev_bone = finger_bones[i - 1]
        curr_bone = finger_bones[i]

        vec1 = np.array(curr_bone['next_joint']) - np.array(curr_bone['prev_joint'])
        vec2 = np.array(prev_bone['next_joint']) - np.array(prev_bone['prev_joint'])

        vec1 /= np.linalg.norm(vec1)
        vec2 /= np.linalg.norm(vec2)

        dot_product = np.clip(np.dot(vec1, vec2), -1.0, 1.0)
        angle_rad = np.arccos(dot_product)
        angle_deg = np.degrees(angle_rad)
        angles.append(angle_deg)

    return angles

def calculate_hand_joint_angles(hand_data):
    joint_angles = []
    for finger_name in ['thumb', 'index', 'middle', 'ring']:
        bones = hand_data.get(finger_name, [])
        if bones:
            angles = calculate_finger_flexion_angles(bones)
            joint_angles.extend(angles)
    return joint_angles

def parse_csv_position(data_string):
    """Parses a comma-separated XYZ string into a scaled 3D vector."""
    try:
        x_str, y_str, z_str = data_string.strip().split(',')
        x, y, z = float(x_str), float(y_str), float(z_str)
        SCALE = 1.0 / 1000.0
        position = np.array([x, y, z]) * SCALE
        # Optional: apply the same rotation or offset as in C++
        position[1] += -0.2
        position[2] += 0.5
        return position
    except Exception as e:
        print("Failed to parse CSV input:", e)
        return None

def start_tcp_client(host='127.0.0.1', port=60002):
    print(f"Connecting to Ultraleap TCP server at {host}:{port}...")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        print("Connected.")
        while True:
            data = s.recv(1024)
            if not data:
                break
            position = parse_csv_position(data.decode('utf-8'))
            if position is not None:
                print("Received 3D Position:", position)

if __name__ == '__main__':
    start_tcp_client()