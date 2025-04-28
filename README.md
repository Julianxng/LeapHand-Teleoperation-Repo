# LEAP Hand Teleoperation System (with Ultraleap Controller)

---

## Project Overview

This project enables **teleoperation of a LEAP robotic hand** using the **Ultraleap Motion Controller 2** for live hand tracking. It is built with ROS 2 and Python, integrating the Ultraleap hand tracking API to dynamically calculate and command flexion angles to the LEAP hand motors.

- **Teleoperation**: Mapping human finger joint angles (from Ultraleap) to motor commands (on LEAP hand).
- **System split**: Hand tracking and robot control are separated into dedicated ROS 2 nodes.
- **Environment**: ROS 2 Jazzy, Python 3.12, Ubuntu Linux VM.

---

## System Architecture

**Nodes:**

| Node Name                  | Description                                                      |
|-----------------------------|------------------------------------------------------------------|
| `leaphand_node.py`          | Hardware control node. Talks directly to the LEAP motors.        |
| `leap_ultraleap_control.py` | Ultraleap client. Reads live hand tracking data and sends motor commands. |

**Information Flow:**

- **Ultraleap sensor** is directly connected to the laptop.
- **`leap_ultraleap_control.py`** reads hand tracking frames (joint positions).
- Flexion angles are calculated from bone vectors.
- Angles are mapped into LEAP Hand motor commands.
- **`leaphand_node.py`** executes the commands to the physical motors.

---

## Ultraleap Python SDK Installation (Required)

The **Ultraleap Hand Tracking Service** and **Python Bindings** must be installed separately.

### 1. Install Ultraleap Hand Tracking Service

Download and install the Linux version from:  
👉 [Ultraleap Tracking Downloads](https://developer.ultraleap.com/tracking-software-download)

(Install the SDK **before** installing the Python bindings.)

---

### 2. Install Ultraleap Python Bindings

Clone and install the bindings manually:

```bash
cd ~
git clone https://github.com/ultraleap/leapc-python-bindings.git
cd leapc-python-bindings/leapc-python-api
pip install .
```

⚡ **Important:**  
Install inside the same virtual environment (`leap_env`) that you will use to run the ROS 2 package.

---

### 3. Check installation

```bash
python
>>> import leap
```
If no error appears, the installation was successful.

---

## How to Setup and Run

### 1. Install Required Software
- ROS 2 Jazzy
- Ultraleap Hand Tracking Service
- Python3.8, pip
- Clone this repository to **any folder you want** (no dependency on exact path).

---

### 2. Create and Activate Virtual Environment
```bash
python3 -m venv ~/leap_env
source ~/leap_env/bin/activate
```

---

### 3. Install Python Dependencies
```bash
cd ~/path/to/LeapHand-Teleoperation-Repo
pip install -r requirements.txt
```

---

### 4. Build the ROS 2 Workspace
```bash
cd ~/path/to/LeapHand-Teleoperation-Repo/leap_ws
colcon build --symlink-install
source install/setup.bash
```

---

### 5. Run the Launch File
```bash
ros2 launch leap_hand leap_ultra.launch.py
```

✅ This will start both:
- LEAP motor control (`leaphand_node.py`)
- Ultraleap sensor processing (`leap_ultraleap_control.py`)

---

## Current Known Issue

- **Problem:**
  - The node `leap_ultraleap_control.py` crashes because it cannot find the dynamic library `leapc_cffi/_leapc_cffi.so`.
  - Exact Error: `ImportError: Cannot import leapc_cffi: No module named 'leapc_cffi._leapc_cffi'`

- **Cause:**
  - The Ultraleap Python SDK depends on a compiled CFFI shared object, typically located at `/usr/lib/ultraleap-hand-tracking-service/leapc_cffi/`.
  - This file is either missing or not properly linked.

- **Solution:**
  - Reinstall the Ultraleap Hand Tracking Service.
  - Confirm the presence of `_leapc_cffi.so`.

- **Impact:**
  - The LEAP hand initialization (`leaphand_node.py`) works fine.
  - The Ultraleap-based control (`leap_ultraleap_control.py`) does not yet function due to the missing dependency.

---

## Summary of Problems Encountered (SOLVED)

- Multiple motor control issues due to having 15 motors instead of 16.
- Ultraleap TCP server integration was initially planned, later changed to direct sensor connection.
- Python package (`leapc-python-bindings`) conflicted with ROS workspace structure.
- Complexities with ROS 2 launch syntax and Python executables.
- Missing critical shared libraries (`leapc_cffi`) at runtime. (Solution in previous section)

---

## Repo Structure

```plaintext
LeapHand-Teleoperation-Repo/
├── .gitignore
├── README.md
├── requirements.txt
├── leap_ws/
│   └── src/
│       └── ros2_module/
│           ├── package.xml
│           ├── CMakeLists.txt
│           ├── launch/
│           │   └── leap_ultra.launch.py
│           ├── scripts/
│           │   ├── leaphand_node.py
│           │   ├── leap_ultraleap_control.py
│           │   ├── leap_teleop.py
│           │   ├── ultraleap_teleop.py
│           │   └── leap_hand_utils/
│           │       ├── __init__.py
│           │       ├── dynamixel_client.py
│           │       └── leap_hand_utils.py
```
## Purpose of Each Python Script

| Script                             | Purpose |
|------------------------------------|---------|
| `leaphand_node.py`                 | Main ROS 2 node that initializes and controls the LEAP hand motors. Handles low-level communication with Dynamixel motors using the `dynamixel_client.py`. Publishes and listens to ROS services to command the hand. |
| `leap_ultraleap_control.py`        | ROS 2 node that reads live Ultraleap sensor data, computes flexion angles for each finger, maps them to motor commands, and sends them to the LEAP hand. |
| `leap_teleop.py`                   | Provides helper functions for connecting to a TCP server for Ultraleap tracking data. Originally built for TCP-based setups but now supports direct integration too. Contains the `compute_all_flexion_angles_from_leap` function. |
| `ultraleap_teleop.py`              | Standalone TCP client node designed for testing with fake Ultraleap server data. Useful for simulating input if real Ultraleap sensor is unavailable. |
| `leap_hand_utils/dynamixel_client.py` | Python client class for the Dynamixel SDK. Provides functions to connect to motors, send motor commands, and read motor states (position, velocity, effort). |
| `leap_hand_utils/leap_hand_utils.py`  | Helper utilities for converting joint angles between different hand formats (e.g., Allegro to LEAP Hand) and setting motor positions correctly according to hand kinematics. |




---

# End of README


