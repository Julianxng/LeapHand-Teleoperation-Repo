# LEAP Hand Teleoperation System (with Ultraleap Controller)

---

## Project Overview

This project enables **teleoperation of a LEAP robotic hand** using the **Ultraleap Motion Controller 2** for live hand tracking. It is built with ROS 2 and Python, integrating the Ultraleap hand tracking API to dynamically calculate and command flexion angles to the LEAP hand motors.

- **Teleoperation**: Mapping human finger joint angles (from Ultraleap) to motor commands (on LEAP hand).
- **System split**: Hand tracking and robot control are separated into dedicated ROS 2 nodes.
<<<<<<< HEAD
- **Environment**: ROS 2 Jazzy, Python 3.8, Ubuntu Linux VM.
=======
- **Environment**: ROS 2 Jazzy, Python 3.8 (via pyenv), Ubuntu Linux VM.
>>>>>>> 28bf87c (Update README and requirements.txt for Python 3.8/ROS 2 Jazzy compatibility with Ultraleap SDK)

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

### 2. Install Ultraleap Python Bindings

Clone and install the bindings manually:

```bash
cd ~
git clone https://github.com/ultraleap/leapc-python-bindings.git
cd leapc-python-bindings/leapc-python-api
pip install .
```

⚡ **Important:**  Install inside the same virtual environment (`leap_env38`) that you will use to run the ROS 2 package.

### 3. Check Installation

```bash
python
>>> import leap
```
If no error appears, the installation was successful.

---

## How to Setup and Run

### 1. Install Required Software
- ROS 2 Jazzy (Ubuntu 24.04)
- Ultraleap Hand Tracking Service
<<<<<<< HEAD
- Python3.8, pip
=======
- Python 3.8 installed via [pyenv](https://github.com/pyenv/pyenv)
>>>>>>> 28bf87c (Update README and requirements.txt for Python 3.8/ROS 2 Jazzy compatibility with Ultraleap SDK)
- Clone this repository to **any folder you want** (no dependency on exact path).

### 2. Create and Activate Virtual Environment (Python 3.8)
```bash
pyenv install 3.8.18  # if not already installed
pyenv global 3.8.18
python -m venv ~/leap_env38
source ~/leap_env38/bin/activate
```

### 3. Install Python Dependencies
```bash
cd ~/LeapHand-Teleoperation-Repo
pip install -r requirements.txt
```

### 4. Build the ROS 2 Workspace
```bash
cd ~/LeapHand-Teleoperation-Repo/leap_ws
colcon build --symlink-install
source install/setup.bash
```

### 5. Run the Launch File
```bash
ros2 launch leap_hand leap_ultra.launch.py
```

✅ This will start both:
- LEAP motor control (`leaphand_node.py`)
- Ultraleap sensor processing (`leap_ultraleap_control.py`)

---

## Critical Python Version Note

Due to Ultraleap's SDK providing `.so` files built for Python 3.8, **you must use Python 3.8** in your ROS 2 environment.
- Ubuntu 24.04 system Python is 3.12 (incompatible).
- Use **pyenv** to install and manage Python 3.8 cleanly.
- All ROS helper packages (e.g., `empy`, `catkin_pkg`, `lark`, etc.) must be installed manually inside the Python 3.8 virtualenv.

---

## Summary of Problems Encountered (SOLVED)

- Initial missing shared library (`leapc_cffi`) due to Python version mismatch.
- Need for manually building Python 3.8.
- Reinstalling ROS helper libraries (`empy`, `catkin_pkg`, `lark`, etc.) in venv.
- Resolving colcon build errors from missing Python modules.
- Ultraleap CFFI compiled extensions tied to Python 3.8 specifically.

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
| `leaphand_node.py`                 | Main ROS 2 node for LEAP hand motor control. |
| `leap_ultraleap_control.py`        | ROS 2 node to read Ultraleap tracking data and generate motor commands. |
| `leap_teleop.py`                   | Ultraleap TCP server helper (legacy support). |
| `ultraleap_teleop.py`              | Standalone TCP testing client for fake Ultraleap data. |
| `leap_hand_utils/dynamixel_client.py` | Python client for Dynamixel SDK motor communication. |
| `leap_hand_utils/leap_hand_utils.py`  | Helper functions for hand kinematics and angle conversions. |

---

# End of README
