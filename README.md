# LEAP Hand Teleoperation System (with Ultraleap Controller and TCP Architecture)

---

## Project Overview

This project enables **teleoperation of a LEAP robotic hand** using the **Ultraleap Motion Controller 2** for live hand tracking. It is built with ROS 2 and Python, cleanly separating the hand tracking and robot control into different environments.

- **Teleoperation**: Mapping human finger joint angles (from Ultraleap) to motor commands (on LEAP hand).
- **System split**: Ultraleap hand tracking runs in a separate TCP server process.
- **Environment**:
  - ROS 2 Jazzy (Python 3.12) for motor control and TCP client node.
  - Python 3.8 (via pyenv) for Ultraleap server and SDK communication.

---

## Updated System Architecture

**Nodes and Processes:**

| Component                      | Description                                                           |
|---------------------------------|-----------------------------------------------------------------------|
| `server.py` (TCP server)        | Reads Ultraleap tracking data and sends all landmarks over TCP.        |
| `leap_tcp_client_node.py`       | ROS 2 node that connects to the TCP server, parses data, computes joint angles, and publishes motor commands. |
| `leaphand_node.py`              | ROS 2 node that receives motor commands and drives LEAP hand motors via Dynamixel SDK. |

**Information Flow:**

- Ultraleap sensor -> `server.py` -> TCP connection -> `leap_tcp_client_node.py` -> ROS 2 topic -> `leaphand_node.py`

---

## Full Repository Structure

```plaintext
LeapHand-Teleoperation-Repo/
├── README.md
├── requirements.txt
├── leap_ws/
│   └── src/
│       └── ros2_module/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── launch/
│           │   └── leap_hand_launch.py
│           ├── scripts/
│           │   ├── leaphand_node.py
│           │   ├── leap_tcp_client_node.py
│           │   └── leap_hand_utils/
│           │       ├── __init__.py
│           │       ├── dynamixel_client.py
│           │       └── leap_hand_utils.py
│           └── srv/
│               ├── LeapPosition.srv
│               ├── LeapVelocity.srv
│               ├── LeapEffort.srv
│               └── LeapPosVelEff.srv
├── teleop_server/
│   ├── server.py
│   ├── constants.py
│   └── requirements.txt
└── tree_output.txt
```


---

## Ultraleap Teleop Server Setup (Python 3.8)

### 1. Install Python 3.8 Using Pyenv
```bash
pyenv install 3.8.18
pyenv virtualenv 3.8.18 leap_env38
pyenv activate leap_env38
```

### 2. Install Server Requirements
```bash
cd ~/teleop_server
pip install -r requirements.txt
```

### 3. Install Ultraleap SDK + Python Bindings
SDK can be installed anywhere, python bindings must be installed in 3.8 venv

Download and install the **Ultraleap Hand Tracking Service for Linux**:

👉 [Ultraleap Tracking Downloads](https://developer.ultraleap.com/tracking-software-download)

> 📌 **Important:** Install this **before** attempting to use the Python bindings. The SDK provides the necessary native `.so` libraries.

---

#### Install Ultraleap Python Bindings

In a terminal where your **Python 3.8 virtual environment (`leap_env38`) is activated**, run:

```bash
# Clone the official Ultraleap Python bindings
git clone https://github.com/ultraleap/leapc-python-bindings.git

# Navigate to the API subdirectory
cd leapc-python-bindings/leapc-python-api

# Install the bindings into your virtual environment
pip install .
```

---

#### Verify Installation

Run Python and try importing the module:

```bash
python
>>> import leap
```

If no error is shown, the installation succeeded.


---

## ROS 2 Workspace Setup (Python 3.12)

### 1. Source ROS 2 Jazzy
```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Build ROS 2 Workspace
```bash
cd ~/LeapHand-Teleoperation-Repo/leap_ws
colcon build --symlink-install
source install/setup.bash
```

---

## How to Launch the Whole System

In **two terminals**:

1. **Terminal 1** (Python 3.8 venv active)
```bash
cd ~/teleop_server
python server.py
```

2. **Terminal 2** (sourced ROS 2 environment)
```bash
cd ~/LeapHand-Teleoperation-Repo/leap_ws
source install/setup.bash
ros2 launch leap_hand leap_hand_launch.py
```

✅ This will start:
- `server.py` (Ultraleap readings and TCP initiation)
- `leap_tcp_client_node.py` (TCP client receiving Ultraleap data, calculating flexions, publishing commands)
- `leaphand_node.py` (motor control)


---

## Python Packages and Versions

| Package         | Purpose                                | Version Needed |
|-----------------|----------------------------------------|----------------|
| Python          | ROS 2 workspace: 3.12, Teleop server: 3.8 | - |
| Leap SDK        | Ultraleap data parsing                  | Python 3.8 only |
| ROS 2 Jazzy     | Main ROS 2 framework                   | Ubuntu 24.04 default |
| Dynamixel SDK   | Motor control library                  | Installed via ROS 2 |


---

## Purpose of Each Key Script

| Script                             | Purpose |
|------------------------------------|---------|
| `leaphand_node.py`                 | Motor control node. Receives motor targets via ROS topic and drives motors. |
| `leap_tcp_client_node.py`          | Connects to TCP server, parses landmark data, computes flexion angles, publishes motor targets. |
| `leap_hand_utils/dynamixel_client.py` | Communicates with Dynamixel motors. |
| `leap_hand_utils/leap_hand_utils.py`  | Helper functions for kinematic conversions (Allegro/LEAP hand mapping). |
| `server.py` (teleop_server)        | TCP server broadcasting Ultraleap tracking data (landmarks). |


---

# End of Updated README


