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


**Information Flow:**

- Ultraleap sensor -> `server.py` -> TCP connection -> `leap_tcp_client_node.py` -> Motors
										 -> Joint_states topic -> RVIZ

---

## Full Repository Structure

```plaintext
LeapHand-Teleoperation-Repo/
├── README.md
├── requirements.txt
├── leap_ws/
│   └── src/
│       └── leap_hand_ros/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── launch/
│           │   └── leap_ultra_TCPlaunch.py
│           ├── leap_hand/
│           │   ├── leaphand_node.py (not used right now)
│           │   ├── tcp_leap_reciever.py (main code)
│           │   └── leap_hand_utils/ 
│           │       ├── __init__.py
│           │       ├── dynamixel_client.py
│           │       └── leap_hand_utils.py
│           ├── leap_urdf/
│           │   ├── robot.urdf
│           │   └── meshes/
│           │       ├── All .STL files for URDF
│           ├── rviz/
│           │   ├── Leap.rviz (config)
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
source ~/leap_env38/bin/activate
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
### Install System Python Packages for ROS 2 (IMPORTANT)
Before building, ensure your system Python has the required packages:

```bash
sudo apt update
sudo apt install python3-pip
pip3 install -U catkin_pkg empy lark-parser pyparsing setuptools
```
⚡ Important: These must be installed globally (i.e., not inside your Python 3.8 virtualenv).

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
ros2 launch leap_hand leap_ultra_TCP.launch.py

```

✅ This will start:
- `server.py` (Ultraleap readings and TCP initiation)
- `leap_tcp_client_node.py` (TCP client receiving Ultraleap data, calculating flexions, publishing commands)
- Rviz + Joint_states topic


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
| `TCP_leap_reciever.py`          | Connects to TCP server, parses landmark data, computes flexion angles, publishes motor targets. |
| `leap_hand_utils/dynamixel_client.py` | Communicates with Dynamixel motors. |
| `leap_hand_utils/leap_hand_utils.py`  | Helper functions for kinematic conversions (Allegro/LEAP hand mapping). |
| `server.py` (teleop_server)        | TCP server broadcasting Ultraleap tracking data (landmarks). |


---

# End of Updated README


