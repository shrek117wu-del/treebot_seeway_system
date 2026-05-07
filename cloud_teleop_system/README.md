# FRCOBOT Cloud Teleoperation System (Ultra-Low Latency)

This directory contains the complete reference implementation for the 3-stage WAN teleoperation architecture. It is designed to provide <150ms glass-to-glass latency for the Gemini 330 camera and sub-10ms logic loops for the FRCOBOT robotic arm.

## Directory Structure
- **`signaling_server`**: A lightweight Python WebSocket server for WebRTC SDP exchange.
- **`web_client`**: An HTML5 frontend that renders the minimal-latency video and captures Gamepad/Joystick input, streaming it via WebRTC Data Channels.
- **`webrtc_agent`**: A Python ROS 2 node running on the Jetson that bridges ROS Image topics to WebRTC, and bridges WebRTC Data Channel JSON payloads into ROS `sensor_msgs/Joy` topics.
- **`cloud_teleop_ros`**: A C++ ROS 2 package containing the `teleop_watchdog_node`. It subscribes to the joystick topic, maps it to MoveIt2 Servo `TwistStamped`, and enforces a strict 100ms connection watchdog to safeguard the arm.

## Run Instructions

### 1. Setup Cloud / LAN Signaling Server
Run the signaling server on a machine accessible by both the Jetson and your Operator PC.
```bash
cd signaling_server
pip3 install websockets
python3 server.py
# Running on ws://0.0.0.0:8765
```

### 2. Build and Run Jetson ROS Modules
Copy the `cloud_teleop_ros` and `webrtc_agent` to your ROS 2 workspace on the Jetson Orin NX.
```bash
cd ~/ros2_ws
colcon build --packages-select cloud_teleop_ros
source install/setup.bash

# Run the safety watchdog (C++)
ros2 run cloud_teleop_ros teleop_watchdog_node

# (In a new terminal) Run the WebRTC Jetson Agent (Python)
pip3 install aiortc websockets opencv-python av
python3 src/webrtc_agent/webrtc_ros_agent.py
```
*(Note: Edit `webrtc_ros_agent.py` to point `SIGNALING_SERVER_URL` to your actual server IP).*

### 3. Open Web Client
Open `web_client/index.html` in a modern browser (Chrome/Edge) on the Operator's computer.
- Connect a supported gamepad (Xbox/PS5 Controller).
- Click **"Connect to Robot"**.
- Video will stream natively to the browser, and joystick data will bypass standard TCP stacks, streaming directly over the UDP-based DataChannel to the Jetson.

---
*If the `teleop_watchdog_node` stops receiving gamepad data for >100ms, it will automatically publish `0` velocities to halt the FRCOBOT arm to prevent collision damage.*
