# Cloud Teleoperation System (Ultra-Low Latency V2)
**Installation and Running Guide**

This document outlines how to install dependencies, configure, and run the newly optimized hardware-accelerated WebRTC cloud teleoperation system. 

## 1. Architecture Overview

This V2 architecture utilizes:
1. **Targeted Signaling**: Secure room-based WebSocket matching (Robot `<->` Client).
2. **GStreamer Hardware Acceleration**: Bypassing CPU and using NVIDIA NVENC (`nvv4l2h265enc`) for sub-100ms video encoding.
3. **50Hz Control Loop**: The web client reads gamepad inputs and sends UDP packets every 20ms with automatic reconnection logic.
4. **EMA Smoothing**: The ROS 2 Watchdog node smooths out WAN network jitter using an Exponential Moving Average filter before sending velocities to MoveIt2.

---

## 2. Installation Prerequisites

### 2.1 Signaling Server (Cloud or Local PC)
```bash
pip3 install websockets
```

### 2.2 Jetson Orin NX (Robot Side)
You must install GStreamer, its Python bindings, and the WebRTC plugins:
```bash
sudo apt update
sudo apt install -y libgirepository1.0-dev gcc libcairo2-dev pkg-config python3-dev gir1.2-gtk-3.0
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly gstreamer1.0-tools

# Install Python requirements
pip3 install websockets PyGObject
```

*Note: Ensure your ROS 2 workspace is built and sourced so that `rclpy` and `sensor_msgs` are available.*

---

## 3. Running the System

### Step 1: Start the Signaling Server
Run this on a machine accessible to both the Jetson and the operator (can be a cloud server or local PC).
```bash
cd cloud_teleop_system/signaling_server
python3 server.py
# Expected output: Signaling server running on ws://0.0.0.0:8765
```

### Step 2: Start the Teleop Watchdog (Jetson)
Compile and run the C++ Watchdog node to safely listen to the incoming joystick commands and forward them to the arm.
```bash
cd ~/ros2_ws
colcon build --packages-select cloud_teleop_ros
source install/setup.bash

# Run the node
ros2 run cloud_teleop_ros teleop_watchdog_node
```

### Step 3: Start the Hardware-Accelerated WebRTC Agent (Jetson)
*Before running, edit `webrtc_agent/webrtc_gstreamer_agent.py` to point `SIGNALING_SERVER_URL` to your actual signaling server IP.*
```bash
# In a new terminal on Jetson
source ~/ros2_ws/install/setup.bash
cd cloud_teleop_system/webrtc_agent

python3 webrtc_gstreamer_agent.py
# Expected output: Connected to signaling server as robot...
```

### Step 4: Open the Web Client (Operator PC)
1. Host the web client folder:
   ```bash
   cd cloud_teleop_system/web_client
   python3 -m http.server 8080
   ```
2. Open a modern browser (Chrome/Edge) and navigate to `http://localhost:8080`.
3. Plug in your Gamepad (Xbox/PS5).
4. Click **Connect to Robot**. 
5. The UI will establish a P2P WebRTC connection. Video will stream directly from the Jetson's hardware encoder, and joystick commands will be sent at 50Hz via UDP DataChannels.

---

## 4. Configuration & Tuning

### Adjusting the Smoothing Filter (EMA)
If the robotic arm feels too "laggy" or "spongy" to control, you can decrease the smoothing. If the arm jitters too much due to poor 5G signal, increase the smoothing.
- Edit `cloud_teleop_ros/src/teleop_watchdog_node.cpp`.
- Change `this->declare_parameter("ema_alpha", 0.4);` 
  - `1.0` = No smoothing (raw responsiveness)
  - `0.1` = Heavy smoothing (slow response, very stable)

### Modifying the GStreamer Pipeline
The current script assumes a standard V4L2 camera (`/dev/video0`). If you are using the Gemini 330 or an MIPI CSI camera, modify the `PIPELINE_DESC` string in `webrtc_gstreamer_agent.py`:
- **For CSI Cameras**: Replace `v4l2src device=/dev/video0` with `nvarguscamerasrc`.
- **For ROS Image Topics**: Replace the source with `appsrc` and feed it OpenCV frames.
