# Goal Description

The objective is to refine and optimize the existing cloud teleoperation system (`cloud_teleop_system`) to achieve true ultra-low latency (<100ms) and production-level stability. The current implementation provides a solid foundation but suffers from a few bottlenecks: the Python signaling server broadcasts to all clients, the WebRTC video pipeline on the Jetson uses CPU-heavy Python conversions instead of hardware acceleration, the web client lacks reconnection resilience, and the ROS 2 teleop commands are susceptible to network jitter.

## User Review Required

Please review the proposed optimizations below. In particular, note the shift from `aiortc`'s built-in video tracks to a native GStreamer `webrtcbin` pipeline for the Jetson. This is critical for achieving sub-100ms video latency using the Orin NX's NVENC hardware encoder.

## Proposed Changes

### Phase 1: Signaling Server Optimization

**Problem:** The current `server.py` broadcasts all SDP and ICE candidate messages to every connected client indiscriminately. This causes connection failures if multiple robots or operators are connected to the same signaling server.
**Solution:**
- Update `signaling_server/server.py` to implement a simple room-based or ID-based pairing mechanism (e.g., passing a query parameter like `/?id=robot1`).
- Ensure operators only exchange WebRTC signaling data with their designated robot.

### Phase 2: Hardware-Accelerated Video Pipeline (Jetson WebRTC)

**Problem:** `webrtc_ros_agent.py` uses `cv_bridge` and PyAV to convert ROS Image messages to video frames in Python. This entirely bypasses the Jetson's hardware encoders, maxing out CPU usage and adding hundreds of milliseconds of latency.
**Solution:**
- Rewrite the video streaming portion of the Jetson agent to use GStreamer's `webrtcbin` (`gi.repository.GstWebRTC`).
- Construct a pipeline that reads directly from the camera or a ROS 2 shared memory topic and uses `nvv4l2h265enc` (NVIDIA hardware H.265 encoding) for zero-copy encoding.
- Retain the DataChannel logic for receiving JSON joystick commands and publishing them to `/teleop_joy`.

### Phase 3: Web Client Resilience and Responsiveness

**Problem:** The Web UI's gamepad loop runs at 20Hz, which might feel slightly jerky for high-speed robotic arm movements. Furthermore, if the signaling server disconnects, there is no automatic reconnection.
**Solution:**
- Update `web_client/main.js` to increase the Gamepad polling loop from 20Hz to 50Hz (20ms interval).
- Implement exponential backoff auto-reconnection logic for the WebSocket signaling.
- Add ping/latency estimation on the DataChannel to display real-time network health to the operator.

### Phase 4: ROS 2 Teleop Command Smoothing

**Problem:** Over 4G/5G WAN, packets may arrive with jitter (e.g., two packets arrive simultaneously, then a 60ms gap). Feeding raw, jittery velocity commands directly to MoveIt2 Servo can cause the FRCOBOT arm to vibrate or exhibit jerky motion.
**Solution:**
- Update `cloud_teleop_ros/src/teleop_watchdog_node.cpp`.
- Implement an Exponential Moving Average (EMA) or a simple low-pass filter on the output `TwistStamped` velocities.
- This will smooth out the control signals before they reach the servo controller, while still respecting the 100ms safety watchdog (which will forcefully zero velocities on true packet loss).

## Verification Plan

### Manual Verification
1. **Signaling**: Run multiple instances of the web client and verify they don't receive each other's SDP offers.
2. **Video Latency**: Deploy the updated Jetson GStreamer agent and measure glass-to-glass latency using a millisecond stopwatch on a screen in front of the camera. Target: < 100ms.
3. **Control Smoothness**: Observe the robotic arm's movement while injecting network jitter; verify the EMA filter dampens vibrations compared to the raw feed.
