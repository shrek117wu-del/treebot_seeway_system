# Goal Description
The objective is to fix high CPU usage on the T113i daemon caused by inefficient GPIO/ADC polling, resolve data races in the TCP server transport, enforce safer service timeout mechanisms, prevent ROS2 node deadlocks, and implement a safe rootfs power-off sequence.

## Proposed Changes

### T113i Daemon: I/O Performance Refactoring [ALREADY IMPLEMENTED]
*   **Status**: Done. Cache file descriptors, replaced `ifstream` with `pread`/`pwrite`.
*   **Status**: Done. Replaced `sleep_for()` busy-waiting with [poll()](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/t113i_daemon/src/gpio_controller.cpp#237-289) interrupt-driven events.

### Jetson ROS 2 Driver: Transport Thread Safety [ALREADY IMPLEMENTED]
*   **Status**: Done. Replaced `int client_fd_` with `std::atomic<int> client_fd_` to fix multi-threading Segfault bugs.

### Jetson ROS 2 Driver: Resolve Service Deadloads (NEW)
Currently, [driver_node.cpp](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_driver/src/driver_node.cpp) registers Services (`SetGpio`, `PowerControl`, `SendTask`, etc.) in the default Mutually Exclusive callback group. Because these service callbacks invoke `std::future::wait_for()` waiting for T113i, they block the default `SingleThreadedExecutor` and cause performance drops or deadlocks across the entire node.

#### [MODIFY] [seeway_interface_driver/include/seeway_interface_driver/driver_node.hpp](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_driver/include/seeway_interface_driver/driver_node.hpp)
- Add `rclcpp::CallbackGroup::SharedPtr service_callback_group_;` to class scope.

#### [MODIFY] [seeway_interface_driver/src/driver_node.cpp](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_driver/src/driver_node.cpp)
- **[on_configure()](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_hardware/src/seeway_hardware_interface.cpp#31-48)**: Initialize `service_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);`.
- Register all 4 ROS services using this `service_callback_group_` so they run concurrently without blocking one another or blocking message subscriptions.

### Safe Graceful Shutdown Handshake (NEW)
When `POWER_ALL_OFF` (ESTOP/SHUTDOWN_ALL) is triggered, the T113i immediately slices power to the Jetson, which guarantees filesystem (`rootfs`) corruption.

#### [MODIFY] [t113i_daemon/include/protocol.h](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/t113i_daemon/include/protocol.h)
- Add `EVT_SYSTEM_REQ = 0x30` to the `EventType` enum.

#### [MODIFY] [t113i_daemon/src/task_executor.cpp](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/t113i_daemon/src/task_executor.cpp)
- **[do_shutdown_all()](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/t113i_daemon/src/task_executor.cpp#162-184)**: Before cutting power, send `MSG_EVENT` with `type=EVT_SYSTEM_REQ, code=1` to the Jetson. Delay for 15 seconds (`std::this_thread::sleep_for(15s)`) to allow the OS to perform a safe shutdown, and *then* trigger `PWR_ALL_OFF`.

#### [MODIFY] [seeway_interface_driver/src/driver_node.cpp](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_driver/src/driver_node.cpp)
- **[handle_input_event()](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_driver/src/driver_node.cpp#483-508)**: Trap `EVT_SYSTEM_REQ` (0x30) with code 1. When caught, issue OS command [system("sudo poweroff");](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/seeway_interface_driver/src/driver_node.cpp#465-482).

### Scheme A (Eclipse Zenoh) Teleop Integration (NEW)
**Goal:** Implement a native ROS 2 to ROS 2 WAN bridging solution using Eclipse Zenoh for ultra-low latency teleoperation, and provide a test plan to compare it against the WebRTC DataChannel (Scheme B).

#### [NEW] `cloud_teleop_system/zenoh_scheme/cloud_router/docker-compose.yml`
- Docker configuration to deploy `zenohd` (Zenoh Router) on a public cloud server, providing a static meeting point for robot and operator.

#### [NEW] `cloud_teleop_system/zenoh_scheme/robot_bridge/robot.json5`
- Configuration for `zenoh-bridge-dds` on the Jetson Orin NX. Connects to the cloud router and exposes `/teleop_joy` to Jetson's local ROS 2 network.

#### [NEW] `cloud_teleop_system/zenoh_scheme/operator_node/`
- A ROS 2 Python package (`operator_joy_node`) for the Operator PC to capture local Gamepad input and publish to `/teleop_joy`.
- `operator.json5` configuration for the operator's PC to bridge `/teleop_joy` back to the Zenoh cloud router.

#### [NEW] Test Plan Artifact
- A structured methodology artifact detailing how to measure latency, jitter, packet loss, and CPU overhead between Zenoh (Scheme A) and WebRTC (Scheme B).
