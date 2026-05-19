# JuxieDrive ROS2 CAN driver

`seeway_task_system/juxiedrive` implements a ROS2 Python driver for the JuxieDrive integrated joint modules described in:

- `关节模组使用协议简易说明书.pdf`
- `巨蟹智能产品选型手册V2.0.3压缩版(3).pdf`

The package follows the same layered structure as `seeway_task_system/lz_omni_chassis_driver`:

- `juxiedrive/protocol.py` — documented CANopen/CAN-FD frame builders and parsers
- `juxiedrive/can_driver.py` — SocketCAN transport
- `juxiedrive/joint_device.py` — single-joint abstraction and synchronous SDO helpers
- `juxiedrive/juxiedrive_node.py` — ROS2 topics/services/diagnostics bridge
- `config/*.yaml`, `launch/*.py`, `examples/*.py`, `test/*.py`

## Documented commands covered

### CANopen / SDO commands

- Version information reads (`0x1008`, `0x1000`, `0x100A`, `0x1009`)
- Joint ID write (`0x2530`)
- Zero calibration (`0x2531`)
- Position read (`0x6064`)
- Disable watchdog / limit protection (`0x2650`)
- Heartbeat configuration (`0x1017`)
- Positive / negative limit write and read (`0x607D`)
- CAN-FD data bitrate write (`0x2540`)
- Profile-position control sequence (`0x6040`, `0x6060`, `0x6083`, `0x6084`, `0x6081`, `0x607A`)
- Profile-velocity control sequence (`0x6060`, `0x6083`, `0x6084`, `0x60FF`)
- Current-mode control sequence (`0x6060`, `0x6071`)
- PI parameter write / read (`0x2532` .. `0x2538`)
- State reads (`0x6078`, `0x6064`, `0x606C`, `0x603F`, `0x6041`, `0x2662`, `0x2663`)

### Custom CAN-FD commands

- Single-joint custom PDO control (`0x100 + node_id`)
- Multi-joint broadcast control (`0x200`)
- MIT control (`0x110 + node_id`)
- SYNC trigger (`0x080`)
- Custom feedback parser (`0x300 + node_id`)
- Boot-up / heartbeat parser (`0x700 + node_id`)

## ROS2 interfaces

### Topic

- Subscribed: `/juxiedrive/command` (`juxiedrive/msg/JointCommand`)
- Published: `/juxiedrive/status` (`juxiedrive/msg/JointStatus`)
- Published: `/joint_states` (`sensor_msgs/msg/JointState`)
- Published: `/juxiedrive/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)

### Services

- `/juxiedrive/execute_command` (`juxiedrive/srv/ExecuteCommand`)
- `/juxiedrive/read_object` (`juxiedrive/srv/ReadObject`)
- `/juxiedrive/write_object` (`juxiedrive/srv/WriteObject`)

`ExecuteCommand` supports high-level commands such as:

- `start_node`
- `sync_feedback`
- `enable`
- `set_zero`
- `disable_watchdog`
- `set_id`
- `set_heartbeat`
- `set_positive_limit`
- `set_negative_limit`
- `read_positive_limit`
- `read_negative_limit`
- `set_canfd_bitrate`
- `set_pi`
- `read_pi`
- `query_versions`
- `query_state`
- `profile_position`
- `profile_velocity`
- `current_mode`
- `single_pdo`
- `mit`
- `broadcast`

## Build and run

```bash
cd ~/colcon_ws/src/treebot_seeway_system/seeway_task_system
colcon build --packages-select juxiedrive
source install/setup.bash
ros2 launch juxiedrive juxiedrive.launch.py
```

## Examples

```bash
python3 examples/frame_preview_example.py
python3 examples/control_sequence_example.py
```

## Tests

```bash
python3 -m unittest discover test
```
