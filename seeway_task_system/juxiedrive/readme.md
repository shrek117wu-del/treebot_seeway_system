# JuxieDrive ROS2 Driver Plan and Usage

本目录现在同时包含原始 PDF 文档和一个新的 ROS2 Python 驱动包 `juxiedrive`，实现时复用了 `lz_omni_chassis_driver` 的组织方式：

- `protocol.py`：协议常量、帧构造、反馈解析
- `can_driver.py`：SocketCAN / CAN FD 收发线程
- `driver_node.py`：ROS2 节点，订阅 JSON 指令 Topic，发布反馈 Topic
- `launch/`、`config/`：启动与参数配置
- `examples/`：覆盖 SDO、PDO、MIT、多轴广播等示例
- `test/`：协议层单元测试

## 已覆盖的文档指令/报文

### 1. CANopen / SDO / NMT / SYNC
- NMT 启动节点（`0x000`）
- SYNC 同步帧（`0x080`）
- 版本/型号查询：`0x1008`、`0x1000`、`0x100A`、`0x1009`
- 修改电机 ID：`0x2530`
- 零位标定：`0x2531`
- 关闭看门狗和限位：`0x2650`
- 设置心跳：`0x1017`
- 正/负限位：`0x607D:02` / `0x607D:01`
- CANFD 数据段波特率：`0x2540`
- 位置模式、速度模式、电流模式控制流程
- PI 参数写入/读取：`0x2532` ~ `0x2538`
- 状态读取：实际电流 `0x6078`、实际位置 `0x6064`、实际速度 `0x606C`、故障 `0x603F`、状态字 `0x6041`、MOS 温度 `0x2662`、电机温度 `0x2663`
- CANopen PDO 映射流程（TPDO1 / RPDO1）

### 2. 自定义 CAN FD PDO
- 单轴控制：`0x100 + node_id`
- 多轴广播控制：`0x200`
- MIT 模式控制：`0x110 + node_id`
- 自定义反馈：`0x300 + node_id`

### 3. 反馈解析
- `0x580 + node_id`：SDO 响应/Abort
- `0x700 + node_id`：上电/心跳
- `0x300 + node_id`：自定义位置/速度/电流/故障/温度/模式反馈
- `0x180 + node_id`：按文档映射后的 TPDO1 反馈

## ROS2 接口

### 输入
- `/juxiedrive/command` (`std_msgs/String`)
  - 内容为 JSON，例如：

```json
{"type":"profile_velocity","node_id":1,"target_velocity_rpm":500,"acceleration":1000,"deceleration":1000}
```

### 输出
- `/juxiedrive/response`：SDO 返回 JSON
- `/juxiedrive/heartbeat`：心跳/上电 JSON
- `/juxiedrive/custom_feedback`：自定义反馈 JSON
- `/juxiedrive/tpdo_feedback`：PDO 映射反馈 JSON
- `/juxiedrive/joint_state`：标准 `sensor_msgs/JointState`
- `/juxiedrive/diagnostics`：标准 `diagnostic_msgs/DiagnosticStatus`
- `/juxiedrive/version`：聚合后的版本 JSON

## 启动

```bash
colcon build --packages-select juxiedrive
source install/setup.bash
ros2 launch juxiedrive juxiedrive.launch.py can_channel:=can0
```

## 示例

```bash
# 仅预览帧，不依赖硬件
python3 seeway_task_system/juxiedrive/examples/command_demo.py --mode preview
python3 seeway_task_system/juxiedrive/examples/custom_pdo_demo.py --mode preview
python3 seeway_task_system/juxiedrive/examples/pdo_mapping_demo.py --mode preview

# 直接通过 CAN/CAN FD 发送
python3 seeway_task_system/juxiedrive/examples/command_demo.py --mode can --can-channel can0
```

## 常用 JSON 指令

```json
{"type":"query_version","node_id":1}
{"type":"enable","node_id":1}
{"type":"set_node_id","node_id":1,"new_node_id":5}
{"type":"set_zero_position","node_id":1}
{"type":"set_heartbeat","node_id":1,"heartbeat_ms":2000}
{"type":"set_positive_limit","node_id":1,"value":32560}
{"type":"set_negative_limit","node_id":1,"value":-32568}
{"type":"profile_position","node_id":1,"target_position":16384,"profile_velocity":10,"acceleration":2000,"deceleration":2000}
{"type":"profile_velocity","node_id":1,"target_velocity_rpm":500,"acceleration":1000,"deceleration":1000}
{"type":"current","node_id":1,"target_current_ma":500}
{"type":"set_pi","node_id":1,"parameter":"speed_kp","value":123}
{"type":"read_pi","node_id":1,"parameter":"speed_kp"}
{"type":"query_state","node_id":1}
{"type":"configure_pdo","node_id":1}
{"type":"rpdo1","node_id":1,"controlword":15,"target_current_ma":100,"target_position":3641}
{"type":"custom_control","node_id":1,"mode":"profile_position","target_1":2530,"target_2":2000,"feedforward":10}
{"type":"multi_control","commands":[{"node_id":1,"mode":"profile_position","target_1":100},{"node_id":2,"mode":"profile_velocity","target_1":500,"target_2":2000}]}
{"type":"mit_control","node_id":1,"position":10.0,"velocity":120.0,"kp":50.0,"kd":1.5,"torque":3.0}
```
