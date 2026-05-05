# Seeway Robot Task Management System

## 系统概述

这是一个完整的 ROS 2 任务管理系统，为轮臂式清洁机器人设计。系统包含以下核心功能：

- 📋 **任务发布与管理** - 发布和追踪清洁任务
- 🎮 **Xbox 手柄控制** - 实时遥控机器人底盘
- 🗺️ **导航集成** - 与 Nav2 导航栈集成
- ⚙️ **任务调度** - 自动执行任务队列和状态转换

---

## 🎯 四个核心节点

### 1. Task Manager Node (`task_manager_node.py`)

**功能**：
- 发布任务命令到 `/sys_task_cmd` Topic
- 支持 JSON 格式的参数解析和验证
- 任务追踪与历史管理

**发布的 Topic**：
- `/sys_task_cmd` (TaskCommand) - 任务命令

**订阅的 Topic**：
- `/task_status_feedback` (TaskStatus) - 任务状态反馈

**消息格式**：
```json
{
  "own": "vla",
  "task": "clean task1",
  "param": "{\"location\": \"toilet\", \"force\": 100}"
}
```

**关键方法**：
```python
create_task_command(own, task, param)           # 创建任务
create_task_from_json(json_str)                 # 从JSON创建
publish_task(cmd)                               # 发布任务
publish_task_from_json(json_str)                # 从JSON发布
validate_task_command(cmd)                      # 验证任务
get_task_status(task_id)                        # 查询状态
list_active_tasks()                             # 列表活跃任务
```

---

### 2. Xbox Controller Node (`xbox_controller_node.py`)

**功能**：
- 订阅 Xbox 360 手柄输入 (`/joy` Topic)
- 映射手柄控制到底盘速度命令
- 支持多种控制模式

**订阅的 Topic**：
- `/joy` (sensor_msgs/Joy) - 手柄输入

**发布的 Topic**：
- `/cmd_vel` (geometry_msgs/Twist) - 底盘速度命令

**手柄映射**：
| 输入 | 功能 | 映射 |
|------|------|------|
| 左摇杆 Y 轴 | 前进/后退 | linear_x |
| 左摇杆 X 轴 | 左转/右转 | angular_z |
| LB 按钮 | 启用/禁用 | Toggle enable |
| RB 按钮 | 涡轮加速 | 1.5x 速度倍数 |
| DPAD | 精细控制 | 精确转向 |

**配置参数**（`xbox_controller_config.yaml`）：
```yaml
deadzone: 0.1                    # 死区
max_linear_speed: 1.0            # m/s
max_angular_speed: 2.0           # rad/s
turbo_multiplier: 1.5            # 涡轮倍数
exponential_scale: 2.0           # 指数缩放
cmd_vel_timeout: 0.5             # 超时保护
```

**特点**：
- ✅ 死区处理
- ✅ 指数缩放（平顺非线性控制）
- ✅ 超时保护机制
- ✅ 涡轮加速模式

---

### 3. Nav2 Client Node (`nav2_client_node.py`)

**功能**：
- 与 Nav2 导航栈通信
- 发送 `NavigateToPose` Action 目标
- 监听导航反馈和结果

**使用的 Action**：
- `navigate_to_pose` (nav2_msgs/NavigateToPose)

**导航状态机**：
```
IDLE -> PLANNING -> EXECUTING -> SUCCEEDED/FAILED
```

**关键方法**：
```python
send_navigation_goal(goal_pose, goal_id)        # 发送导航目标
check_goal_reached()                            # 检查是否到达
cancel_navigation()                             # 取消导航
get_navigation_state()                          # 获取状态
is_navigating()                                 # 检查是否导航中
create_pose_from_xy(x, y, theta)                # 创建目标姿态
```

**配置参数**：
```yaml
goal_threshold: 0.2              # 到达阈值 (米)
planning_timeout: 60.0           # 规划超时 (秒)
execution_timeout: 300.0         # 执行超时 (秒)
```

---

### 4. Task Scheduler Node (`task_scheduler_node.py`) - 核心协调器

**功能**：
- 接收任务命令
- 管理任务队列
- 协调底盘导航
- 发布任务状态反馈

**订阅的 Topic**：
- `/sys_task_cmd` (TaskCommand) - 任务命令

**发布的 Topic**：
- `/task_status_feedback` (TaskStatus) - 任务状态反馈

**预定义清洁点**：
```python
'toilet': (1.0, 1.0)      # 马桶
'sink': (2.0, 2.0)        # 洗手台
'urinal': (3.0, 1.0)      # 小便池
'floor': (1.5, 1.5)       # 地面
'wall': (2.5, 2.5)        # 墙面
```

**任务状态流程**：
```
WAITING 
  ↓
NAVIGATING_TO_TARGET (发送导航目标到 RK3588)
  ↓
EXECUTING (执行清洁任务)
  ↓
COMPLETED ✓ (或 FAILED ✗)
```

**关键方法**：
```python
start_next_task()                               # 启动下一个任务
send_navigation_goal(target_xy, theta)          # 发送导航
task_state_checker()                            # 周期性检查
publish_task_status(task, status)               # 发布状态
get_task_statistics()                           # 获取统计信息
```

---

## 🚀 快速开始

### 1. 构建包

```bash
cd ~/colcon_ws
colcon build --packages-select seeway_task_msgs seeway_task_manager
source install/setup.bash
```

### 2. 启动所有节点

```bash
# 启动完整的任务系统
ros2 launch seeway_task_manager task_system.launch.py

# 或启动特定节点
ros2 launch seeway_task_manager task_system.launch.py \
  enable_xbox_controller:=true \
  enable_task_manager:=true \
  enable_nav2_client:=true \
  enable_scheduler:=true
```

### 3. 发布任务命令

**方式1：使用示例程序**
```bash
ros2 run seeway_task_manager task_publisher_example.py
```

**方式2：使用 ROS CLI**
```bash
# 清洁马桶
ros2 topic pub -1 /sys_task_cmd seeway_task_msgs/TaskCommand \
  "{own: 'vla', task: 'clean_toilet', param: '{\"location\": \"toilet\", \"force\": 100}'}"

# 清洁洗手台
ros2 topic pub -1 /sys_task_cmd seeway_task_msgs/TaskCommand \
  "{own: 'vla', task: 'clean_sink', param: '{\"location\": \"sink\", \"force\": 80}'}"
```

### 4. 监听任务状态

```bash
ros2 topic echo /task_status_feedback
```

### 5. 可视化手柄输入

```bash
ros2 run seeway_task_manager joy_visualizer_example.py
```

---

## 📊 系统拓扑

```
┌────────────────────────────────────┐
│   Task Manager Node                │
│   发布 /sys_task_cmd Topic          │
└─────────────┬──────────────────────┘
              ↓
┌────────────────────────────────────┐
│   Task Scheduler Node (核心)        │
│   管理任务队列和状态转换             │
└──────┬───────────────┬──────────────┘
       ↓               ↓
┌─────────────────┐  ┌─────────────────────┐
│ Nav2 Client     │  │ Xbox Controller     │
│ 发送导航目标    │  │ 处理手柄输入        │
└────────┬────────┘  └────────┬────────────┘
         ↓                    ↓
      Nav2栈             底盘 /cmd_vel
    (RK3588)           (x86+RTX 4090)
```

---

## 📝 自定义配置

### 修改清洁位置

编辑 `task_scheduler_node.py` 中的 `cleaning_locations` 字典：

```python
self.cleaning_locations = {
    'custom_location': {
        'start_point': (0.0, 0.0),
        'target_point': (x, y),        # 修改目标坐标
        'theta': 0.0,                  # 修改朝向
        'duration': 120                # 修改执行时间
    }
}
```

### 修改手柄参数

编辑 `config/xbox_controller_config.yaml`：

```yaml
xbox_controller_node:
  ros__parameters:
    max_linear_speed: 1.5              # 增大速度
    max_angular_speed: 3.0             # 增大转向
    exponential_scale: 1.5             # 调整线性度
```

### 修改导航参数

编辑 `config/navigation_config.yaml`：

```yaml
nav2_client_node:
  ros__parameters:
    goal_threshold: 0.3                # 到达阈值
    execution_timeout: 600.0           # 执行超时
```

---

## 🔧 调试和故障排查

### 查看所有 Topic

```bash
ros2 topic list
```

### 查看 Topic 频率

```bash
ros2 topic hz /sys_task_cmd
ros2 topic hz /task_status_feedback
```

### 查看节点信息

```bash
ros2 node list
ros2 node info /task_manager
```

### 启用调试日志

```bash
export ROS_LOG_LEVEL=debug
ros2 launch seeway_task_manager task_system.launch.py
```

### 检查 Nav2 服务

```bash
ros2 service list | grep navigate
ros2 action list | grep navigate
```

---

## 📚 示例代码

### 发布任务

```python
import rclpy
from seeway_task_msgs.msg import TaskCommand

node = rclpy.create_node('example')
pub = node.create_publisher(TaskCommand, '/sys_task_cmd', 10)

msg = TaskCommand()
msg.own = "vla"
msg.task = "clean_toilet"
msg.param = '{"location": "toilet", "force": 100}'

pub.publish(msg)
```

### 订阅任务状态

```python
from seeway_task_msgs.msg import TaskStatus

def callback(msg):
    print(f"Task {msg.task_id}: {msg.status} ({msg.progress}%)")

sub = node.create_subscription(TaskStatus, '/task_status_feedback', callback, 10)
```

---

## 📦 依赖

- ROS 2 Humble/Iron
- nav2_msgs
- sensor_msgs
- geometry_msgs
- rclpy

## 🔗 相关项目

- [Fairino FR5 ROS2 Guide](https://fairino-doc-zhs.readthedocs.io/)
- [Quest2ROS2](https://github.com/Taokt/Quest2ROS2)
- [Fairino Frcobot URDF](https://github.com/FAIR-INNOVATION/frcobot_ros2)
- [Nav2 Documentation](https://navigation.ros.org/)

---

## 📄 许可证

Apache License 2.0

## 👤 作者

Seeway Development Team

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！
