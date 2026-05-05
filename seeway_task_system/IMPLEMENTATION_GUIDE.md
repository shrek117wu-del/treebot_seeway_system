# Seeway Task System

轮臂式清洁机器人ROS2任务管理系统的完整实现

## 📂 目录结构
seeway_task_system/ ├── seeway_task_msgs/ │ ├── msg/ │ │ ├── TaskCommand.msg # 任务命令消息定义 │ │ └── TaskStatus.msg # 任务状态反馈消息定义 │ ├── CMakeLists.txt │ └── package.xml │ └── seeway_task_manager/ ├── seeway_task_manager/ │ ├── init.py │ ├── task_manager_node.py # 任务管理器节点 │ ├── xbox_controller_node.py # Xbox手柄控制节点 │ ├── nav2_client_node.py # Nav2导航客户端节点 │ ├── task_scheduler_node.py # 任务调度器节点（核心） │ └── examples/ │ ├── init.py │ ├── task_publisher_example.py # 任务发布示例 │ └── joy_visualizer_example.py # 手柄可视化示例 │ ├── launch/ │ ├── task_system.launch.py # 完整系统启动文件 │ └── task_scheduler_only.launch.py # 仅调度器启动文件 │ ├── config/ │ ├── xbox_controller_config.yaml # 手柄参数配置 │ └── navigation_config.yaml # 导航参数配置 │ ├── CMakeLists.txt └── package.xml

## 🚀 快速开始

### 1. 编译包

```bash
cd ~/colcon_ws
colcon build --packages-select seeway_task_msgs seeway_task_manager
source install/setup.bash
2. 启动完整系统
bash
# 启动所有节点
ros2 launch seeway_task_manager task_system.launch.py

# 或仅启动调度器和任务管理器
ros2 launch seeway_task_manager task_scheduler_only.launch.py
3. 发布任务
bash
# 方法1：使用示例程序自动发布任务序列
ros2 run seeway_task_manager task_publisher_example.py

# 方法2：手动发布单个任务
ros2 topic pub -1 /sys_task_cmd seeway_task_msgs/TaskCommand \
  "{own: 'vla', task: 'clean_toilet', param: '{\"location\": \"toilet\", \"force\": 100}'}"
4. 监听任务反馈
bash
ros2 topic echo /task_status_feedback
5. 测试手柄控制
bash
ros2 run seeway_task_manager joy_visualizer_example.py
📋 核心功能模块
Task Manager Node
功能：创建和发布任务
发布：/sys_task_cmd
订阅：/task_status_feedback
Xbox Controller Node
功能：处理手柄输入，转换为底盘速度命令
订阅：/joy
发布：/cmd_vel
特性：死区处理、涡轮加速、指数缩放
Nav2 Client Node
功能：与Nav2导航栈通信
使用：navigate_to_pose Action
特性：自动路径规划、目标追踪
Task Scheduler Node（核心）
功能：任务队列管理、状态转移、导航协调
订阅：/sys_task_cmd
发布：/task_status_feedback
预定义清洁点：厕所、洗手台、小便池、地面、墙面、基点
🎮 Xbox 360 手柄映射
输入	功能	说明
左摇杆 Y	前进/后退	linear_x
左摇杆 X	左转/右转	angular_z
LB	启用/禁用	切换控制模式
RB	涡轮加速	1.5倍速度
DPAD	精细控制	替代摇杆
📊 任务状态流程
Code
WAITING 
  ↓
NAVIGATING_TO_TARGET (发送导航目标到RK3588)
  ↓ (导航成功)
EXECUTING (执行清洁任务)
  ↓ (超时或完成)
COMPLETED ✓ (或 FAILED ✗)
🔧 配置说明
修改清洁位置
编辑 config/navigation_config.yaml：

YAML
task_scheduler_node:
  ros__parameters:
    cleaning_locations:
      custom_location:
        x: 1.5
        y: 2.5
        theta: 1.57
        duration: 120
修改手柄参数
编辑 config/xbox_controller_config.yaml：

YAML
xbox_controller_node:
  ros__parameters:
    max_linear_speed: 1.5
    max_angular_speed: 3.0
    exponential_scale: 1.5
📚 API 示例
发布任务
Python
import rclpy
from seeway_task_msgs.msg import TaskCommand

node = rclpy.create_node('example')
pub = node.create_publisher(TaskCommand, '/sys_task_cmd', 10)

msg = TaskCommand()
msg.own = "vla"
msg.task = "clean_toilet"
msg.param = '{"location": "toilet", "force": 100}'

pub.publish(msg)
订阅任务状态
Python
from seeway_task_msgs.msg import TaskStatus

def callback(msg):
    print(f"Task {msg.task_id}: {msg.status} ({msg.progress}%)")

sub = node.create_subscription(TaskStatus, '/task_status_feedback', callback, 10)
🐛 调试
bash
# 查看所有Topic
ros2 topic list

# 实时查看任务命令
ros2 topic echo /sys_task_cmd

# 实时查看任务反馈
ros2 topic echo /task_status_feedback

# 启用调试日志
export ROS_LOG_LEVEL=debug
ros2 launch seeway_task_manager task_system.launch.py
📦 依赖
ROS 2 Humble/Iron
nav2_msgs
sensor_msgs
geometry_msgs
rclpy
🔗 相关链接
Fairino FR5 ROS2 Guide
Quest2ROS2
Fairino URDF
Nav2 文档
📄 许可证
Apache License 2.0

👥 作者
Seeway Development Team