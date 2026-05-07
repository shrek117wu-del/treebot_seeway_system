

# 1. 编译驱动包
colcon build --packages-select lz_omni_chassis_driver
source install/setup.bash

# 2. 启动驱动（UART 模式）
ros2 launch lz_omni_chassis_driver chassis_driver.launch.py comm_type:=uart

# 或启动驱动（CAN 模式）
ros2 launch lz_omni_chassis_driver chassis_driver.launch.py comm_type:=can

# 3. 在另一个终端发送测试命令
ros2 run lz_omni_chassis_driver send_example.py

# 或使用 ROS CLI 发送
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist \
  "linear: {x: 0.5, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.5}"
  
--------------------

colcon build --packages-select seeway_task_msgs seeway_task_manager
# 启动
ros2 launch seeway_task_manager task_system.launch.py

# 发布任务
ros2 run seeway_task_manager task_publisher_example.py

# 监听反馈
ros2 topic echo /task_status_feedback

----------------------------------------------------------------------

验证 UART/CAN 连接与 Topic 工作的完整方法
一、硬件连接验证
UART 连接检查
bash
# 1. 确认串口设备存在
ls -la /dev/ttyUSB* /dev/ttyACM*

# 2. 检查串口权限（避免 Permission denied）
sudo usermod -aG dialout $USER   # 一次性设置，需重新登录
# 或临时授权
sudo chmod 666 /dev/ttyUSB0

# 3. 测试串口原始读写（无 ROS）
sudo apt install minicom
minicom -D /dev/ttyUSB0 -b 115200
# 看是否有乱码数据涌入（说明底盘在主动发送反馈帧）

# 4. 用 python 直接读串口
python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(0.5)
data = s.read(64)
print("RX raw:", data.hex())
s.close()
EOF
CAN 连接检查
bash
# 1. 确认 CAN 接口存在
ip link show can0

# 2. 启动 SocketCAN 接口
sudo ip link set can0 up type can bitrate 500000
ip link show can0   # 应显示 UP 状态

# 3. 实时监听 CAN 总线（需 can-utils）
sudo apt install can-utils
candump can0        # 实时打印所有 CAN 帧
# 期望看到：can0  069   [8]  ...   (电池帧)
#           can0  06B   [8]  ...   (状态帧)
#           can0  06F   [8]  ...   (辅助帧)

# 4. 手动发一帧测试 TX 是否正常（发送零速 0x40 命令）
# [CMD=0x40][00][00][00][00][00][00][XOR=0x40]
cansend can0 001#4000000000000040
二、启动节点并查看连接日志
bash
# 启动（UART 模式）
ros2 launch lz_omni_chassis_driver chassis_driver.launch.py \
    comm_type:=uart uart_port:=/dev/ttyUSB0

# 启动（CAN 模式）
ros2 launch lz_omni_chassis_driver chassis_driver.launch.py \
    comm_type:=can can_channel:=can0

# 开启 debug 日志（查看每帧收发详情）
ros2 launch lz_omni_chassis_driver chassis_driver.launch.py \
    comm_type:=uart \
    --ros-args --log-level chassis_driver_node:=debug
正常启动日志应包含：

Code
[INFO] UART opened: /dev/ttyUSB0 @ 115200 baud
[INFO] Sending version query (CMD 0x11)...
[INFO] Firmware version: v1.2.3 build 45
三、验证每个 Topic
查看所有话题是否存在
bash
ros2 topic list
# 期望输出：
# /battery_state
# /chassis_velocity
# /chassis_diagnostics
# /firmware_version
# /cmd_vel
# /motor_speeds
逐一验证反馈 Topic
bash
# 0x69 → /battery_state
ros2 topic echo /battery_state
# 期望字段：voltage(V), current(A), percentage(0~1), present=true

# 0x6B → /chassis_velocity
ros2 topic echo /chassis_velocity
# 期望字段：twist.linear.x/y (m/s), twist.angular.z (rad/s)

# 0x6F → /chassis_diagnostics
ros2 topic echo /chassis_diagnostics
# 期望字段：name="chassis", level=0(OK), message="temp=25.3°C uptime=12345ms"

# 0x11 → /firmware_version
ros2 topic echo /firmware_version
# 期望字段：data="v1.2.3 build 45"
检查话题发布频率
bash
ros2 topic hz /battery_state       # 应约 10 Hz
ros2 topic hz /chassis_velocity    # 应约 10 Hz
ros2 topic hz /chassis_diagnostics # 应约 10 Hz
四、验证控制指令（TX）
发送 CMD 0x40 全向控制（通过 /cmd_vel）
bash
# 发送一次前进 0.2 m/s 命令
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 持续发送（5 Hz）
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
发送 CMD 0x01 电机控制（通过 /motor_speeds）
bash
# 四轮各设 100 RPM
ros2 topic pub --once /motor_speeds std_msgs/msg/Int32MultiArray \
    "{data: [100, 100, 100, 100]}"
debug 模式验证 TX 帧内容
Code
# 日志中应出现：
[DEBUG] UART TX CMD=0x40: vx=200 vy=0 vw=0  frame=0a0c064000c800000000ce
[DEBUG] UART TX CMD=0x01: m1=100 m2=100 m3=100 m4=100  frame=0a0c08010064006400640064XX
五、端到端闭环验证
bash
# 终端1：监听 chassis_velocity（验证底盘反馈速度）
ros2 topic echo /chassis_velocity --field twist.linear

# 终端2：发送运动指令
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# 终端3：查看 rqt 图形化工具
rqt
# 或
rqt_graph   # 查看节点/话题连接图
rqt_plot /chassis_velocity/twist/linear/x   # 实时绘制速度曲线
六、常见问题排查
现象	原因	解决
Failed to open UART	权限不足或设备不存在	sudo chmod 666 /dev/ttyUSB0
Topic 存在但无数据	底盘未主动发送反馈	检查底盘供电；用 candump/minicom 确认硬件有数据
/chassis_velocity 全为 0	0x6B 帧解析偏移错误	debug 日志打印 raw payload，对照协议逐字节校验
CAN 帧发出但无响应	CAN ID 不匹配	用 candump 确认底盘响应的 arbitration_id
XOR 校验失败日志	帧同步���失	FrameParser 会自动重同步，重新上电底盘
超时后速度不归零	cmd_vel_timeout 设置过长	调小 cmd_vel_timeout: 0.3
  