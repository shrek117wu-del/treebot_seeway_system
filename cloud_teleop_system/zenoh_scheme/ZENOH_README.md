# Zenoh 方案 A 操作手册 (Eclipse Zenoh Teleoperation)

本文档说明如何基于 **Eclipse Zenoh** 建立机器人与操作员之间的 ROS 2 话题云端透传链路，实现原生超低延迟的遥操控制。

---

## 架构概览

```
[操作员 PC]                   [云端服务器]              [Jetson Orin NX]
  pygame 摇杆读取
  → ROS2 /teleop_joy
  → zenoh-bridge-dds  ──── zenohd (router) ────  zenoh-bridge-dds
    (operator.json5)    TCP/UDP WAN             (robot.json5)
                                                  → ROS2 /teleop_joy
                                                  → teleop_watchdog_node
                                                  → MoveIt2 Servo → FRCOBOT
```

整个链路**无需任何 NAT 穿透配置**——双端均主动连接云端 Router，天然穿透防火墙。

---

## 目录结构

```
zenoh_scheme/
├── cloud_router/
│   ├── docker-compose.yml    # 云端一键部署 Zenoh Router
│   └── router.json5          # Router 配置
├── robot_bridge/
│   ├── robot.json5           # Jetson 桥接配置
│   └── start_robot_bridge.sh # 快速启动脚本
└── operator_node/
    ├── operator.json5          # 操作员 PC 桥接配置
    └── operator_gamepad_node.py # 摇杆读取节点
```

---

## 部署步骤

### Step 1 — 云端：部署 Zenoh Router

在您的公网云服务器（需有公网 IP）上执行：

```bash
cd zenoh_scheme/cloud_router
# 确保 7447 端口 (TCP/UDP) 已在安全组中放行
docker compose up -d
```

验证是否启动成功：

```bash
curl http://localhost:8000/router/local
# 应返回 JSON 格式的 Router 信息
```

---

### Step 2 — Jetson：配置并启动机器人桥接

**编辑 `robot.json5`**，将 `YOUR_CLOUD_SERVER_IP` 替换为实际的云服务器公网 IP：

```jsonc
"endpoints": ["tcp/123.45.67.89:7447"]
```

**安装 zenoh-bridge-dds**（Jetson Ubuntu 22.04）：

```bash
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update && sudo apt install zenoh-bridge-dds
```

**启动桥接**：

```bash
cd zenoh_scheme/robot_bridge
bash start_robot_bridge.sh
```

同时启动 ROS 2 安全看门狗（在另一个终端）：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run cloud_teleop_ros teleop_watchdog_node
```

---

### Step 3 — 操作员 PC：启动摇杆节点与桥接

> 操作员 PC 需要安装 ROS 2 Humble 和 `zenoh-bridge-dds`（安装方法同 Step 2）。

**编辑 `operator.json5`**，填入相同的云服务器 IP。

**安装 Python 依赖**：

```bash
pip3 install pygame
```

**启动摇杆节点**（终端 1）：

```bash
cd zenoh_scheme/operator_node
python3 operator_gamepad_node.py
# 输出: Gamepad connected: "Xbox Controller" (Axes: 6, Buttons: 17)
```

**启动操作员端 Zenoh 桥接**（终端 2）：

```bash
zenoh-bridge-dds -c operator.json5
```

---

## 验证链路是否打通

在 Jetson 上运行：

```bash
ros2 topic echo /teleop_joy
```

拨动手柄摇杆，若 Jetson 端能看到实时变化的 Joy 数据，说明端到端链路打通。

---

## 与方案 B (WebRTC) 的关键区别

| 属性 | 方案 A (Zenoh) | 方案 B (WebRTC DataChannel) |
|:--|:--|:--|
| **操作员端要求** | 需安装 ROS 2 + zenoh-bridge | 只需浏览器 |
| **控制数据类型** | 原生 ROS 2 Joy 话题 | JSON over SCTP |
| **视频流** | 需配合其他方案（如 ROS2 image_transport） | 内置在同一 WebRTC 连接中 |
| **延迟特性** | 极低抖动，Rust 原生编写 | 受 JS 事件循环调度影响 |
| **适合场景** | 操作员有 ROS 环境、高精度操作 | 快速部署、无客户端安装需求 |

---

## 下一步

完整测试对比流程请参考：`zenoh_vs_webrtc_test_plan.md`
