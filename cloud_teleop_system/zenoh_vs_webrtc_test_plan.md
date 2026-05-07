# Zenoh 对比 WebRTC DataChannel 云端遥控测试验证方案

此文档旨在为**方案 A (Eclipse Zenoh)** 与 **方案 B (WebRTC DataChannel)** 提供标准化的全方位对比测试流程。通过定量实测数据，帮助确定最适合当前 5G/WAN 复杂网况环境的最佳控制链路通信方案。

---

## 1. 测试环境拓扑

为了保证测试公平性与模拟真实环境，采用以下测试拓扑配置：
*   **机器人侧 (Jetson Orin NX)**:
    *   通过外接 4G/5G 路由器或不稳定 Wi-Fi 接入广域网 (WAN)。
    *   同时运行两种接收服务：`teleop_watchdog_node`（ROS2 节点）负责接收及记录指令延迟。
*   **操作端侧 (Operator PC)**:
    *   处于完全不同物理位置的专线或家用宽带环境中。
    *   将同一个硬件手柄（Xbox/PS5）的拨动事件，打上时间戳后，同时经由 Zenoh 端点与 Web 前端触发。
*   **云端基建**:
    *   固定云服务器 IP，分别部署 Zenoh Router (`zenohd`) 和 WebRTC 信令服务器 + Coturn 中转服务。

---

## 2. 核心客观测试指标（Metrics）

### 2.1 端到端指令指令延迟 (End-to-End Latency)与抖动 (Jitter)
*   **测试方法**: 操作端在发出 `Joy` 指令包的 Payload 中强制注入本地绝对高精度时间戳（UNIX Epoch Timestamp, 毫秒级）。Jetson 收到后采用 NTP 时间校准器比对当前系统时间，记录往返或单工时差。
*   **Zenoh 观察点**: 原生 DDS 映射经过 Cloud Router 中转的序列化与反序列化时延开销。
*   **WebRTC 观察点**: SCTP 协议通过 DataChannel（配置 `ordered=false`）在 Web 底层封装处理的 C++ 到 JS 转换开销。

### 2.2 弱网下抗丢包与卡顿率评测 (Packet Loss Resiliency)
*   **测试方法**: 在上游路由器端使用 Linux [tc](file:///D:/WorkingProject/Antigravity_workspace/treebot_interfaceboard_sw_linux=working/t113i_daemon/src/input_handler.cpp#52-55) (Traffic Control) 软件，或者轻量化注入网络损伤工具，模拟 5%、10%、20% 的硬性丢包率。
*   **观察指标**:
    *   在持续的 100Hz 摇杆推送下，接收端观察记录到的接收频率是否平稳。
    *   是否有指令堆积导致机器人瞬间抖动“瞬移”，还是能够做到“优雅的丢旧收新”。

### 2.3 边缘端点资源消耗 (Resource Footprint)
*   **测试方法**: 使用 `htop` 和 `jtop` 在 Jetson 上连续监测 10 分钟。
*   **对比数据**: WebRTC Agent 的 Python 桥接进程占用 `vs` Rust 编写的原生 `zenoh-bridge-dds` 二进制文件的 CPU/RAM 占用率比值。

---

## 3. 主观操控手感测试评测（MoveIt2 伺服评价）

数据漂亮并不等于好用，操作员的真实反馈同样关键。在 100ms（合格）乃至 200ms（较差）的注入延迟网络下，让操作员蒙眼分别使用方案 A 和方案 B：
1.  **急停反馈感**: 拨动摇杆到底然后瞬间松开，观测相机视频流里的机械臂末端是否产生“刹车漂移”或者滞后撞击？由于 `teleop_watchdog_node` 内置了 10~100ms 截断器，该测试将检验哪种传输层能够更快触发并维持存活心跳。
2.  **微操平顺性**: 进行诸如“将螺钉插入孔位”这样细微的 0.1mm 极慢速逼近操作。评估 DDS 端到端原生传输（Zenoh）与 JS JSON 解析（WebRTC）在极低速率推杆下的连续性是否会因为底层协议定时机制产生卡顿。

## 4. 结论记录表 (样表)

| 测试维度 | 环境情况 | 方案A (Eclipse Zenoh) 表现 | 方案B (WebRTC DataChannel) 表现 | 选型建议权重比 |
| :--- | :--- | :--- | :--- | :--- |
| **平均延迟 (Ping)** | 5G 网络，无丢包 | ___ ms | ___ ms | 30% |
| **延迟抖动 (Jitter)**| 5G 网络，无丢包 | ___ ms | ___ ms | 20% |
| **高丢包可用性** | 注入 15% 随机丢包 | 机械臂表现描述... | 机械臂表现描述... | 40% |
| **CPU 占用率** | Jetson Orix NX 静默 | ___ % | ___ % | 10% |

> 注：最终选型可不唯数据论。若方案 B 满足极低延迟要求，且免按装客户端的 Web 控制天然优势巨大，则建议采用方案 B；若方案 A 延迟极低又完美贴合复杂的建图生态，则推荐方案 A。
