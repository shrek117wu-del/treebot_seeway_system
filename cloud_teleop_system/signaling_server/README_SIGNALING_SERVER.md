# Signaling Server 使用说明

## 简介

`server.py` 是一个轻量级的 WebSocket 信令服务器，用于在机器人（Jetson）和操作员浏览器之间完成 WebRTC 握手（SDP 信息交换）。握手完成后，双方建立直接的 P2P UDP 连接，信令服务器就不再参与数据传输。

---

## 常见误解

**❌ 不能直接在浏览器地址栏访问 `ws://0.0.0.0:8765/`**

原因有两点：
1. `ws://` 协议不是 HTTP，浏览器地址栏只支持 `http://` 和 `https://`。
2. `0.0.0.0` 是服务器的**绑定地址**（监听所有网卡），不是外部可访问的 IP。

从客户端连接应使用：
- 同机测试：`ws://127.0.0.1:8765`
- 局域网内：`ws://[服务器实际IP]:8765`

---

## 本地完整运行步骤

### 第一步：启动信令服务器

```bash
cd cloud_teleop_system/signaling_server
pip install websockets
python server.py
# 正常输出: Signaling server running on ws://0.0.0.0:8765
```

### 第二步：用 HTTP 托管前端页面

```bash
cd cloud_teleop_system/web_client
python -m http.server 8080
```

### 第三步：浏览器访问前端

打开浏览器，访问：`http://localhost:8080`

页面加载后，插入手柄，点击 **"Connect to Robot"** 按钮，浏览器内的 JavaScript 将自动通过 `ws://127.0.0.1:8765` 与信令服务器建立 WebSocket 握手连接。

---

## 修改远程服务器地址

如果信令服务器部署在另一台机器（或云服务器），编辑 `web_client/main.js` 第一行：

```javascript
// 将地址替换为云服务器公网 IP
const SIGNALING_SERVER_URL = "ws://your.cloud.ip:8765";
```

---

## 架构说明

```
[浏览器 HTTP :8080]  ---页面加载--->  [web_client/index.html + main.js]
       |
       | WebSocket (ws://127.0.0.1:8765)
       v
[signaling_server/server.py :8765]
       |
       | SDP 交换后建立 P2P
       v
[Jetson webrtc_ros_agent.py]   <--(UDP P2P WebRTC)-->  [浏览器视频/控制]
```

---

## 注意事项

- 服务器端口 `8765` 需在防火墙/安全组中放行（TCP）。
- 信令服务器本身不传输视频或控制数据，带宽需求极低（仅握手用）。
- 生产环境建议为信令服务器添加 WSS（HTTPS + 证书）加密。
