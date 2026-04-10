# ros2_socketcan

ROS2 SocketCAN 接口封装包，提供 CAN 总线的发送与接收功能。支持 CAN_RAW 和 ISO-TP (CAN_ISOTP) 两种协议。

## 概述

本项目包含两个 ROS2 包：

| 包名 | 说明 |
|------|------|
| `ros2_socketcan` | SocketCAN 核心库及 ROS2 生命周期节点 |
| `ros2_socketcan_msgs` | 自定义消息类型定义 |

### 支持的协议

- **CAN_RAW**：标准 CAN 协议，支持最多 8 字节（经典 CAN）或 64 字节（CAN FD）的数据传输
- **CAN_ISOTP (ISO 15765-2)**：ISO-TP 传输协议，支持最多 4095 字节的多帧数据传输，适用于大数据量场景（如 IMU 数据）

## 依赖

- ROS2（支持 Humble / Iron / Jazzy / Rolling）
- Linux SocketCAN
- `can_msgs`
- `rclcpp`、`rclcpp_lifecycle`、`rclcpp_components`

## 编译

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆仓库
git clone https://github.com/a-strong-python/ros2_socketcan.git

# 编译
cd ~/ros2_ws
colcon build --packages-select ros2_socketcan_msgs ros2_socketcan
source install/setup.bash
```

## 消息类型

### FdFrame.msg

用于 CAN FD 和 ISO-TP 通信的消息类型：

```
std_msgs/Header header
uint32 id
bool is_extended
bool is_error
uint8 len
uint8[<=64] data
```

> **注意**：`data` 字段最大为 64 字节。对于 ISO-TP 传输，实际有效载荷会被截断到 64 字节。对于 40 字节的 IMU 数据等场景完全适用。

## CAN_RAW 使用方法

### 启动 CAN 桥接

```bash
# 启动 CAN_RAW 收发桥接（同时启动 sender 和 receiver）
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=can0
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `interface` | `can0` | CAN 接口名称 |
| `receiver_interval_sec` | `0.01` | 接收器轮询间隔（秒） |
| `sender_timeout_sec` | `0.01` | 发送器超时时间（秒） |
| `enable_can_fd` | `false` | 是否启用 CAN FD |
| `enable_frame_loopback` | `false` | 是否启用本地回环 |
| `filters` | `0:0` | CAN 过滤器 |
| `use_bus_time` | `false` | 是否使用总线时间戳 |

### 话题

| 话题名 | 消息类型 | 方向 | 说明 |
|--------|----------|------|------|
| `from_can_bus` | `can_msgs/msg/Frame` | 发布 | 从 CAN 总线接收的帧 |
| `to_can_bus` | `can_msgs/msg/Frame` | 订阅 | 要发送到 CAN 总线的帧 |

## ISO-TP 使用方法

ISO-TP 协议允许传输超过 8 字节的数据。底层会自动处理多帧分段与重组。

### 前置条件

```bash
# 加载 can_isotp 内核模块
sudo modprobe can_isotp
```

### 启动 ISO-TP 桥接

```bash
# 启动 ISO-TP 收发桥接
ros2 launch ros2_socketcan socket_can_isotp_bridge.launch.xml interface:=can0
```

### 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `interface` | `can0` | CAN 接口名称 |
| `receiver_interval_sec` | `0.01` | 接收器轮询间隔（秒） |
| `sender_timeout_sec` | `0.01` | 发送器超时时间（秒） |
| `sender_tx_id` | `0x600` | 发送端的发送 CAN ID |
| `sender_rx_id` | `0x601` | 发送端的接收 CAN ID（流控） |
| `receiver_tx_id` | `0x601` | 接收端的发送 CAN ID |
| `receiver_rx_id` | `0x600` | 接收端的接收 CAN ID |

### 话题

| 话题名 | 消息类型 | 方向 | 说明 |
|--------|----------|------|------|
| `from_can_bus_isotp` | `ros2_socketcan_msgs/msg/FdFrame` | 发布 | 从 ISO-TP 接收的数据 |
| `to_can_bus_isotp` | `ros2_socketcan_msgs/msg/FdFrame` | 订阅 | 要通过 ISO-TP 发送的数据 |

### 单独启动

```bash
# 仅启动 ISO-TP 接收节点
ros2 launch ros2_socketcan socket_can_isotp_receiver.launch.py \
  interface:=can0 tx_id:=0x601 rx_id:=0x600

# 仅启动 ISO-TP 发送节点
ros2 launch ros2_socketcan socket_can_isotp_sender.launch.py \
  interface:=can0 tx_id:=0x600 rx_id:=0x601
```

## 使用虚拟 CAN 接口测试

```bash
# 创建虚拟 CAN 接口
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# 启动 CAN_RAW 桥接
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=vcan0

# 启动 ISO-TP 桥接
ros2 launch ros2_socketcan socket_can_isotp_bridge.launch.xml interface:=vcan0
```

使用 `can-utils` 工具包进行测试：

```bash
# 安装 can-utils
sudo apt install can-utils

# CAN_RAW 测试
cansend vcan0 123#DEADBEEF        # 发送
candump vcan0                     # 接收

# ISO-TP 测试
echo "01 02 03 04 05 06 07 08 09 0A" | isotpsend -s 0x600 -d 0x601 vcan0
isotprecv -s 0x601 -d 0x600 vcan0
```

## 节点架构

所有节点均采用 ROS2 生命周期节点（Lifecycle Node）实现，支持以下状态转换：

```
Unconfigured → Inactive → Active → Inactive → Unconfigured
                  ↓
              Finalized
```

### CAN_RAW 节点

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| `SocketCanReceiverNode` | `socket_can_receiver_node_exe` | CAN_RAW 接收节点 |
| `SocketCanSenderNode` | `socket_can_sender_node_exe` | CAN_RAW 发送节点 |

### ISO-TP 节点

| 节点 | 可执行文件 | 说明 |
|------|-----------|------|
| `SocketCanIsotpReceiverNode` | `socket_can_isotp_receiver_node_exe` | ISO-TP 接收节点 |
| `SocketCanIsotpSenderNode` | `socket_can_isotp_sender_node_exe` | ISO-TP 发送节点 |

## 项目结构

```
ros2_socketcan/
├── ros2_socketcan/                    # 核心包
│   ├── include/ros2_socketcan/        # 头文件
│   │   ├── socket_can_common.hpp      # 通用工具函数
│   │   ├── socket_can_id.hpp          # CAN ID 封装及常量定义
│   │   ├── socket_can_receiver.hpp    # CAN_RAW 接收器
│   │   ├── socket_can_sender.hpp      # CAN_RAW 发送器
│   │   ├── socket_can_isotp_receiver.hpp  # ISO-TP 接收器
│   │   ├── socket_can_isotp_sender.hpp    # ISO-TP 发送器
│   │   ├── socket_can_receiver_node.hpp   # CAN_RAW 接收节点
│   │   ├── socket_can_sender_node.hpp     # CAN_RAW 发送节点
│   │   ├── socket_can_isotp_receiver_node.hpp  # ISO-TP 接收节点
│   │   └── socket_can_isotp_sender_node.hpp    # ISO-TP 发送节点
│   ├── src/                           # 源文件
│   ├── launch/                        # 启动文件
│   ├── test/                          # 测试文件
│   └── design/                        # 设计文档
└── ros2_socketcan_msgs/               # 消息定义包
    └── msg/
        └── FdFrame.msg                # CAN FD / ISO-TP 消息类型
```

## 许可证

Apache License 2.0
