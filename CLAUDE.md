# 描述

这是一个风筝的绞盘的硬件设备。类似与Daiwa Winch。

# 硬件构成
- 设备的主控芯片为Arduino Nano3 Compatible（LGTBF328P）
- 经由USB-TTL电平转换，通过serial连接PX4，接受PX4的控制信号，返回结果和状态
- 连接带AB相位霍尔编码器的 TT电机

# 通信协议

项目参考 ArduPilot Daiwa Winch 的 MAVLink 指令结构和回传字段，定制一套轻量级串口协议。
这个方案不依赖飞控修改，Arduino 直接解析 PX4 发出的 MAV_CMD_DO_WINCH 指令，并回传状态。

### 核心设计思路

PX4 本身不驱动绞盘，但它会**广播** `MAV_CMD_DO_WINCH`（指令 ID 42600）到 MAVLink 串口。Arduino 不“移植驱动”，只需要**作为一个 MAVLink 组件**，解析这条指令并执行。

### 控制指令格式（PX4 → Arduino）

PX4 发送的是标准的 `COMMAND_LONG` 消息，关键字段如下：

| 字段 | 含义 | 需要映射的物理动作 |
| :--- | :--- | :--- |
| `param1` | 绞盘实例编号 | 固定填 0 |
| `param2` | **动作类型** | **0=放松（可手动拉出） / 1=长度控制 / 2=速率控制** |
| `param3` | 目标线长（米） | 正值放线，负值收线（配合 `param2=1`） |
| `param4` | 目标速率（m/s） | 正值放线，负值收线（配合 `param2=2`） |

**Arduino侧**：
- 收到 `param2=2, param4=0.5`，就驱动 TT 电机**以 0.5m/s 放线**。
- 收到 `param2=2, param4=-0.3`，就**以 0.3m/s 收线**。
- 收到 `param2=0`，就**松开电机（或低扭矩保持）**，让风筝手拉时能扯出线。
- 收到 `param2=1, param3=10.0`，就**放线到累计长度 10 米处停止**（需要具备长度累计能力）。

### 状态回传格式（Arduino → PX4）

ArduPilot 使用标准的 `WINCH_STATUS` 消息（MAVLink ID 9005）回传状态。Arduino 可以简化发送，只填必要字段：

| 字段 | 类型 | 从编码器/传感器获取的值 |
| :--- | :--- | :--- |
| `line_length` | float | **累计放线长度（米）**。编码器脉冲 × 每脉冲对应线长。 |
| `speed` | float | **当前线速度（m/s）**。正=放线，负=收线。 |
| `tension` | float | 张力（可选）。如果没装拉力传感器，填 `NaN`。 |
| `voltage` | float | 电机供电电压（可选）。 |
| `current` | float | 电机电流（可选，建议加采样做保护）。 |
| `temperature` | int16 | 填 `INT16_MAX`（表示未知）。 |
| `status` | uint32 | 状态位掩码：**健康、运动中、完全收回、离合器**等。 |

### Arduino侧 实现建议

**LGTBF328P 资源有限**，完整 MAVLink 库跑不动。建议：

1. **控制端**：用 `pulseIn` 监听 PWM 转 MAVLink 不现实。更简单的做法是：让 PX4 把指令通过 **MAVLink 串口**直接发给Arduino，Arduino 里只解析**固定的字节序列**（比如识别 `42600` 这个 ID 的 `COMMAND_LONG` 结构）。
2. **状态回传**：可以**不严格遵循 MAVLink 格式**，而是用自定义帧（比如 `0xFA + float len + float speed + checksum`），让 PX4 端的 Lua 脚本接收并转发。这样对 Arduino 的压力最小。
3. **“类似舵机”的关键**：`param2=2`（速率控制）就是实现“舵机手感”的方式——飞控给一个速度值，通过PID 闭环让电机转速紧跟这个值，**负载变化时自动补偿扭矩**。
