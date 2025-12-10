# XJC_READ_485 - 6D力传感器RS485读取与CAN转发项目

## 项目简介

本项目基于ESP-IDF开发，实现通过RS485接口读取6D力传感器数据，并将数据通过CAN总线转发。项目支持读取6个维度的力/力矩数据（Fx, Fy, Fz, Mx, My, Mz），并将数据压缩后通过CAN总线发送。

## 主要功能

- **RS485通信**：通过Modbus协议读取6D力传感器数据
- **数据解析**：解析传感器返回的29字节数据，提取6个维度的力/力矩值
- **CAN总线转发**：将解析后的数据通过CAN总线发送，分为两帧（力数据帧和力矩数据帧）
- **数据压缩**：将float类型的力值乘以100后转换为int16格式，节省带宽

## 硬件配置

### ESP32引脚定义

#### RS485接口
- **TX引脚**：GPIO9
- **RX引脚**：GPIO8
- **RTS引脚**：GPIO10（用于RS485收发控制）
- **波特率**：115200 bps

#### CAN接口
- **TX引脚**：GPIO6
- **RX引脚**：GPIO7
- **波特率**：1 Mbps
- **CAN ID**：根据左右手传感器自动设置
  - 右手传感器：0x71
  - 左手传感器：0x72

### 传感器配置

- **通信协议**：Modbus RTU
- **传感器站号**：0x01（默认，可在代码中修改）
- **功能码**：0x04（读取模拟量）
- **数据长度**：29字节响应数据

## 数据格式

### RS485传感器数据

传感器返回29字节数据，包含6个浮点数（每个4字节，大端序）：
- **Fx**：X轴力（N）
- **Fy**：Y轴力（N）
- **Fz**：Z轴力（N）
- **Mx**：X轴力矩（N·m）
- **My**：Y轴力矩（N·m）
- **Mz**：Z轴力矩（N·m）

### CAN数据格式

#### 力数据帧（帧头：0x01）
- **CAN ID**：根据左右手传感器设置（右手：0x71，左手：0x72）
- **数据长度**：7字节
- **数据格式**：
  ```
  [0x01, Fx低字节, Fx高字节, Fy低字节, Fy高字节, Fz低字节, Fz高字节]
  ```
- **数据说明**：力值乘以100后转换为int16（小端序）

#### 力矩数据帧（帧头：0x02）
- **CAN ID**：根据左右手传感器设置（右手：0x71，左手：0x72）
- **数据长度**：7字节
- **数据格式**：
  ```
  [0x02, Mx低字节, Mx高字节, My低字节, My高字节, Mz低字节, Mz高字节]
  ```
- **数据说明**：力矩值乘以100后转换为int16（小端序）

### 数据转换示例

- 原始值：1.23 N
- 转换后：123（int16，实际值 = 1.23 × 100）
- 接收端解析：123 ÷ 100 = 1.23 N

## 编译和烧录

### 环境要求

- ESP-IDF v5.0 或更高版本
- Python 3.6 或更高版本
- CMake 3.16 或更高版本

### 编译步骤

1. 配置项目：
   ```bash
   idf.py menuconfig
   ```

2. 编译项目：
   ```bash
   idf.py build
   ```

3. 烧录到设备：
   ```bash
   idf.py -p PORT flash
   ```

4. 查看串口输出：
   ```bash
   idf.py -p PORT monitor
   ```

   退出串口监视器：按 `Ctrl+]`

## 使用说明

### 启动流程

1. 上电后，程序自动初始化RS485和CAN接口
2. 每100ms发送一次Modbus读取指令到传感器
3. 接收到传感器响应后，解析数据并打印
4. 自动发送两帧CAN数据（力数据帧和力矩数据帧）

### 日志输出示例

```
I (xxx) RS485_SENSOR: RS485传感器读取程序启动
I (xxx) RS485_SENSOR: CAN驱动初始化成功
I (xxx) RS485_SENSOR: UART配置完成，开始读取传感器数据
I (xxx) RS485_SENSOR: ========== 传感器数据 ==========
I (xxx) RS485_SENSOR: Fx: -0.45722 N
I (xxx) RS485_SENSOR: Fy: 2.17374 N
I (xxx) RS485_SENSOR: Fz: 0.29097 N
I (xxx) RS485_SENSOR: Mx: -0.00441 N·m
I (xxx) RS485_SENSOR: My: -0.81292 N·m
I (xxx) RS485_SENSOR: Mz: -0.00023 N·m
I (xxx) RS485_SENSOR: ================================
I (xxx) RS485_SENSOR: CAN力数据帧已发送: ID=0x71, Header=0x01, Fx=-45, Fy=217, Fz=29
I (xxx) RS485_SENSOR: CAN力矩数据帧已发送: ID=0x71, Header=0x02, Mx=0, My=-81, Mz=0

**注意**：CAN ID会根据配置的左右手传感器自动设置（右手0x71，左手0x72）
```

## 故障排除

### CAN发送超时问题

如果出现 `ESP_ERR_TIMEOUT` 错误，请检查：

1. **CAN总线连接**：
   - 确认CAN_H和CAN_L正确连接
   - 确认GND连接

2. **终端电阻**：
   - CAN总线两端需要120Ω终端电阻
   - 如果只有ESP32一个节点，也需要一个终端电阻

3. **CAN收发器**：
   - 确认CAN收发器（如MCP2551、TJA1050等）工作正常
   - 检查收发器供电是否正常

4. **总线负载**：
   - CAN总线需要至少两个节点才能正常通信
   - 如果只有ESP32，可以连接CAN分析仪或其他CAN节点

### RS485通信问题

如果无法读取传感器数据，请检查：

1. **接线**：
   - 确认A/B线正确连接（注意极性）
   - 确认GND连接

2. **传感器站号**：
   - 确认传感器站号与代码中配置一致
   - 可在代码中修改 `SENSOR_STATION_ID` 宏定义

3. **波特率**：
   - 确认传感器波特率与代码中配置一致（115200）

## 代码结构

```
XJC_READ_485/
├── main/
│   ├── 6DForce.c          # 主程序文件
│   └── CMakeLists.txt     # 组件构建配置
├── CMakeLists.txt         # 项目构建配置
├── sdkconfig              # 项目配置文件
└── README.md              # 本文件
```

## 主要函数说明

- `can_init()`：初始化CAN驱动
- `send_force_data_to_can()`：发送力数据到CAN总线
- `send_moment_data_to_can()`：发送力矩数据到CAN总线
- `parse_sensor_data()`：解析传感器返回数据
- `force_to_int16()`：将float力值转换为int16（乘以100）
- `sensor_read_task()`：传感器读取任务（主循环）

## 修改配置

### 修改传感器站号

在 `main/6DForce.c` 中修改：
```c
#define SENSOR_STATION_ID   (0x09)  // 例如改为站号9
```

### 修改CAN引脚

在 `main/6DForce.c` 中修改：
```c
#define CAN_TX_PIN            (6)     // CAN发送引脚
#define CAN_RX_PIN            (7)     // CAN接收引脚
```

### 修改CAN ID（通过左右手传感器配置）

在 `main/6DForce.c` 中修改传感器手型，CAN ID会自动设置：
```c
// 使用右手传感器（CAN ID = 0x71）
#define SENSOR_HAND           (SENSOR_HAND_RIGHT)

// 或使用左手传感器（CAN ID = 0x72）
#define SENSOR_HAND           (SENSOR_HAND_LEFT)
```

**左右手传感器ID定义**：
- `SENSOR_HAND_RIGHT`：右手传感器ID（0x71）
- `SENSOR_HAND_LEFT`：左手传感器ID（0x72）

**注意**：修改 `SENSOR_HAND` 宏后，`CAN_ID` 会自动更新为对应的值。

## 许可证

本项目仅供学习和研究使用。

## 更新日志

### v1.1
- 新增左右手传感器区分功能
- 支持通过 `#define` 宏定义切换左右手传感器
- 右手传感器CAN ID：0x71
- 左手传感器CAN ID：0x72

### v1.0
- 初始版本
- 支持RS485读取6D力传感器数据
- 支持CAN总线转发数据
- 数据压缩和分帧发送功能

