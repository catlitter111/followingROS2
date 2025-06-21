

# ROS2节点详细技术文档

## 1. 概述

* **节点名称**: `dlrobot_robot_node`
* **源码语言**: C++ / Python
* **核心功能**: 该节点是DLRobot机器人底盘的核心控制节点，通过串口与STM32下位机通信，接收来自下位机的IMU传感器数据、里程计数据和电源电压信息，并发布相应的ROS2话题。同时订阅速度指令话题（支持普通Twist和Ackermann两种模式），将指令转换为串口协议并发送给下位机控制机器人运动。

## 2. 依赖项 (Dependencies)

* **ROS2包**: 
  - `rclcpp` - ROS2 C++客户端库
  - `geometry_msgs` - 几何消息类型
  - `nav_msgs` - 导航消息类型
  - `sensor_msgs` - 传感器消息类型
  - `std_msgs` - 标准消息类型
  - `tf2_ros` - TF2坐标变换
  - `ackermann_msgs` - 阿克曼驱动消息类型
  - `dlrobot_robot_msg` - 自定义机器人消息类型
  - `rclpy` - ROS2 Python客户端库

* **系统库**: 
  - `libserial-dev` - 串口通信库
  - `boost` - C++增强库

## 3. 接口规范 (API Specification)

### 3.1 发布的话题
| 话题名称 | 消息类型 | 描述 |
|---|---|---|
| `/odom_combined` | `nav_msgs/msg/Odometry` | 机器人里程计信息，包含位置、姿态、速度和协方差矩阵，发布频率约50Hz |
| `/mobile_base/sensors/imu_data` | `sensor_msgs/msg/Imu` | IMU传感器数据，包含三轴加速度、角速度和姿态四元数，发布频率约100Hz |
| `/PowerVoltage` | `std_msgs/msg/Float32` | 机器人电源电压信息，单位为伏特(V)，发布频率约100Hz |
| `/robotpose` | `dlrobot_robot_msg/msg/Data` | 机器人位置信息的自定义格式 |
| `/robotvel` | `dlrobot_robot_msg/msg/Data` | 机器人速度信息的自定义格式 |

### 3.2 订阅的话题
| 话题名称 | 消息类型 | 描述 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 标准速度指令，包含线速度和角速度，用于控制机器人运动 |
| `/ackermann_cmd` | `ackermann_msgs/msg/AckermannDriveStamped` | 阿克曼驱动指令，包含速度和转向角，适用于阿克曼转向机器人 |

### 3.3 服务
| 服务名称 | 服务类型 | 描述 |
|---|---|---|
| 无 | 无 | 该节点不提供服务接口 |

### 3.4 动作
| 动作名称 | 动作类型 | 描述 |
|---|---|---|
| 无 | 无 | 该节点不提供动作接口 |

### 3.5 参数
| 参数名称 | 数据类型 | 默认值 | 描述 |
|---|---|---|---|
| `usart_port_name` | `string` | `"/dev/dlrobot_controller"` | 串口设备路径，连接STM32下位机的串口 |
| `serial_baud_rate` | `int` | `115200` | 串口波特率，与下位机通信的波特率设置 |
| `robot_frame_id` | `string` | `"base_footprint"` | 机器人坐标系名称 |
| `odom_frame_id` | `string` | `"odom_combined"` | 里程计坐标系名称 |
| `gyro_frame_id` | `string` | `"gyro_link"` | IMU陀螺仪坐标系名称 |
| `cmd_vel` | `string` | `"cmd_vel"` | 速度指令话题名称 |
| `akm_cmd_vel` | `string` | `"ackermann_cmd"` | 阿克曼指令话题名称，设为"none"时禁用阿克曼模式 |
| `product_number` | `int` | `0` | 产品型号标识 |

## 4. 核心逻辑简述 (Core Logic)

* **串口通信协议**: 节点采用自定义的串口协议与STM32下位机通信，数据帧格式为：帧头(0x7B) + 数据 + 校验位 + 帧尾(0x7D)
* **传感器数据处理**: 从下位机接收24字节的传感器数据包，包含三轴速度、IMU原始数据和电源电压，经过单位转换后发布ROS话题
* **里程计积分**: 使用接收到的机器人速度数据，通过数值积分计算机器人在世界坐标系中的位置和姿态
* **四元数姿态解算**: 利用IMU的角速度和加速度数据，通过四元数互补滤波算法计算机器人的三轴姿态
* **双模式支持**: 支持标准Twist速度控制和Ackermann阿克曼转向控制两种模式，可通过参数切换

## 5. 使用与测试指南 (Usage & Testing Guide)

### 5.1 启动方法

* **Launch文件启动**:
  ```bash
  # 普通模式启动（差分驱动）
  ros2 launch turn_on_dlrobot_robot tank.launch.py akmcar:=false
  
  # 阿克曼模式启动（包含cmd_vel到ackermann转换节点）
  ros2 launch turn_on_dlrobot_robot tank.launch.py akmcar:=true
  ```

* **独立运行**:
  ```bash
  # 编译后，使用ros2 run启动主节点
  ros2 run turn_on_dlrobot_robot dlrobot_robot_node --ros-args \
    -p usart_port_name:=/dev/dlrobot_controller \
    -p serial_baud_rate:=115200 \
    -p akm_cmd_vel:=none
  
  # 单独启动cmd_vel到ackermann转换节点
  ros2 run turn_on_dlrobot_robot cmd_vel_to_ackermann_drive.py
  ```

### 5.2 如何控制机器人移动

**确保节点运行**：首先，通过上述方法启动节点，确认串口连接正常。

**发布速度指令**：要控制机器人，需要向话题 `/cmd_vel` 发布 `geometry_msgs/msg/Twist` 类型的消息。

**示例命令**:

**前进**:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**后退**:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**左转**:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

**右转**:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

**停止**:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**阿克曼模式控制**（当启用阿克曼模式时）:
```bash
# 前进并左转
ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.5, steering_angle: 0.3}}"
```

### 5.3 数据监控与调试

**查看机器人状态**:
```bash
# 查看里程计信息
ros2 topic echo /odom_combined

# 查看IMU数据
ros2 topic echo /mobile_base/sensors/imu_data

# 查看电源电压
ros2 topic echo /PowerVoltage
```

**检查话题列表**:
```bash
ros2 topic list
```

**监控TF变换**:
```bash
ros2 run tf2_tools view_frames
```

### 5.4 故障排除

1. **串口连接问题**: 检查设备路径 `/dev/dlrobot_controller` 是否存在，确认串口权限
2. **无法接收数据**: 检查波特率设置是否与下位机匹配（默认115200）
3. **机器人不响应指令**: 确认速度指令话题名称正确，检查串口通信是否正常
4. **IMU数据异常**: 检查下位机IMU传感器连接和初始化状态