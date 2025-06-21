# ROS2双目视觉功能包 - 构建与使用说明

## 概述

本功能包将原始的 `stero_vision.py` 双目视觉程序完整移植到ROS2 Humble平台，严格保持与原始程序相同的逻辑和参数。

## 📋 功能包结构

```
src/stereo_vision_ros2/
├── package.xml                           # 功能包配置文件
├── setup.py                              # Python包安装配置
├── resource/stereo_vision_ros2            # ROS2资源标识文件
├── README.md                              # 详细使用说明
├── BUILD_INSTRUCTIONS.md                 # 本文件
├── srv/
│   └── GetDistance.srv                    # 距离查询服务定义
├── launch/
│   └── stereo_vision.launch.py          # 启动文件
└── stereo_vision_ros2/
    ├── __init__.py                       # Python包初始化
    ├── stereo_vision_node.py            # 主节点实现（与原程序逻辑一致）
    └── test_distance_client.py          # 距离查询测试客户端
```

## 🚀 快速开始

### 1. 构建功能包

```bash
# 进入ROS2工作空间
cd ~/ros2_ws

# 构建功能包
colcon build --packages-select stereo_vision_ros2

# 载入环境
source install/setup.bash
```

### 2. 启动双目视觉节点

```bash
# 方法1：直接运行节点
ros2 run stereo_vision_ros2 stereo_vision_node

# 方法2：使用launch文件（推荐）
ros2 launch stereo_vision_ros2 stereo_vision.launch.py
```

### 3. 查看发布的话题

```bash
# 查看所有话题
ros2 topic list

# 查看左摄像头图像
ros2 topic info /stereo_vision/left_image

# 查看视差图
ros2 topic info /stereo_vision/disparity
```

### 4. 测试距离查询服务

```bash
# 启动测试客户端
ros2 run stereo_vision_ros2 test_distance_client

# 测试指定坐标
ros2 run stereo_vision_ros2 test_distance_client 320 240
```

## ⚙️ 核心参数配置（与原程序完全一致）

### 相机参数
- **基线距离**: 25.100mm
- **焦距**: 663像素  
- **光心坐标**: (317, 210)
- **分辨率**: 1280×480

### SGBM算法参数
- **minDisparity**: 3
- **numDisparities**: 16
- **blockSize**: 7
- **P1**: 1176
- **P2**: 4704
- **disp12MaxDiff**: 4
- **preFilterCap**: 31
- **uniquenessRatio**: 10
- **speckleWindowSize**: 100
- **speckleRange**: 32

### WLS滤波器参数
- **lambda**: 8000.0
- **sigma**: 1.5

## 🔧 技术实现要点

### 1. 严格的原程序一致性
- ✅ 所有算法参数与原程序完全一致
- ✅ 图像处理流程与原程序完全一致
- ✅ SGBM+WLS立体匹配算法与原程序完全一致
- ✅ 距离测量逻辑与原程序完全一致

### 2. 完整的错误处理
- ✅ 强制使用traceback模块
- ✅ 详细的中文错误信息
- ✅ 异常情况下的优雅恢复

### 3. ROS2标准接口
- ✅ 标准话题发布接口
- ✅ 自定义服务接口
- ✅ Launch文件支持
- ✅ 参数配置支持

## 📊 话题和服务

### 发布话题
| 话题名称 | 消息类型 | 频率 | 描述 |
|---------|---------|------|------|
| `/stereo_vision/left_image` | `sensor_msgs/Image` | 30Hz | 左摄像头图像 |
| `/stereo_vision/disparity` | `sensor_msgs/Image` | 30Hz | 视差图（伪彩色） |

### 提供服务
| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/stereo_vision/get_distance` | `stereo_vision_ros2/GetDistance` | 查询指定像素点距离 |

## 🛠️ 故障排除

### 常见问题及解决方案

1. **相机设备无法打开**
   ```bash
   # 检查设备
   ls /dev/video*
   
   # 检查权限
   sudo usermod -a -G video $USER
   # 重新登录后生效
   ```

2. **缺少cv_bridge依赖**
   ```bash
   sudo apt install ros-humble-cv-bridge ros-humble-image-transport
   ```

3. **缺少ximgproc模块**
   ```bash
   pip3 install opencv-contrib-python
   ```

4. **服务接口未构建**
   ```bash
   # 重新构建功能包
   colcon build --packages-select stereo_vision_ros2
   source install/setup.bash
   ```

### 调试命令

```bash
# 查看节点状态
ros2 node list
ros2 node info /stereo_vision_node

# 查看话题状态
ros2 topic hz /stereo_vision/left_image
ros2 topic echo /stereo_vision/disparity --no-arr

# 查看服务状态
ros2 service list | grep stereo_vision
ros2 service type /stereo_vision/get_distance

# 查看日志
ros2 run stereo_vision_ros2 stereo_vision_node --ros-args --log-level debug
```

## 📝 开发注意事项

### 算法一致性要求
⚠️ **严格禁止修改以下内容**：
- 所有相机标定参数
- SGBM算法参数配置
- WLS滤波器参数
- 图像预处理流程
- 立体匹配算法实现
- 距离测量公式

### 代码风格要求
- ✅ 所有注释必须使用中文
- ✅ 必须使用traceback模块进行错误处理
- ✅ 每个重要功能块都要有详细注释
- ✅ 异常处理必须包含完整堆栈信息

## 🎯 使用示例

### 基本使用流程

```bash
# 1. 构建功能包
cd ~/ros2_ws
colcon build --packages-select stereo_vision_ros2
source install/setup.bash

# 2. 启动双目视觉节点
ros2 launch stereo_vision_ros2 stereo_vision.launch.py

# 3. 在另一个终端查看图像话题
ros2 topic list | grep stereo_vision

# 4. 测试距离查询（在第三个终端）
ros2 run stereo_vision_ros2 test_distance_client 320 240
```

### 自定义参数启动

```bash
# 指定相机ID
ros2 launch stereo_vision_ros2 stereo_vision.launch.py camera_id:=0

# 设置帧率限制
ros2 launch stereo_vision_ros2 stereo_vision.launch.py fps_limit:=20

# 启用RViz可视化
ros2 launch stereo_vision_ros2 stereo_vision.launch.py enable_rviz:=true
```

## 📈 性能特点

- **实时性**: 30FPS图像处理
- **精度**: 与原程序完全一致的距离测量精度
- **稳定性**: 完善的异常处理和资源管理
- **兼容性**: 标准ROS2接口，易于集成

## 🔗 相关链接

- [原始stero_vision.py代码](../Self-following-Robot/stero_vision.py)
- [ROS2 Humble官方文档](https://docs.ros.org/en/humble/)
- [OpenCV立体视觉文档](https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html)

---

**版本**: 1.0.0  
**最后更新**: 2024年12月  
**维护者**: Stereo Vision Team 