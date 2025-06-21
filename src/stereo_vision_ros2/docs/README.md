# ROS2双目立体视觉功能包

## 功能包概述

本功能包将原始的双目视觉程序 `stero_vision.py` 完整移植到ROS2 Humble平台，严格保持与原始程序相同的逻辑和参数，不得随意修改算法实现。

## 核心功能

- **双目摄像头图像采集**: 实时采集左右摄像头图像并进行预处理
- **立体校正与畸变矫正**: 消除镜头畸变并进行立体校正
- **SGBM立体匹配**: 使用半全局匹配算法计算视差图
- **WLS滤波优化**: 应用加权最小二乘滤波器改善视差图质量
- **3D点云生成**: 将视差图转换为三维点云数据
- **距离测量服务**: 提供指定像素点的距离查询功能

## 系统要求

### 硬件要求
- 双目立体摄像头（1280x480分辨率）
- 支持OpenCV的图像采集设备

### 软件依赖
- ROS2 Humble
- OpenCV 4.5+
- Python 3.8+
- cv_bridge
- sensor_msgs
- 可选：Open3D（用于3D点云可视化）

## 安装与编译

### 1. 克隆功能包
```bash
cd ~/ros2_ws/src
# 功能包已在 stereo_vision_ros2 目录中
```

### 2. 安装依赖
```bash
# 安装ROS2依赖
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-sensor-msgs ros-humble-geometry-msgs

# 安装Python依赖
pip3 install opencv-python-headless numpy scipy Pillow

# 可选：安装Open3D用于3D点云可视化
pip3 install open3d
```

### 3. 编译功能包
```bash
cd ~/ros2_ws
colcon build --packages-select stereo_vision_ros2
source install/setup.bash
```

## 使用方法

### 1. 基本启动
```bash
# 启动双目视觉节点
ros2 run stereo_vision_ros2 stereo_vision_node

# 或使用launch文件启动
ros2 launch stereo_vision_ros2 stereo_vision.launch.py
```

### 2. 自定义参数启动
```bash
# 指定相机ID和帧率
ros2 launch stereo_vision_ros2 stereo_vision.launch.py camera_id:=0 fps_limit:=20

# 启用RViz可视化
ros2 launch stereo_vision_ros2 stereo_vision.launch.py enable_rviz:=true
```

### 3. 查看发布的话题
```bash
# 查看所有话题
ros2 topic list

# 查看左摄像头图像
ros2 topic echo /stereo_vision/left_image

# 查看视差图
ros2 topic echo /stereo_vision/disparity
```

### 4. 使用距离查询服务
```bash
# 查询指定像素点的距离
ros2 service call /stereo_vision/get_distance stereo_vision_ros2/srv/GetDistance "{x: 320, y: 240}"
```

## 话题与服务

### 发布的话题
| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/stereo_vision/left_image` | `sensor_msgs/Image` | 左摄像头图像 |
| `/stereo_vision/disparity` | `sensor_msgs/Image` | 视差图（伪彩色） |

### 提供的服务
| 服务名称 | 服务类型 | 描述 |
|---------|---------|------|
| `/stereo_vision/get_distance` | `stereo_vision_ros2/GetDistance` | 查询指定像素点距离 |

### 服务接口定义
```
# GetDistance.srv
# 请求
int32 x     # 像素x坐标
int32 y     # 像素y坐标
---
# 响应
bool success        # 查询是否成功
float64 distance    # 距离值（米）
string message      # 状态信息
```

## 配置参数

### 相机参数（与原程序完全一致）
- **基线距离**: 25.100mm
- **焦距**: 663像素
- **光心坐标**: (317, 210)
- **分辨率**: 1280x480

### SGBM算法参数（与原程序完全一致）
- **minDisparity**: 3
- **numDisparities**: 16
- **blockSize**: 7
- **P1**: 1176
- **P2**: 4704
- **uniquenessRatio**: 10
- **speckleWindowSize**: 100
- **speckleRange**: 32

### WLS滤波器参数（与原程序完全一致）
- **lambda**: 8000.0
- **sigma**: 1.5

## 故障排除

### 常见问题

1. **相机无法打开**
   ```bash
   # 检查相机设备
   ls /dev/video*
   
   # 测试相机
   ros2 run stereo_vision_ros2 stereo_vision_node
   ```

2. **cv_bridge编译错误**
   ```bash
   # 确保安装了cv_bridge
   sudo apt install ros-humble-cv-bridge
   ```

3. **缺少ximgproc模块**
   ```bash
   # 安装完整的OpenCV
   pip3 install opencv-contrib-python
   ```

4. **权限不足**
   ```bash
   # 添加用户到video组
   sudo usermod -a -G video $USER
   # 重新登录生效
   ```

### 调试方法

1. **查看节点日志**
   ```bash
   ros2 run stereo_vision_ros2 stereo_vision_node --ros-args --log-level debug
   ```

2. **检查话题数据**
   ```bash
   # 查看图像话题信息
   ros2 topic info /stereo_vision/left_image
   
   # 查看话题发布频率
   ros2 topic hz /stereo_vision/left_image
   ```

3. **测试距离查询服务**
   ```bash
   # 测试服务可用性
   ros2 service list | grep stereo_vision
   
   # 查看服务接口
   ros2 service type /stereo_vision/get_distance
   ```

## 技术特点

### 与原程序的一致性
- ✅ 严格保持所有算法参数不变
- ✅ 保持相同的图像处理流程
- ✅ 保持相同的SGBM+WLS算法实现
- ✅ 保持相同的距离测量逻辑
- ✅ 保持相同的相机标定参数

### ROS2集成特性
- ✅ 标准ROS2话题发布
- ✅ 服务接口支持
- ✅ 完整的错误处理和日志记录
- ✅ Launch文件支持
- ✅ 参数可配置

### 错误处理
- ✅ 使用traceback模块记录完整错误堆栈
- ✅ 详细的中文错误信息
- ✅ 优雅的资源清理
- ✅ 异常情况下的自动恢复

## 开发说明

### 代码结构
```
stereo_vision_ros2/
├── package.xml              # 功能包配置
├── setup.py                 # Python包配置  
├── resource/                # 资源文件
├── srv/                     # 服务定义
│   └── GetDistance.srv      # 距离查询服务
├── launch/                  # 启动文件
│   └── stereo_vision.launch.py
└── stereo_vision_ros2/      # Python源码
    ├── __init__.py          # 包初始化
    └── stereo_vision_node.py # 主节点实现
```

### 核心类说明
- **StereoConfig**: 立体视觉系统配置（与原程序一致）
- **stereoCamera**: 双目相机参数（与原程序一致）  
- **StereoVisionNode**: ROS2节点主类

### 关键函数
- **preprocess()**: 图像预处理（与原程序一致）
- **undistortion()**: 去畸变处理（与原程序一致）
- **stereoMatchSGBM_WLS()**: SGBM+WLS立体匹配（与原程序一致）
- **reprojectTo3D()**: 3D重投影（与原程序一致）
- **measure_distance()**: 距离测量（与原程序一致）

## 版本信息

- **版本**: 1.0.0
- **作者**: Stereo Vision Team (ROS2移植版本)
- **原始作者**: young (简化版)
- **许可证**: MIT
- **ROS版本**: ROS2 Humble
- **Python版本**: 3.8+

## 联系与支持

如有技术问题或建议，请联系项目维护团队或提交Issue。

---

**重要提醒**: 本功能包严格保持与原始程序 `stero_vision.py` 相同的算法实现和参数配置，确保移植后的功能完全一致。 