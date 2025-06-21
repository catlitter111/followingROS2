# 自主跟踪小车单人跟随系统技术说明文档

## 1. 系统概述

**自主跟踪小车单人跟随系统**（Self-following-Robot）是一个基于计算机视觉和深度学习的智能跟踪系统，能够实时识别并跟踪指定目标人员，通过分析人员位置和距离信息自动控制移动小车保持安全跟随距离。

### 1.1 核心功能特性

- **多模态目标检测**: 融合服装颜色识别和人体姿态检测双重特征
- **智能跟踪算法**: 基于ByteTracker算法的多目标跟踪与单目标锁定
- **双目立体视觉**: 实现精确的距离测量和深度感知
- **自适应运动控制**: 根据目标位置和距离智能调节小车运动策略
- **实时性能优化**: 支持异步处理和多线程并行计算

### 1.2 系统架构图

```
摄像头采集 ──┐
           │
           v
    ┌─────图像预处理─────┐
    │                 │
    v                 v
服装颜色检测        人体姿态检测
    │                 │
    v                 v
    └─────特征提取模块─────┘
             │
             v
    ┌─────ByteTracker─────┐
    │      跟踪器        │
    └─────────────────────┘
             │
             v
    ┌─────目标锁定─────┐
    │    与决策      │
    └─────────────────┘
             │
             v
    ┌─────运动控制─────┐
    │     指令       │
    └─────────────────┘
             │
             v
    ┌─────串口通信─────┐
    │               │
    └─────────────────┘
             │
             v
        移动小车控制
```

## 2. 系统架构详解

### 2.1 整体数据流

系统采用多线程异步处理架构，主要数据流如下：

1. **视频采集线程** → **目标检测线程** → **跟踪处理线程**
2. **双目视觉线程** → **距离计算模块** → **运动控制线程**
3. **显示渲染线程** ← **所有处理结果**

### 2.2 核心模块组成

#### 2.2.1 目标检测层（Detection Layer）
- **RKNN服装检测模块** (`rknn_colour_detect.py`)
- **YOLOv8姿态检测模块** (`yolov8_pose.py`)
- **特征提取统一接口** (`obtain_features.py`)

#### 2.2.2 跟踪决策层（Tracking Layer）
- **ByteTracker核心算法** (`byte_tracker.py`)
- **单目标跟踪器** (`SingleTargetTracker`)
- **多模态特征融合**

#### 2.2.3 距离感知层（Perception Layer）
- **双目立体视觉** (`stero_vision.py`)
- **姿态距离估算** (`pose_distance.py`)
- **比例距离计算** (`ratio_distance.py`)

#### 2.2.4 控制执行层（Control Layer）
- **串口通信模块** (`serial_utils.py`)
- **运动控制算法**
- **多线程协调器** (`multi.py`)

## 3. 关键技术模块详述

### 3.1 目标检测模块 (rknn_colour_detect)

#### 3.1.1 技术原理
采用RKNN（Rockchip Neural Network）硬件加速框架，实现高效的服装检测与颜色识别。

**核心算法流程**：
1. **图像预处理**: Letterbox缩放至640×640分辨率
2. **RKNN推理**: 利用硬件NPU加速YOLOv5服装检测
3. **后处理**: 非极大值抑制(NMS)筛选检测框
4. **颜色提取**: K-means聚类提取主导颜色特征

```python
# 核心检测函数签名
def detect_picture_with_confidence(img):
    """
    执行服装检测并返回置信度信息
    Args:
        img: 输入图像 (BGR格式)
    Returns:
        detection_results: 检测结果列表
        confidence_info: 置信度统计信息
    """
```

**服装类别定义**：
- **上装**: 短袖衫、长袖衫、外套、背心、吊带等6类
- **下装**: 短裤、长裤、裙子、连衣裙等7类

#### 3.1.2 性能优化策略
- **模型缓存**: 预加载RKNN模型减少推理延迟
- **置信度自适应**: 动态调整检测阈值平衡精度与召回率
- **颜色空间优化**: HSV颜色空间提升颜色识别鲁棒性

### 3.2 特征提取模块 (obtain_features)

#### 3.2.1 多模态特征融合
该模块整合服装检测和姿态估计的特征信息，生成用于目标识别的综合特征向量。

**特征类型**：
1. **服装颜色特征**: RGB主导色彩直方图
2. **人体比例特征**: 基于关键点的身体比例数据
3. **姿态特征**: 17个关键点的相对位置关系

```python
# 身体比例计算示例
def calculate_body_ratios(keypoints):
    """
    计算基于关键点的身体比例特征
    包括：上下肢比例、躯干身高比、肩宽身高比等
    """
    ratios = []
    # 1. 上下肢比例
    upper_limb = (distance(5,7) + distance(7,9) + distance(6,8) + distance(8,10)) / 4
    lower_limb = (distance(11,13) + distance(13,15) + distance(12,14) + distance(14,16)) / 4
    ratios.append(upper_limb / lower_limb)
    return ratios
```

#### 3.2.2 特征存储格式
采用Excel格式存储特征数据，支持多人特征库管理：
- **颜色特征**: `person_features.xlsx`
- **比例特征**: `person_ratios_YYYYMMDD_HHMMSS.xlsx`

### 3.3 双目立体视觉模块 (stero_vision)

#### 3.3.1 立体视觉原理
基于双目相机的被动立体视觉系统，通过视差计算实现深度估计。

**核心参数配置**：
```python
class StereoConfig:
    baseline = 25.100      # 基线距离(mm)
    focal_length = 663     # 焦距(像素)
    cx, cy = 317, 210     # 光心坐标
```

**深度计算公式**：
\[ Z = \frac{f \cdot B}{d} \]

其中：
- Z: 深度距离
- f: 相机焦距
- B: 基线距离  
- d: 视差值

#### 3.3.2 SGBM立体匹配算法
采用半全局匹配（Semi-Global Block Matching）算法生成视差图：

```python
def stereoMatchSGBM_WLS(left_image, right_image, config):
    """
    SGBM立体匹配 + WLS滤波优化
    """
    # 创建SGBM匹配器
    left_matcher = cv2.StereoSGBM_create(
        minDisparity=config.minDisparity,
        numDisparities=config.numDisparities,
        blockSize=config.blockSize,
        P1=config.P1,
        P2=config.P2,
        disp12MaxDiff=config.disp12MaxDiff,
    )
    
    # WLS滤波器后处理
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(config.wls_lambda)
    wls_filter.setSigmaColor(config.wls_sigma)
```

### 3.4 ByteTracker跟踪算法 (byte_tracker)

#### 3.4.1 算法核心思想
ByteTracker是基于卡尔曼滤波和匈牙利算法的多目标跟踪框架，具有以下特点：

**跟踪状态机**：
- `NEW`: 新检测目标
- `TRACKED`: 正在跟踪
- `LOST`: 丢失目标
- `REMOVED`: 移除目标

#### 3.4.2 卡尔曼滤波状态预测
采用恒速度模型进行目标状态预测：

**状态向量**: x = [x, y, a, h, ẋ, ẏ, ȧ, ḣ]ᵀ
- (x, y): 目标中心坐标
- a: 宽高比
- h: 目标高度
- (ẋ, ẏ, ȧ, ḣ): 对应速度分量

**运动模型**:
\[ x_{k+1} = F \cdot x_k + w_k \]

其中 F 为状态转移矩阵，w_k 为过程噪声。

#### 3.4.3 多模态数据关联
融合IoU距离和颜色相似度进行数据关联：

```python
def fuse_iou_with_color(iou_cost, color_cost, detections, w_iou=0.7, w_color=0.3):
    """
    融合IoU和颜色代价矩阵
    Cost = w_iou * IoU_cost + w_color * Color_cost
    """
    return w_iou * iou_cost + w_color * color_cost
```

### 3.5 单目标跟踪器 (SingleTargetTracker)

#### 3.5.1 目标选择策略
基于多维特征相似度计算选择最佳跟踪目标：

```python
def calculate_target_score(self, track):
    """
    综合评分函数
    Score = w1*color_sim + w2*ratio_sim + w3*position_score + w4*confidence
    """
    total_score = (
        0.4 * color_similarity +      # 颜色相似度权重40%
        0.3 * ratio_similarity +      # 比例相似度权重30%  
        0.2 * position_score +        # 位置稳定性权重20%
        0.1 * track.score            # 检测置信度权重10%
    )
```

#### 3.5.2 运动控制逻辑
根据目标位置和距离信息生成控制指令：

**距离控制**：
- 距离 > 150cm: 前进加速
- 100cm < 距离 < 150cm: 保持跟随  
- 距离 < 100cm: 减速后退

**方向控制**：
- 基于目标在图像中的x坐标偏移计算转向角度
- 采用PID控制算法平滑转向响应

## 4. 系统工作流程

### 4.1 初始化阶段

1. **硬件初始化**
   - 双目相机标定和立体校正
   - 串口通信连接检测
   - RKNN模型加载和预热

2. **特征库加载**
   - 读取目标人员特征文件
   - 初始化跟踪器参数配置

### 4.2 运行时流程

系统运行时的主要处理流程：

1. **视频采集** → **目标检测** → **特征提取**
2. **目标跟踪** → **状态预测** → **数据关联**
3. **距离计算** → **运动决策** → **控制指令**
4. **串口发送** → **小车执行** → **反馈监控**

### 4.3 异常处理机制

- **目标丢失重捕获**: 基于历史轨迹预测和特征匹配
- **遮挡处理**: 利用卡尔曼滤波预测目标位置
- **多目标干扰**: 通过特征相似度筛选真实目标

## 5. 接口规范与配置

### 5.1 主要API接口

#### 5.1.1 单目标跟踪接口
```python
def track_single_target(
    video_path=None,              # 视频文件路径（None表示实时摄像头）
    target_features_xlsx=None,    # 目标特征文件路径
    output_path=None,            # 输出视频保存路径
    control_car=False,           # 是否启用小车控制
    car_serial_port='COM7',      # 串口设备名
    callback=None,               # 回调函数
    stop_event=None             # 停止事件
):
```

#### 5.1.2 多线程跟踪接口
```python
def track_with_multiple_threads(
    video_path=None,
    target_features_xlsx=None, 
    output_path=None,
    control_car=False,
    car_serial_port=None
):
```

### 5.2 配置参数

#### 5.2.1 检测配置
```python
CONFIG = {
    'conf_threshold': 0.3,           # 检测置信度阈值
    'nms_iou_threshold': 0.1,        # NMS IoU阈值
    'max_x_distance_ratio': 0.2,     # 服装匹配最大x距离比例
    'dominant_color_k': 4,           # 主导颜色聚类数
    'rknn_model_path': 'best3.rknn', # RKNN模型路径
}
```

#### 5.2.2 跟踪配置
```python
class Args:
    track_thresh = 0.5        # 跟踪阈值
    track_buffer = 30         # 跟踪缓冲帧数
    match_thresh = 0.8        # 匹配阈值
    mot20 = False            # MOT20数据集模式
```

### 5.3 硬件要求

#### 5.3.1 推荐配置
- **处理器**: ARM Cortex-A76/A55 (RK3588) 或同等性能
- **NPU**: 6 TOPS算力神经网络处理单元
- **内存**: 8GB LPDDR4/5
- **存储**: 32GB eMMC或更大容量SD卡
- **摄像头**: 双目立体相机（1280×480分辨率）

#### 5.3.2 软件依赖
```bash
# Python核心依赖
numpy>=1.21.0
opencv-python>=4.5.0
scipy>=1.7.0
scikit-learn>=1.0.0

# RKNN推理框架
rknnlite>=1.4.0

# 3D点云处理
open3d>=0.13.0

# 数据处理
openpyxl>=3.0.7
pandas>=1.3.0

# 串口通信  
pyserial>=3.5
```

## 6. 部署与使用指南

### 6.1 环境搭建

#### 6.1.1 系统环境
```bash
# Ubuntu 20.04 LTS 推荐
sudo apt update
sudo apt install python3.8 python3-pip git

# 安装系统依赖
sudo apt install libopencv-dev libserial-dev
```

#### 6.1.2 Python环境配置
```bash
# 创建虚拟环境
python3 -m venv tracking_env
source tracking_env/bin/activate

# 安装依赖包
pip install -r requirements.txt
```

### 6.2 快速启动

#### 6.2.1 特征采集模式
```bash
# 从摄像头采集目标特征
python obtain_features.py --camera_id 0 --name "target_person" --max_frames 100

# 从图像文件提取特征  
python obtain_features.py --image_path "target.jpg" --name "target_person"
```

#### 6.2.2 跟踪模式
```bash
# 实时跟踪（摄像头）
python byte_tracker.py --target_features "features-data/person_features.xlsx" --control_car

# 视频文件跟踪
python byte_tracker.py --video_path "input.mp4" --target_features "features.xlsx" --output_path "result.mp4"
```

### 6.3 串口控制配置

#### 6.3.1 权限设置（Linux）
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 或临时修改权限
sudo chmod a+rw /dev/ttyUSB0
```

#### 6.3.2 控制指令格式
小车控制采用简化指令集：
- `前进`: 向前移动
- `后退`: 向后移动  
- `左转`: 原地左转
- `右转`: 原地右转
- `停止`: 停止所有运动

## 7. 性能分析与优化

### 7.1 性能指标

#### 7.1.1 实时性能
- **检测速度**: 15-20 FPS（RK3588 NPU）
- **跟踪延迟**: < 50ms
- **端到端延迟**: < 100ms

#### 7.1.2 准确性指标
- **目标检测精度**: mAP@0.5 > 85%
- **跟踪成功率**: > 90%（正常光照条件）
- **距离测量误差**: < 5%（1-5米范围）

### 7.2 优化策略

#### 7.2.1 计算优化
- **异步处理**: 检测、跟踪、控制并行执行
- **帧跳跃**: 动态调整处理帧率
- **模型量化**: INT8量化提升推理速度

#### 7.2.2 鲁棒性优化  
- **光照自适应**: 直方图均衡化和CLAHE
- **多特征融合**: 降低单一特征失效风险
- **预测补偿**: 卡尔曼滤波平滑跟踪轨迹

## 8. 故障诊断与维护

### 8.1 常见问题解决

#### 8.1.1 检测问题
**问题**: 目标检测精度低
**解决方案**:
- 检查光照条件，避免逆光或过暗环境
- 调整`conf_threshold`参数适应场景
- 确认目标服装与训练数据分布匹配

#### 8.1.2 跟踪问题  
**问题**: 频繁跟踪丢失
**解决方案**:
- 增大`track_buffer`缓冲时间
- 降低`match_thresh`匹配阈值
- 检查目标特征文件完整性

#### 8.1.3 硬件问题
**问题**: 串口连接失败
**解决方案**:
```bash
# 检测可用串口
python serial_utils.py

# 测试串口连接
python -c "from serial_utils import test_serial_connection; print(test_serial_connection('/dev/ttyUSB0'))"
```

### 8.2 日志分析

系统提供详细的日志记录功能：

```python
# 日志配置示例
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler("system.log")
    ]
)
```

关键日志信息包括：
- 检测性能统计
- 跟踪状态变化
- 控制指令发送记录
- 异常错误堆栈

## 9. 扩展开发指南

### 9.1 自定义检测模型

#### 9.1.1 模型训练
支持使用自定义数据集训练RKNN检测模型：

```python
# 模型转换示例（ONNX -> RKNN）
from rknn.api import RKNN

rknn = RKNN(verbose=True)
rknn.config(target_platform='rk3588')
rknn.load_onnx(model='custom_model.onnx')
rknn.build(do_quantization=True, dataset='quantization_dataset.txt')
rknn.export_rknn('./custom_model.rknn')
```

#### 9.1.2 新增特征类型
扩展特征提取模块支持新的特征类型：

```python
class CustomFeatureExtractor:
    def extract_features(self, image, detections):
        """
        自定义特征提取实现
        Returns: feature_vector
        """
        pass
```

### 9.2 控制协议扩展

#### 9.2.1 新增控制协议
支持扩展不同的小车控制协议：

```python
class CustomCarController:
    def send_command(self, forward_speed, turn_angle):
        """
        自定义控制指令发送实现
        """
        pass
```

### 9.3 算法替换

支持替换核心算法组件：
- **检测算法**: 可替换为YOLOv8、DETR等
- **跟踪算法**: 可替换为DeepSORT、FairMOT等  
- **距离估计**: 可集成激光雷达、超声波等传感器

## 10. 参考文献与致谢

### 10.1 核心算法参考
1. **ByteTrack**: "ByteTrack: Multi-Object Tracking by Associating Every Detection Box" (ECCV 2022)
2. **YOLOv5**: "YOLOv5: A State-of-the-Art Real-Time Object Detection System"
3. **SGBM**: "Accurate and Efficient Stereo Processing by Semi-Global Matching" (CVPR 2008)

### 10.2 开源项目致谢
- OpenCV计算机视觉库
- RKNN神经网络推理框架  
- Open3D三维数据处理库
- scikit-learn机器学习工具包

---

**版本信息**: v1.0.0  
**最后更新**: 2024年12月  
**维护者**: 自主跟踪小车项目团队

如有技术问题或建议，请联系项目维护团队或提交Issue。 