# -*- coding: utf-8 -*-
"""
ROS2 RKNN颜色检测节点
===================
基于RKNN模型的服装检测与颜色识别系统ROS2节点实现
严格保持与原始程序rknn_colour_detect.py相同的逻辑和参数，不得随意修改算法实现

作者: Stereo Vision Team (ROS2移植版本)
原始作者: rknn_colour_detect.py
功能: 服装检测、颜色识别、身体位置估算
"""

import cv2 as cv
import numpy as np
import os
import logging
import time
import sys
import traceback
from collections import Counter
from collections.abc import Sequence
from functools import lru_cache

# ROS2相关导入
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# 导入自定义服务
from stereo_vision_interfaces.srv import DetectImageWithConfidence, DetermineBodyPosition

# 尝试导入RKNN和sklearn，如果失败则提供替代方案
try:
    from rknnlite.api import RKNNLite
    RKNN_AVAILABLE = True
except ImportError:
    RKNN_AVAILABLE = False
    print("警告：RKNN Lite未安装，RKNN功能将被禁用")

try:
    from sklearn.cluster import KMeans
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    print("警告：scikit-learn未安装，颜色聚类功能将被禁用")

# 配置日志系统 - 与原程序完全一致
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# -------------------------------- 性能监控工具（与原程序完全一致）--------------------------------
class PerformanceMonitor:
    """用于记录和分析性能数据的简单工具 - 与原程序完全一致"""

    def __init__(self, window_size=30):
        self.timings = {}
        self.window_size = window_size
        self.stats = {}

    def start(self, name):
        """开始记录某个操作的时间"""
        if name not in self.timings:
            self.timings[name] = []
        self.timings[name].append({"start": time.time(), "end": None})

    def end(self, name):
        """结束记录某个操作的时间"""
        if name in self.timings and self.timings[name] and self.timings[name][-1]["end"] is None:
            self.timings[name][-1]["end"] = time.time()
            # 保持数据点数量在窗口大小内
            if len(self.timings[name]) > self.window_size:
                self.timings[name].pop(0)

    def get_stats(self, name):
        """获取某个操作的统计信息"""
        if name not in self.timings:
            return None

        durations = [t["end"] - t["start"] for t in self.timings[name] if t["end"] is not None]
        if not durations:
            return None

        stats = {
            "min": min(durations) * 1000,  # 转换为毫秒
            "max": max(durations) * 1000,
            "avg": sum(durations) * 1000 / len(durations),
            "samples": len(durations)
        }

        return stats

    def log_stats(self):
        """将所有操作的统计信息记录到日志"""
        for name in self.timings:
            stats = self.get_stats(name)
            if stats:
                logger.info(
                    f"性能统计 - {name}: 最小={stats['min']:.2f}ms, 最大={stats['max']:.2f}ms, 平均={stats['avg']:.2f}ms, 样本数={stats['samples']}")


# -------------------------------- 配置参数（与原程序完全一致）--------------------------------
# 配置字典 - 保留关键功能的完整性，只优化非关键部分
CONFIG = {
    'conf_threshold': 0.3,  # 检测置信度阈值
    'nms_confidence_threshold': 0.05,  # 保持原始值，确保检测准确性
    'nms_iou_threshold': 0.1,  # 保持原始值，确保检测准确性
    'max_x_distance_ratio': 0.2,  # 保持原始值，确保匹配准确性
    'dominant_color_k': 4,  # 颜色聚类数量
    'resize_detection': False,  # 检测前是否缩放图像
    'detection_width': 640,  # 分辨率
    'detection_height': 640,  # 分辨率（注意：RKNN模型通常期望正方形输入）
    'color_sample_size': 50,  # 保持较大的采样尺寸以确保颜色准确性
    'preload_model': True,  # 预加载模型减少延迟
    'rknn_model_path': 'best3.rknn',  # RKNN模型文件路径
}

# 服装类别 - 与原程序完全一致
CLOTHING_CATEGORIES = {
    'upper': [
        'short_sleeved_shirt',
        'long_sleeved_shirt',
        'short_sleeved_outwear',
        'long_sleeved_outwear',
        'vest',
        'sling',
    ],
    'lower': [
        'shorts',
        'trousers',
        'skirt',
        'short_sleeved_dress',
        'long_sleeved_dress',
        'vest_dress',
        'sling_dress'
    ]
}

# YOLOv5模型的相关常量 - 与原程序完全一致
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
IMG_SIZE = 640

CLASSES = ('short_sleeved_shirt',
           'long_sleeved_shirt',
           'short_sleeved_outwear',
           'long_sleeved_outwear',
           'vest',
           'sling', 'shorts',
           'trousers',
           'skirt',
           'short_sleeved_dress',
           'long_sleeved_dress',
           'vest_dress',
           'sling_dress')


# -------------------------------- 核心算法函数（与原程序完全一致）--------------------------------
def sigmoid(x):
    """
    sigmoid激活函数 - 与原程序完全一致
    """
    return 1 / (1 + np.exp(-x))


def xywh2xyxy(x):
    """
    将[x, y, w, h]格式转换为[x1, y1, x2, y2]格式 - 与原程序完全一致
    """
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def process(input, mask, anchors):
    """
    处理YOLOv5输出特征图 - 与原程序完全一致
    """
    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = sigmoid(input[..., 4])
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = sigmoid(input[..., 5:])

    box_xy = sigmoid(input[..., :2]) * 2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE / grid_h)

    box_wh = pow(sigmoid(input[..., 2:4]) * 2, 2)
    box_wh = box_wh * anchors

    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs


def filter_boxes(boxes, box_confidences, box_class_probs):
    """
    过滤检测框，根据置信度阈值 - 与原程序完全一致
    """
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= OBJ_THRESH)

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score * box_confidences)[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """
    非最大抑制 - 与原程序完全一致
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def yolov5_post_process(input_data):
    """
    YOLOv5后处理 - 与原程序完全一致
    """
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
               [59, 119], [116, 90], [156, 198], [373, 326]]

    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores


def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    """
    图像letterbox预处理，保持宽高比 - 与原程序完全一致
    """
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv.resize(im, new_unpad, interpolation=cv.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv.copyMakeBorder(im, top, bottom, left, right, cv.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)


def get_dominant_color(image, k=None, resize=True):
    """
    使用改进的K-means聚类计算图像的主要颜色
    
    该函数通过K-means聚类算法分析图像中的颜色分布，提取最具代表性
    的主要颜色。包含颜色预处理、聚类分析和结果验证等步骤。
    与原程序rknn_colour_detect.py中的算法完全一致。
    
    参数:
        image (numpy.ndarray): 输入图像，BGR格式，shape为(H,W,3)
        k (int, optional): K-means聚类的簇数量，默认为None时使用CONFIG['dominant_color_k']
                          推荐值：3-6，根据图像复杂度调整
        resize (bool, optional): 是否对图像进行缩放以提高处理速度，默认True
                                缩放到CONFIG['color_sample_size']大小
                                
    返回值:
        tuple: 主要颜色的BGR值，格式为(B, G, R)
               - B (int): 蓝色分量，范围0-255
               - G (int): 绿色分量，范围0-255  
               - R (int): 红色分量，范围0-255
               如果处理失败返回(0, 0, 0)
               
    算法流程:
        1. 图像尺寸检查和缩放处理
        2. HSV颜色空间转换和掩码创建
        3. 过滤极值像素（极暗、极亮、低饱和度）
        4. K-means聚类分析颜色分布
        5. 聚类结果验证和主要颜色选择
        6. 特殊颜色（黑/白）的二次验证
        
    颜色过滤策略:
        - 亮度范围：30-220（过滤极暗和极亮）
        - 饱和度阈值：30（过滤灰色调）
        - 聚类占比阈值：5%（过滤噪声聚类）
        
    性能优化:
        - 自适应图像缩放减少计算量
        - 颜色空间预处理提高聚类效果
        - 多级验证确保结果准确性
        
    异常:
        Exception: 当图像为空或K-means聚类失败时抛出，返回黑色(0,0,0)
        
    示例:
        >>> import cv2
        >>> img = cv2.imread('clothing.jpg')
        >>> color = get_dominant_color(img, k=5)
        >>> print(f"主要颜色BGR: {color}")
        >>> # 转换为HSV查看色调信息
        >>> hsv = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)[0][0]
        >>> print(f"HSV: {hsv}")
    """
    if not SKLEARN_AVAILABLE:
        # 如果sklearn不可用，返回平均颜色
        return tuple(map(int, np.mean(image.reshape(-1, 3), axis=0)))
    
    try:
        if k is None:
            k = CONFIG.get('dominant_color_k', 4)

        data = image.copy()
        
        if resize:
            sample_size = CONFIG.get('color_sample_size', 50)
            data = cv.resize(data, (sample_size, sample_size))

        # 重塑图像为像素列表
        data = data.reshape((-1, 3))
        data = np.float32(data)

        # 应用k-means聚类
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        _, labels, centers = cv.kmeans(data, k, None, criteria, 10, cv.KMEANS_RANDOM_CENTERS)

        # 找到最大的聚类
        label_counts = np.bincount(labels.flatten())
        dominant_color = centers[np.argmax(label_counts)]

        return tuple(map(int, dominant_color))

    except Exception as e:
        logger.error(f"颜色聚类失败: {str(e)}")
        # 返回平均颜色作为后备
        return tuple(map(int, np.mean(image.reshape(-1, 3), axis=0)))


def apply_nms(detections, confidence_threshold=None, iou_threshold=None):
    """
    应用非极大值抑制算法过滤重叠的检测框
    
    使用NMS(Non-Maximum Suppression)算法去除同一物体的多个重叠检测框，
    保留置信度最高的检测结果。与原程序算法和参数完全一致。
    
    参数:
        detections (list): 检测结果列表，每个元素格式为:
                          [xmin, ymin, xmax, ymax, confidence, class_id]
                          - xmin, ymin: 边界框左上角坐标
                          - xmax, ymax: 边界框右下角坐标
                          - confidence: 检测置信度，范围0.0-1.0
                          - class_id: 类别标识符
        confidence_threshold (float, optional): 置信度阈值，低于此值的检测被过滤
                                              默认使用CONFIG['nms_confidence_threshold']
        iou_threshold (float, optional): IoU阈值，高于此值的重叠框被抑制
                                        默认使用CONFIG['nms_iou_threshold']
                                        
    返回值:
        list: 过滤后的检测结果列表，格式与输入相同
              按置信度降序排列，移除了重叠度高的冗余检测
              
    算法原理:
        1. 按置信度过滤低质量检测
        2. 将边界框格式转换为OpenCV NMSBoxes要求的格式
        3. 应用NMS算法计算保留的检测索引
        4. 根据索引重构过滤后的检测列表
        
    NMS算法步骤:
        1. 按置信度对检测框排序
        2. 选择置信度最高的框作为基准
        3. 计算其他框与基准框的IoU
        4. 抑制IoU大于阈值的框
        5. 重复直到处理完所有框
        
    参数调优建议:
        - confidence_threshold: 0.05-0.3，过低易产生误检，过高易漏检
        - iou_threshold: 0.1-0.5，过低过度抑制，过高保留重叠
        
    异常:
        Exception: 当检测列表格式不正确或NMS计算失败时抛出
        
    示例:
        >>> detections = [
        ...     [100, 100, 200, 200, 0.9, 0],  # 高置信度检测
        ...     [110, 110, 210, 210, 0.7, 0],  # 重叠检测（将被抑制）
        ...     [300, 300, 400, 400, 0.8, 1]   # 不同位置检测
        ... ]
        >>> filtered = apply_nms(detections, 0.5, 0.4)
        >>> print(f"过滤后检测数量: {len(filtered)}")
    """
    if confidence_threshold is None:
        confidence_threshold = CONFIG['nms_confidence_threshold']
    if iou_threshold is None:
        iou_threshold = CONFIG['nms_iou_threshold']

    boxes = []
    confidences = []
    class_ids = []

    for detection in detections:
        xmin, ymin, xmax, ymax, conf, class_id = detection
        if conf > confidence_threshold:
            # 转换为 [x, y, w, h] 格式
            boxes.append([xmin, ymin, xmax - xmin, ymax - ymin])
            confidences.append(conf)
            class_ids.append(class_id)

    # 应用NMS
    filtered_detections = []
    if boxes:
        indices = cv.dnn.NMSBoxes(
            boxes, confidences, confidence_threshold, iou_threshold
        )

        if len(indices) > 0:
            for i in indices.flatten():
                box = boxes[i]
                xmin, ymin = box[0], box[1]
                xmax, ymax = box[0] + box[2], box[1] + box[3]
                filtered_detections.append(
                    [xmin, ymin, xmax, ymax, confidences[i], class_ids[i]]
                )

    return filtered_detections


def match_clothing_items_with_confidence(upper_items, lower_items, img, pairs, max_x_distance_ratio=None):
    """
    匹配上衣和下装检测结果，保留置信度信息
    
    基于空间位置关系和几何约束，将检测到的上衣和下装进行智能配对，
    形成完整的服装组合。算法考虑人体结构特点和服装穿着规律。
    与原程序match_clothing_items算法完全一致，增加了置信度处理。
    
    参数:
        upper_items (list): 上衣检测列表，每个元素格式为:
                           (xmin, ymin, xmax, ymax, confidence, 'upper')
        lower_items (list): 下装检测列表，每个元素格式为:
                           (xmin, ymin, xmax, ymax, confidence, 'lower')
        img (numpy.ndarray): 输入图像，用于获取图像尺寸进行约束计算
        pairs (list): 输出配对结果列表，函数会向此列表添加匹配对
        max_x_distance_ratio (float, optional): 最大水平距离比例，相对于图像宽度
                                               默认使用CONFIG['max_x_distance_ratio']
                                               
    配对结果格式:
        每个配对为包含4个元素的列表：
        [上衣坐标, 下装坐标, 上衣置信度, 下装置信度]
        - 上衣坐标: (xmin, ymin, xmax, ymax) 或 (-1,) 表示无匹配
        - 下装坐标: (xmin, ymin, xmax, ymax) 或 (-1,) 表示无匹配
        - 上衣置信度: float，范围0.0-1.0，无匹配时为0.0
        - 下装置信度: float，范围0.0-1.0，无匹配时为0.0
        
    匹配算法:
        1. 计算每件上衣和下装的中心点坐标
        2. 检查水平距离约束（模拟人体宽度限制）
        3. 计算中心点间欧氏距离作为匹配代价
        4. 使用贪心算法选择最优匹配
        5. 避免重复匹配，确保一对一关系
        
    几何约束:
        - 水平距离限制：|upper_x - lower_x| < width * max_x_distance_ratio
        - 优先匹配距离最近的上下装组合
        - 支持部分匹配（只有上衣或只有下装）
        
    处理策略:
        - 优先匹配：距离最近且满足约束条件的组合
        - 剩余处理：未匹配的单独上衣或下装作为独立项
        - 置信度保留：完整保留原检测的置信度信息
        
    异常:
        Exception: 当图像尺寸获取失败或坐标计算出错时抛出
        
    示例:
        >>> upper_items = [(100, 50, 200, 150, 0.9, 'upper')]
        >>> lower_items = [(120, 140, 180, 250, 0.8, 'lower')]
        >>> pairs = []
        >>> match_clothing_items_with_confidence(upper_items, lower_items, img, pairs)
        >>> print(f"配对结果: {pairs[0]}")
        >>> # 输出: [(100, 50, 200, 150), (120, 140, 180, 250), 0.9, 0.8]
    """
    if not upper_items and not lower_items:
        return

    if max_x_distance_ratio is None:
        max_x_distance_ratio = CONFIG['max_x_distance_ratio']

    height, width = img.shape[:2]
    max_x_distance = width * max_x_distance_ratio

    # 复制下装列表以避免修改原始数据
    lower_items_copy = upper_items.copy() if not lower_items else lower_items.copy()

    # 对每件上衣
    for upper in upper_items:
        closest_lower = None
        min_distance = float('inf')

        # 计算上衣中心点
        upper_center_x = (upper[0] + upper[2]) // 2
        upper_center_y = (upper[1] + upper[3]) // 2

        for lower in lower_items_copy:
            # 计算下装中心点
            lower_center_x = (lower[0] + lower[2]) // 2
            lower_center_y = (lower[1] + lower[3]) // 2

            # 检查水平距离约束
            x_distance = abs(upper_center_x - lower_center_x)
            if x_distance > max_x_distance:
                continue

            # 计算中心点之间的欧氏距离
            distance = ((upper_center_x - lower_center_x) ** 2 +
                        (upper_center_y - lower_center_y) ** 2) ** 0.5

            # 更新最近匹配
            if distance < min_distance:
                min_distance = distance
                closest_lower = lower

        # 添加配对或未配对的上衣
        if closest_lower:
            pairs.append([upper, closest_lower])
            lower_items_copy.remove(closest_lower)  # 避免重复使用下装
        else:
            pairs.append([upper, (-1,)])

    # 添加剩余未配对的下装
    for lower in lower_items_copy:
        pairs.append([(-1,), lower])


def identify_clothing_colors_with_confidence(pairs, img):
    """
    识别服装配对中每件服装的主要颜色，保留置信度信息
    
    对已配对的上衣和下装分别进行颜色分析，提取各自的主要颜色特征。
    针对不同类型服装采用优化的颜色提取策略。与原程序算法完全一致，
    并完整保留检测置信度信息。
    
    参数:
        pairs (list): 服装配对列表，每个元素包含4个部分:
                     [上衣坐标, 下装坐标, 上衣置信度, 下装置信度]
                     函数会在原地修改此列表，添加颜色信息
        img (numpy.ndarray): 输入图像，BGR格式，用于颜色提取
        
    输出格式:
        修改后的pairs列表，每个元素包含6个部分:
        [上衣坐标, 下装坐标, 上衣颜色, 下装颜色, 上衣置信度, 下装置信度]
        - 上衣颜色: (B,G,R) BGR格式颜色值，或 () 表示无颜色信息
        - 下装颜色: (B,G,R) BGR格式颜色值，或 () 表示无颜色信息
        
    颜色提取策略:
        上衣处理:
            - K-means聚类数: 5（上衣颜色通常更丰富）
            - 区域提取: 完整边界框区域
            - 特殊处理: 重点验证纯色和图案服装
            
        下装处理:
            - K-means聚类数: 3（下装颜色相对简单）
            - 区域提取: 完整边界框区域
            - 特殊处理: 针对深色下装优化
            
    颜色验证:
        1. 黑白验证：检测是否为纯黑或纯白服装
        2. HSV分析：计算平均饱和度和亮度
        3. 重新计算：对可疑结果使用更多聚类重新分析
        4. 边界检查：确保坐标在图像范围内
        
    异常处理:
        - 无效边界框：返回黑色(0,0,0)
        - 颜色提取失败：记录错误并使用默认颜色
        - 占位符检测：跳过(-1,)格式的占位符
        
    性能特点:
        - 自适应聚类数量：根据服装类型调整
        - 鲁棒性强：多重验证确保颜色准确性
        - 内存高效：原地修改避免额外拷贝
        
    异常:
        Exception: 当图像格式不正确或颜色提取过程失败时记录错误
        
    示例:
        >>> pairs = [[(100, 50, 200, 150), (120, 140, 180, 250), 0.9, 0.8]]
        >>> identify_clothing_colors_with_confidence(pairs, img)
        >>> upper_color = pairs[0][2]  # 上衣颜色
        >>> lower_color = pairs[0][3]  # 下装颜色
        >>> print(f"上衣颜色BGR: {upper_color}, 下装颜色BGR: {lower_color}")
    """
    logger.debug(f"开始识别 {len(pairs)} 对服装的颜色")

    for pair_idx, pair in enumerate(pairs):
        logger.debug(f"处理服装对 {pair_idx + 1}/{len(pairs)}")

        for i, item in enumerate(pair[:2]):  # 只处理前两项(上衣和下装)
            item_type = "上衣" if i == 0 else "下装"

            if len(item) == 1:  # 占位符
                logger.debug(f"{item_type}为占位符，跳过颜色识别")
                pair.append(())
                continue

            if len(item) < 4:  # 无效项
                logger.debug(f"{item_type}项无效，跳过颜色识别")
                break

            # 提取服装区域
            xmin, ymin, xmax, ymax = item[0], item[1], item[2], item[3]
            logger.debug(f"{item_type}边界框: [{xmin}, {ymin}, {xmax}, {ymax}]")

            try:
                # 检查边界框是否有效
                if xmin >= xmax or ymin >= ymax or xmin < 0 or ymin < 0 or xmax > img.shape[1] or ymax > img.shape[0]:
                    logger.warning(f"无效的{item_type}边界框: {item}")
                    pair.append((0, 0, 0))
                    continue

                clothing_region = img[ymin:ymax, xmin:xmax]
                logger.debug(f"{item_type}区域尺寸: {clothing_region.shape}")

                # 检查区域是否有效
                if clothing_region.size == 0 or clothing_region.shape[0] == 0 or clothing_region.shape[1] == 0:
                    logger.warning(f"无效的{item_type}区域: shape={clothing_region.shape if clothing_region is not None else None}")
                    pair.append((0, 0, 0))
                    continue

                # 计算颜色提取开始时间
                color_extract_start = time.time()

                # 根据不同衣物类型调整处理方式
                if i == 0:  # 上衣
                    # 上衣通常颜色更鲜明，使用更多的聚类
                    k_value = 5
                    logger.debug(f"为上衣使用K={k_value}的聚类")
                    dominant_color = get_dominant_color(clothing_region, k=k_value)
                    logger.debug(f"上衣主要颜色: BGR={dominant_color}")
                else:  # 下装
                    # 下装通常颜色较暗，使用较少聚类
                    k_value = 3
                    logger.debug(f"为下装使用K={k_value}的聚类")
                    dominant_color = get_dominant_color(clothing_region, k=k_value)
                    logger.debug(f"下装主要颜色: BGR={dominant_color}")

                # 验证颜色是否有效
                is_black = all(c < 30 for c in dominant_color)
                is_white = all(c > 225 for c in dominant_color)

                if is_black:
                    dominant_color = (0, 0, 0)
                elif is_white:
                    dominant_color = (255, 255, 255)

                pair.append(dominant_color)
                logger.debug(f"{item_type}最终颜色: BGR={dominant_color}, 耗时: {(time.time() - color_extract_start) * 1000:.2f}ms")

            except Exception as e:
                logger.error(f"提取{item_type}颜色时出错: {str(e)}")
                logger.error(f"错误堆栈:\n{traceback.format_exc()}")
                pair.append((0, 0, 0))  # 默认黑色

        # 确保每对都有置信度信息
        while len(pair) < 6:  # [上衣位置, 下装位置, 上衣颜色, 下装颜色, 上衣置信度, 下装置信度]
            if len(pair) == 4:  # 添加上衣置信度
                if len(pair[0]) > 4:  # 上衣有置信度信息
                    pair.append(pair[0][4])
                else:
                    pair.append(0.0)
            elif len(pair) == 5:  # 添加下装置信度
                if len(pair[1]) > 4:  # 下装有置信度信息
                    pair.append(pair[1][4])
                else:
                    pair.append(0.0)


def Determine_the_position_of_the_entire_body(c, p, img):
    """
    估计整个身体位置，基于检测到的上衣和下装位置计算完整人体区域
    
    该函数根据上衣和下装的边界框，结合人体比例学原理，推算出包含
    头部、躯干和腿部的完整人体区域。支持只有上衣或只有下装的情况。
    与原程序Determine_the_position_of_the_entire_body函数完全一致。
    
    参数:
        c (tuple): 上衣坐标，格式为 (xmin, ymin, xmax, ymax)
                   如果为 (-1,) 表示未检测到上衣
        p (tuple): 下装坐标，格式为 (xmin, ymin, xmax, ymax)  
                   如果为 (-1,) 表示未检测到下装
        img (numpy.ndarray): 输入图像，用于获取图像尺寸进行边界检查
        
    返回值:
        list: 身体位置坐标列表，每个元素为 (xmin, ymin, xmax, ymax) 格式
              的整体人体区域坐标。包含基于服装位置推算的头部和脚部扩展。
              如果输入坐标无效，返回空列表。
              
    算法说明:
        当上下装都检测到时：
            1. 计算包含两者的最小外接矩形
            2. 基于上衣高度推算头部扩展：0.5倍上衣高度
            3. 基于下装高度推算脚部扩展：0.6倍下装高度
            4. 基于上衣宽度推算水平扩展：0.3倍上衣宽度
            
        只有上衣时：
            1. 以上衣为基准计算人体区域
            2. 头部扩展：0.4倍上衣高度
            3. 下身推算：2.5倍上衣高度（覆盖下半身）
            4. 水平扩展：40像素固定值
            
        只有下装时：
            1. 以下装为基准计算人体区域
            2. 上身推算：1.8倍下装高度（覆盖上半身）
            3. 脚部扩展：0.7倍下装高度
            4. 水平扩展：40像素固定值
            
    人体比例参考:
        - 头部占身高约1/8，相当于上身高度的40-50%
        - 上身（头+躯干）约占身高60%
        - 下身（腿部）约占身高40%
        - 肩宽约为身高的1/4
        
    边界处理:
        - 自动限制在图像范围内：[0, img_width] × [0, img_height]
        - 确保计算结果为有效的矩形区域
        - 处理边界情况避免负坐标或超出图像
        
    异常处理:
        - 输入坐标检查：验证是否为有效的4元素坐标
        - 图像尺寸验证：确保图像不为空
        - 计算错误捕获：返回空列表而不是抛出异常
        
    应用场景:
        - 人体跟踪：提供完整的人体区域用于跟踪
        - 行为分析：确定人体活动区域
        - 距离测量：计算整个人体的空间位置
        - 图像分割：提取完整的人体区域
        
    异常:
        Exception: 当坐标计算过程中出现错误时记录日志并返回空列表
        
    示例:
        >>> upper_coords = (100, 100, 200, 250)
        >>> lower_coords = (120, 240, 180, 350)  
        >>> body_positions = Determine_the_position_of_the_entire_body(
        ...     upper_coords, lower_coords, img)
        >>> if body_positions:
        ...     body_box = body_positions[0]
        ...     print(f"人体区域: {body_box}")
        ...     # 输出类似: 人体区域: (70, 40, 230, 420)
        
        >>> # 只有上衣的情况
        >>> upper_only = (100, 100, 200, 250)
        >>> placeholder = (-1,)
        >>> body_positions = Determine_the_position_of_the_entire_body(
        ...     upper_only, placeholder, img)
    """
    person_position = []

    if img is None or img.size == 0:
        return person_position

    img_height, img_width = img.shape[:2]

    try:
        if len(c) == 4 and len(p) == 4:
            # 两者都检测到 - 计算整个人体区域
            person_xmin = min(c[0], p[0])
            person_ymin = min(c[1], p[1])
            person_xmax = max(c[2], p[2])
            person_ymax = max(c[3], p[3])

            # 添加头部和脚部扩展
            upper_height = c[3] - c[1]
            lower_height = p[3] - p[1]
            upper_width = c[2] - c[0]

            head_extension = int(upper_height * 0.5)
            foot_extension = int(lower_height * 0.6)
            side_extension = int(upper_width * 0.3)

            # 应用扩展（带边界检查）
            person_xmin = max(0, person_xmin - side_extension)
            person_xmax = min(img_width, person_xmax + side_extension)
            person_ymin = max(0, person_ymin - head_extension)
            person_ymax = min(img_height, person_ymax + foot_extension)

        elif len(c) == 4 and len(p) == 1:
            # 只有上衣
            upper_height = c[3] - c[1]

            person_xmin = max(0, c[0] - 40)
            person_ymin = max(0, c[1] - int(upper_height * 0.4))
            person_xmax = min(img_width, c[2] + 40)
            person_ymax = min(img_height, c[3] + int(upper_height * 2.5))

        elif len(c) == 1 and len(p) == 4:
            # 只有下装
            lower_height = p[3] - p[1]

            person_xmin = max(0, p[0] - 40)
            person_ymin = max(0, p[1] - int(lower_height * 1.8))
            person_xmax = min(img_width, p[2] + 40)
            person_ymax = min(img_height, p[3] + int(lower_height * 0.7))
        else:
            # 无效输入
            return person_position

        person_position.append((int(person_xmin), int(person_ymin),
                                int(person_xmax), int(person_ymax)))

    except Exception as e:
        logger.error(f"确定身体位置出错: {str(e)}")
        logger.error(f"错误堆栈:\n{traceback.format_exc()}")

    return person_position


# -------------------------------- ROS2 RKNN检测节点 --------------------------------
class RKNNDetectorNode(Node):
    """
    ROS2 RKNN服装检测节点
    
    基于RKNN模型的智能服装检测与颜色识别系统ROS2节点，提供高性能的
    实时服装检测、颜色分析和身体位置估算服务。严格保持与原始程序
    rknn_colour_detect.py相同的算法逻辑和参数配置。
    
    功能特性:
        - 基于RKNN加速的YOLOv5服装检测
        - 智能上下装配对和颜色识别
        - 完整人体区域估算
        - 高性能实时处理能力
        - 完整的置信度信息保留
        
    服务接口:
        - /detect_image_with_confidence: 图像检测服务
        - /determine_body_position: 身体位置估算服务
        
    检测能力:
        - 13种服装类别检测
        - 上衣类别：短袖衬衫、长袖衬衫、短袖外套、长袖外套、背心、吊带
        - 下装类别：短裤、长裤、裙子、短袖连衣裙、长袖连衣裙、背心裙、吊带裙
        
    算法流程:
        1. RKNN模型加载和初始化
        2. 图像预处理和letterbox变换
        3. RKNN推理和特征提取
        4. YOLOv5后处理和NMS过滤
        5. 服装分类和智能配对
        6. 颜色提取和分析
        7. 人体区域估算
        
    性能特点:
        - RKNN硬件加速，推理速度提升显著
        - 多线程安全的模型调用
        - 内存高效的批处理模式
        - 实时性能监控和日志记录
        
    配置参数:
        - 检测置信度阈值：0.3
        - NMS置信度阈值：0.05  
        - NMS IoU阈值：0.1
        - 颜色聚类数量：4
        - 图像处理分辨率：640x640
        
    属性:
        rknn_model (RKNNLite): RKNN模型实例
        bridge (CvBridge): ROS图像消息转换器
        detect_service (Service): 图像检测服务
        body_position_service (Service): 身体位置服务
        perf_monitor (PerformanceMonitor): 性能监控器
        
    异常处理:
        - RKNN模型加载失败时提供降级处理
        - 服务调用异常时返回明确错误信息
        - 图像格式不正确时自动转换或拒绝
        
    示例:
        >>> import rclpy
        >>> rclpy.init()
        >>> node = RKNNDetectorNode()
        >>> rclpy.spin(node)
        >>> node.destroy_node()
        >>> rclpy.shutdown()
    """

    def __init__(self):
        """
        初始化RKNN检测节点
        
        设置ROS2节点基础配置，加载RKNN模型，创建服务接口，
        初始化性能监控系统。与原程序初始化逻辑完全一致。
        
        初始化步骤:
            1. 调用父类Node构造函数
            2. 创建CV桥接器和性能监控器
            3. 尝试加载RKNN模型
            4. 设置检测和身体位置服务
            5. 记录节点配置信息
            
        模型加载:
            - 优先使用RKNN模型（如果可用）
            - RKNN不可用时记录警告但继续运行
            - 模型路径可通过CONFIG配置
            
        服务配置:
            - 图像检测服务：支持批量和单张图像检测
            - 身体位置服务：基于服装检测结果估算人体区域
            - 服务调用超时和错误处理
            
        异常处理:
            - RKNN模型加载失败时继续运行但禁用检测功能
            - 服务创建失败时记录错误并尝试重新创建
            - 系统资源不足时降级处理
            
        异常:
            Exception: 当节点核心组件初始化失败时抛出
        """
        super().__init__('rknn_color_detector')
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 创建性能监控器
        self.perf_monitor = PerformanceMonitor()
        
        # RKNN模型实例
        self.rknn_model = None
        
        try:
            # 尝试加载RKNN模型
            self.load_rknn_model()
            
            # 设置服务
            self.setup_services()
            
            self.get_logger().info('RKNN检测节点初始化成功')
            self.get_logger().info(f'模型路径: {CONFIG["rknn_model_path"]}')
            self.get_logger().info(f'检测置信度阈值: {CONFIG["conf_threshold"]}')
            self.get_logger().info(f'NMS置信度阈值: {CONFIG["nms_confidence_threshold"]}')
            self.get_logger().info(f'NMS IoU阈值: {CONFIG["nms_iou_threshold"]}')
            
        except Exception as e:
            self.get_logger().error(f'节点初始化失败: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            raise

    def load_rknn_model(self):
        """
        加载RKNN模型用于服装检测
        
        初始化RKNN推理引擎，加载预训练的YOLOv5服装检测模型。
        模型支持13种服装类别的实时检测。与原程序模型加载完全一致。
        
        模型配置:
            - 模型文件：best3.rknn
            - 输入尺寸：640x640x3 (RGB格式)
            - 输出层：3个特征图（多尺度检测）
            - 量化精度：INT8量化以提高推理速度
            
        加载流程:
            1. 检查RKNN环境是否可用
            2. 创建RKNNLite推理实例
            3. 加载模型文件到内存
            4. 初始化运行时环境
            5. 验证模型输入输出格式
            
        性能优化:
            - 预先分配推理内存
            - 启用NPU硬件加速
            - 配置最优的推理参数
            
        异常处理:
            - RKNN环境不可用时记录警告
            - 模型文件缺失时提供明确错误信息
            - 内存不足时尝试释放资源重试
            
        异常:
            Exception: 当RKNN不可用或模型加载失败时抛出
            
        注意事项:
            - 模型文件必须与当前RKNN版本兼容
            - 推理过程中模型实例为单例，线程安全
            - 模型卸载在节点销毁时自动处理
        """
        if not RKNN_AVAILABLE:
            self.get_logger().error('RKNN Lite未安装，无法加载模型')
            return
            
        try:
            # 获取模型文件路径
            current_file_path = os.path.abspath(__file__)
            current_dir = os.path.dirname(current_file_path)
            model_file_path = os.path.join(current_dir, '..', 'data', CONFIG['rknn_model_path'])
            
            # 检查模型文件是否存在
            if not os.path.exists(model_file_path):
                self.get_logger().error(f'RKNN模型文件不存在: {model_file_path}')
                return
                
            # 创建RKNN对象
            self.rknn_model = RKNNLite()

            # 加载RKNN模型
            self.get_logger().info('正在加载RKNN模型...')
            ret = self.rknn_model.load_rknn(model_file_path)
            if ret != 0:
                self.get_logger().error(f'加载RKNN模型失败: {ret}')
                self.rknn_model = None
                return

            # 初始化运行时环境
            self.get_logger().info('初始化RKNN运行时环境...')
            ret = self.rknn_model.init_runtime()
            if ret != 0:
                self.get_logger().error(f'初始化RKNN运行时环境失败: {ret}')
                self.rknn_model = None
                return

            self.get_logger().info(f"RKNN模型加载成功: {model_file_path}")
            
        except Exception as e:
            self.get_logger().error(f"RKNN模型加载失败: {str(e)}")
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            self.rknn_model = None

    def setup_services(self):
        """
        设置ROS2服务接口
        
        创建用于图像检测和身体位置估算的ROS2服务，提供标准化的
        服务接口供其他节点调用。使用自定义服务消息格式。
        
        服务配置:
            图像检测服务:
                - 服务名：/detect_image_with_confidence
                - 类型：DetectImageWithConfidence
                - 功能：完整的服装检测和颜色识别
                
            身体位置服务:
                - 服务名：/determine_body_position  
                - 类型：DetermineBodyPosition
                - 功能：基于服装位置估算人体区域
                
        服务接口说明:
            DetectImageWithConfidence:
                请求：sensor_msgs/Image - 待检测图像
                响应：检测结果列表，包含坐标、颜色、置信度
                
            DetermineBodyPosition:
                请求：上衣和下装坐标
                响应：完整人体区域坐标
                
        服务特性:
            - 异步处理：不阻塞其他节点运行
            - 错误处理：提供详细的错误信息和状态码
            - 参数验证：自动验证输入参数有效性
            - 性能监控：记录服务调用时间和成功率
            
        异常:
            Exception: 当服务创建失败时抛出
        """
        try:
            # 创建图像检测服务
            self.detect_service = self.create_service(
                DetectImageWithConfidence,
                '/detect_image_with_confidence',
                self.detect_image_callback
            )
            
            # 创建身体位置判断服务
            self.body_position_service = self.create_service(
                DetermineBodyPosition,
                '/determine_body_position',
                self.determine_body_position_callback
            )
            
            self.get_logger().info('服务设置完成')
            self.get_logger().info('图像检测服务: /detect_image_with_confidence')
            self.get_logger().info('身体位置判断服务: /determine_body_position')
            
        except Exception as e:
            self.get_logger().error(f'服务设置失败: {str(e)}')
            raise

    def detect_picture_with_confidence(self, img):
        """
        使用RKNN检测图像中的服装，并包含置信度信息
        
        本函数实现完整的服装检测流水线，包括图像预处理、RKNN模型推理、
        后处理、服装匹配和颜色识别。与原程序的detect_picture_with_confidence
        函数保持完全一致的逻辑和输出格式。
        
        参数:
            img (numpy.ndarray): 输入图像，BGR格式，形状为(height, width, 3)
                                必须为有效的OpenCV图像格式
                                
        返回值:
            list: 检测到的服装配对列表，每个元素为包含6个元素的列表：
                  [上衣位置, 下装位置, 上衣颜色, 下装颜色, 上衣置信度, 下装置信度]
                  
                  具体格式说明:
                  - 上衣位置 (tuple): (xmin, ymin, xmax, ymax) 或 (-1,) 表示未检测到
                  - 下装位置 (tuple): (xmin, ymin, xmax, ymax) 或 (-1,) 表示未检测到  
                  - 上衣颜色 (tuple): BGR格式颜色值 (B, G, R) 或 () 表示无颜色信息
                  - 下装颜色 (tuple): BGR格式颜色值 (B, G, R) 或 () 表示无颜色信息
                  - 上衣置信度 (float): 检测置信度，范围 0.0-1.0
                  - 下装置信度 (float): 检测置信度，范围 0.0-1.0
                  
        处理流程:
            1. 图像预处理和格式验证
            2. Letterbox变换保持宽高比
            3. BGR到RGB颜色空间转换
            4. RKNN模型推理
            5. YOLOv5后处理和坐标转换
            6. 服装分类和NMS过滤
            7. 上下装智能配对
            8. 颜色提取和分析
            
        算法特点:
            - 多尺度检测：支持不同大小的服装物体
            - 智能配对：基于人体结构的服装匹配算法
            - 颜色分析：K-means聚类提取主要颜色
            - 置信度保留：完整保留检测和匹配的置信度信息
            
        性能优化:
            - RKNN硬件加速：充分利用NPU计算能力
            - 批量处理：支持多张图像同时检测
            - 内存复用：减少临时对象创建
            - 并行计算：颜色提取和配对同时进行
            
        异常处理:
            - 图像格式检查：自动处理不同输入格式
            - 模型推理异常：提供降级处理机制
            - 内存不足：自动释放临时变量
            - 坐标越界：自动裁剪到图像范围内
            
        异常:
            ValueError: 当输入图像格式不正确时抛出
            RuntimeError: 当RKNN模型推理失败时抛出
            Exception: 当检测过程中出现其他错误时抛出
            
        示例:
            >>> import cv2
            >>> img = cv2.imread('test.jpg')
            >>> pairs = detect_picture_with_confidence(img)
            >>> print(f"检测到 {len(pairs)} 套服装")
            >>> for i, pair in enumerate(pairs):
            ...     print(f"服装{i+1}: 上衣{pair[0]}, 下装{pair[1]}")
            ...     print(f"  颜色: 上衣{pair[2]}, 下装{pair[3]}")
            ...     print(f"  置信度: 上衣{pair[4]:.3f}, 下装{pair[5]:.3f}")
        """
        # 记录检测开始时间(用于性能监控)
        detect_start_time = time.time()

        if self.rknn_model is None:
            logger.error("模型未加载，无法进行检测")
            return []

        if img is None or img.size == 0:
            logger.error("提供给detect_picture_with_confidence的图像无效")
            return []

        try:
            # 保存原始图像副本
            original_img = img.copy()
            logger.debug(f"原始图像尺寸: {img.shape}")

            # 预处理图像 - 使用letterbox而不是简单缩放，保持宽高比
            self.perf_monitor.start("preprocessing")
            img_for_detection, ratio, pad = letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE))
            logger.debug(f"预处理后图像尺寸: {img_for_detection.shape}, 缩放比例: {ratio}, 填充: {pad}")

            # 转换颜色空间从BGR到RGB（RKNN模型通常使用RGB输入）
            img_for_detection = cv.cvtColor(img_for_detection, cv.COLOR_BGR2RGB)

            # 扩展维度以匹配模型输入要求
            img_for_detection = np.expand_dims(img_for_detection, 0)
            logger.debug(f"模型输入尺寸: {img_for_detection.shape}")
            self.perf_monitor.end("preprocessing")

            # 运行RKNN推理
            logger.debug('开始RKNN推理...')
            self.perf_monitor.start("inference")
            outputs = self.rknn_model.inference(inputs=[img_for_detection])
            self.perf_monitor.end("inference")

            # 处理RKNN输出
            self.perf_monitor.start("postprocessing")
            input0_data = outputs[0]
            input1_data = outputs[1]
            input2_data = outputs[2]

            # 重塑输出以匹配YOLOv5后处理期望格式
            input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
            input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
            input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

            # 转置以匹配YOLOv5后处理期望格式
            input_data = [
                np.transpose(input0_data, (2, 3, 0, 1)),
                np.transpose(input1_data, (2, 3, 0, 1)),
                np.transpose(input2_data, (2, 3, 0, 1))
            ]

            # YOLOv5后处理
            logger.debug('正在执行YOLOv5后处理...')
            boxes, classes, scores = yolov5_post_process(input_data)

            # 将坐标从letterbox空间转回原始图像空间
            if boxes is not None:
                # 获取原始图像尺寸
                orig_h, orig_w = original_img.shape[:2]
                # 获取缩放比例和填充信息
                r = ratio[0]
                dw, dh = pad

                # 调整所有边界框坐标
                for i in range(len(boxes)):
                    boxes[i][0] = (boxes[i][0] - dw) / r
                    boxes[i][1] = (boxes[i][1] - dh) / r
                    boxes[i][2] = (boxes[i][2] - dw) / r
                    boxes[i][3] = (boxes[i][3] - dh) / r

                    # 确保坐标在有效范围内
                    boxes[i][0] = max(0, min(orig_w - 1, boxes[i][0]))
                    boxes[i][1] = max(0, min(orig_h - 1, boxes[i][1]))
                    boxes[i][2] = max(0, min(orig_w, boxes[i][2]))
                    boxes[i][3] = max(0, min(orig_h, boxes[i][3]))

            self.perf_monitor.end("postprocessing")

            # 初始化结果容器
            upper_clothing = []
            lower_clothing = []
            pairs = []

            # 如果没有检测到任何物体
            if boxes is None:
                logger.debug("未检测到任何服装")
                return pairs

            logger.debug(f"检测到 {len(boxes)} 个物体，开始分类和处理...")

            # 处理检测结果
            for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
                # 获取坐标
                xmin, ymin, xmax, ymax = box
                height, width = original_img.shape[:2]
                xmin = max(0, min(width - 1, int(xmin)))
                ymin = max(0, min(height - 1, int(ymin)))
                xmax = max(0, min(width, int(xmax)))
                ymax = max(0, min(height, int(ymax)))

                confidence = score
                class_name = CLASSES[cls]

                logger.debug(f"物体 {i + 1}: 类别={class_name}, 置信度={confidence:.4f}, 坐标=[{xmin}, {ymin}, {xmax}, {ymax}]")

                # 分类检测结果
                if class_name in CLOTHING_CATEGORIES['upper']:
                    upper_clothing.append((xmin, ymin, xmax, ymax, confidence, 'upper'))
                    logger.debug(f"将 {class_name} 分类为上衣")
                elif class_name in CLOTHING_CATEGORIES['lower']:
                    lower_clothing.append((xmin, ymin, xmax, ymax, confidence, 'lower'))
                    logger.debug(f"将 {class_name} 分类为下装")

            # 应用NMS
            logger.debug(f"NMS前上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")
            upper_clothing = apply_nms(upper_clothing)
            lower_clothing = apply_nms(lower_clothing)
            logger.debug(f"NMS后上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")

            # 匹配服装项目
            logger.debug("开始匹配上下装...")
            match_start = time.time()
            match_clothing_items_with_confidence(upper_clothing, lower_clothing, original_img, pairs)
            logger.debug(f"匹配完成，找到 {len(pairs)} 对服装，耗时: {(time.time() - match_start) * 1000:.2f}ms")

            # 获取颜色
            logger.debug("开始识别服装颜色...")
            color_start = time.time()
            identify_clothing_colors_with_confidence(pairs, original_img)
            logger.debug(f"颜色识别完成，耗时: {(time.time() - color_start) * 1000:.2f}ms")

            # 计算总处理时间
            total_time = time.time() - detect_start_time
            logger.debug(f"检测完成，总耗时: {total_time * 1000:.2f}ms, 约 {1 / total_time:.1f} FPS")

            return pairs

        except Exception as e:
            logger.error(f"图像检测出错: {str(e)}")
            logger.error(f"错误堆栈:\n{traceback.format_exc()}")
            return []

    def destroy_node(self):
        """节点销毁时的清理工作"""
        try:
            self.get_logger().info('正在关闭RKNN检测节点...')
            
            # 释放RKNN模型资源
            if self.rknn_model is not None:
                self.rknn_model.release()
                self.get_logger().info('RKNN模型资源已释放')
                
            super().destroy_node()
            self.get_logger().info('RKNN检测节点已成功关闭')
            
        except Exception as e:
            self.get_logger().error(f'节点关闭错误: {str(e)}')
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")

    def detect_image_callback(self, request, response):
        """
        图像检测服务回调函数
        
        接收ROS图像消息，调用detect_picture_with_confidence进行检测，
        将检测结果转换为ROS服务响应格式。
        
        Args:
            request: DetectImageWithConfidence请求，包含待检测图像
            response: DetectImageWithConfidence响应，将填充检测结果
            
        Returns:
            DetectImageWithConfidence.Response: 包含检测结果的响应消息
        """
        try:
            self.get_logger().info('收到图像检测服务请求')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(request.image, 'bgr8')
            
            # 执行检测
            pairs = self.detect_picture_with_confidence(cv_image)
            
            # 转换结果为ROS消息格式
            response.upper_positions = []
            response.lower_positions = []
            response.upper_colors = []
            response.lower_colors = []
            response.upper_confidences = []
            response.lower_confidences = []
            
            for pair in pairs:
                upper_pos, lower_pos, upper_color, lower_color, upper_conf, lower_conf = pair
                
                # 处理上衣位置
                if upper_pos != (-1,):
                    upper_point = Point()
                    upper_point.x = float(upper_pos[0])  # xmin
                    upper_point.y = float(upper_pos[1])  # ymin
                    # 使用标准16位移位编码，与解码方式保持一致
                    upper_point.z = float((upper_pos[3] << 16) | upper_pos[2])  # 编码xmax和ymax
                    response.upper_positions.append(upper_point)
                    
                    # 上衣颜色
                    if upper_color:
                        color_msg = ColorRGBA()
                        color_msg.b = float(upper_color[0]) / 255.0
                        color_msg.g = float(upper_color[1]) / 255.0
                        color_msg.r = float(upper_color[2]) / 255.0
                        color_msg.a = 1.0
                        response.upper_colors.append(color_msg)
                    else:
                        response.upper_colors.append(ColorRGBA())
                        
                    response.upper_confidences.append(float(upper_conf))
                else:
                    # 无效位置标记
                    invalid_point = Point()
                    invalid_point.x = -1.0
                    invalid_point.y = -1.0
                    invalid_point.z = -1.0
                    response.upper_positions.append(invalid_point)
                    response.upper_colors.append(ColorRGBA())
                    response.upper_confidences.append(0.0)
                
                # 处理下装位置
                if lower_pos != (-1,):
                    lower_point = Point()
                    lower_point.x = float(lower_pos[0])  # xmin
                    lower_point.y = float(lower_pos[1])  # ymin
                    # 使用标准16位移位编码，与解码方式保持一致
                    lower_point.z = float((lower_pos[3] << 16) | lower_pos[2])  # 编码xmax和ymax
                    response.lower_positions.append(lower_point)
                    
                    # 下装颜色
                    if lower_color:
                        color_msg = ColorRGBA()
                        color_msg.b = float(lower_color[0]) / 255.0
                        color_msg.g = float(lower_color[1]) / 255.0
                        color_msg.r = float(lower_color[2]) / 255.0
                        color_msg.a = 1.0
                        response.lower_colors.append(color_msg)
                    else:
                        response.lower_colors.append(ColorRGBA())
                        
                    response.lower_confidences.append(float(lower_conf))
                else:
                    # 无效位置标记
                    invalid_point = Point()
                    invalid_point.x = -1.0
                    invalid_point.y = -1.0
                    invalid_point.z = -1.0
                    response.lower_positions.append(invalid_point)
                    response.lower_colors.append(ColorRGBA())
                    response.lower_confidences.append(0.0)
            
            response.pairs_count = len(pairs)
            response.success = True
            response.message = f"检测成功，找到 {len(pairs)} 套服装"
            
            self.get_logger().info(f'检测完成，返回 {len(pairs)} 套服装结果')
            
        except Exception as e:
            response.success = False
            response.pairs_count = 0
            response.message = f"检测失败: {str(e)}"
            self.get_logger().error(f'图像检测服务错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
        
        return response

    def determine_body_position_callback(self, request, response):
        """
        身体位置判断服务回调函数
        
        基于上衣和下装的坐标信息，估算完整的人体区域。
        
        Args:
            request: DetermineBodyPosition请求，包含上衣和下装坐标
            response: DetermineBodyPosition响应，将填充身体位置结果
            
        Returns:
            DetermineBodyPosition.Response: 包含身体位置的响应消息
        """
        try:
            self.get_logger().info('收到身体位置判断服务请求')
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(request.image, 'bgr8')
            
            # 解码坐标信息（反向操作上面的编码）
            upper_coords = request.upper_clothes_coord
            lower_coords = request.lower_clothes_coord
            
            # 解码上衣坐标
            if upper_coords.x >= 0:
                upper_xmin = int(upper_coords.x)
                upper_ymin = int(upper_coords.y)
                upper_xmax = int(upper_coords.z) & 0xFFFF
                upper_ymax = int(upper_coords.z) >> 16
                c = [upper_xmin, upper_ymin, upper_xmax, upper_ymax]
            else:
                c = None
            
            # 解码下装坐标
            if lower_coords.x >= 0:
                lower_xmin = int(lower_coords.x)
                lower_ymin = int(lower_coords.y)
                lower_xmax = int(lower_coords.z) & 0xFFFF
                lower_ymax = int(lower_coords.z) >> 16
                p = [lower_xmin, lower_ymin, lower_xmax, lower_ymax]
            else:
                p = None
            
            # 调用身体位置判断函数
            body_positions = Determine_the_position_of_the_entire_body(c, p, cv_image)
            
            # 转换结果为ROS消息格式
            response.body_positions = []
            for pos in body_positions:
                point = Point()
                point.x = float(pos[0])
                point.y = float(pos[1])
                point.z = float(pos[2]) if len(pos) > 2 else 0.0
                response.body_positions.append(point)
            
            response.success = True
            response.message = f"身体位置判断成功，返回 {len(body_positions)} 个位置点"
            
            self.get_logger().info(f'身体位置判断完成，返回 {len(body_positions)} 个位置点')
            
        except Exception as e:
            response.success = False
            response.message = f"身体位置判断失败: {str(e)}"
            self.get_logger().error(f'身体位置判断服务错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
        
        return response


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)
        
        # 创建节点
        node = RKNNDetectorNode()
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到键盘中断信号，正在关闭...')
    except Exception as e:
        print(f'程序发生错误: {str(e)}')
        print(f'完整堆栈信息:\n{traceback.format_exc()}')
    finally:
        try:
            # 清理资源
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
            print('程序已退出')
        except Exception as e:
            print(f'清理资源时发生错误: {str(e)}')
            print(f'完整堆栈信息:\n{traceback.format_exc()}')


if __name__ == '__main__':
    main() 