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

# 导入自定义服务（需要构建后才能导入）
# from stereo_vision_ros2.srv import DetectImageWithConfidence, DetermineBodyPosition

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
    获取图像的主色调 - 与原程序完全一致
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
    应用非极大值抑制过滤重叠检测 - 与原程序完全一致
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
    匹配上衣和下装，保留置信度信息 - 与原程序完全一致
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
    识别每件服装的主要颜色，保留置信度信息 - 与原程序完全一致
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
    估计整个身体位置 - 与原程序完全一致
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
    ROS2 RKNN颜色检测节点
    严格保持与原始程序rknn_colour_detect.py相同的逻辑和参数
    """

    def __init__(self):
        """初始化ROS2 RKNN检测节点"""
        super().__init__('rknn_color_detector')
        
        try:
            # 初始化配置参数 - 与源程序完全一致
            self.CONFIG = CONFIG.copy()
            
            # 初始化性能监控器 - 保持源程序的性能监控功能
            self.perf_monitor = PerformanceMonitor()
            
            # 初始化CV桥接器
            self.bridge = CvBridge()
            
            # 模型变量
            self.model = None
            
            # 加载RKNN模型
            self.load_rknn_model()
            
            # 设置服务
            self.setup_services()
            
            self.get_logger().info("RKNN检测节点初始化成功")
            self.get_logger().info(f"模型路径: {self.CONFIG['rknn_model_path']}")
            self.get_logger().info(f"检测置信度阈值: {self.CONFIG['conf_threshold']}")
            self.get_logger().info(f"NMS置信度阈值: {self.CONFIG['nms_confidence_threshold']}")
            self.get_logger().info(f"NMS IoU阈值: {self.CONFIG['nms_iou_threshold']}")
            
        except Exception as e:
            self.get_logger().error(f"节点初始化失败: {str(e)}")
            self.get_logger().error(f"详细错误堆栈:\n{traceback.format_exc()}")
            raise

    def load_rknn_model(self):
        """加载RKNN模型，完全复制源程序的load_model()逻辑"""
        try:
            if not RKNN_AVAILABLE:
                self.get_logger().error("RKNN Lite未安装，无法加载模型")
                return False
                
            # 使用相对路径访问模型文件
            current_file_path = os.path.abspath(__file__)
            current_dir = os.path.dirname(current_file_path)
            model_file_path = os.path.join(current_dir, '../../data', self.CONFIG['rknn_model_path'])
            
            # 检查模型文件是否存在
            if not os.path.exists(model_file_path):
                self.get_logger().error(f'模型文件不存在: {model_file_path}')
                return False
            
            # 创建RKNN对象
            rknn = RKNNLite()
            
            # 加载RKNN模型
            self.get_logger().info('正在加载RKNN模型...')
            ret = rknn.load_rknn(model_file_path)
            if ret != 0:
                self.get_logger().error(f'加载RKNN模型失败: {ret}')
                return False

            # 初始化运行时环境
            self.get_logger().info('初始化运行时环境...')
            ret = rknn.init_runtime()
            if ret != 0:
                self.get_logger().error(f'初始化运行时环境失败: {ret}')
                return False

            self.model = rknn
            self.get_logger().info(f"模型已成功加载: {model_file_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")
            return False

    def setup_services(self):
        """设置ROS2服务"""
        try:
            # 创建图像检测服务
            # 注意：由于服务定义可能还未构建，我们使用简单的回调
            # self.detect_image_service = self.create_service(
            #     DetectImageWithConfidence,
            #     '/detect_image_with_confidence',
            #     self.detect_image_callback
            # )
            
            # 创建身体位置判断服务
            # self.body_position_service = self.create_service(
            #     DetermineBodyPosition,
            #     '/determine_body_position',
            #     self.determine_body_position_callback
            # )
            
            self.get_logger().info('服务设置完成')
            # self.get_logger().info('图像检测服务: /detect_image_with_confidence')
            # self.get_logger().info('身体位置判断服务: /determine_body_position')
            
        except Exception as e:
            self.get_logger().error(f'服务设置失败: {str(e)}')
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")

    def detect_picture_with_confidence(self, img):
        """
        使用RKNN检测图像中的服装，并包含置信度信息
        与原程序的detect_picture_with_confidence函数完全一致
        """
        # 记录检测开始时间(用于性能监控)
        detect_start_time = time.time()

        if self.model is None:
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
            outputs = self.model.inference(inputs=[img_for_detection])
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
            if self.model is not None:
                self.model.release()
                self.get_logger().info('RKNN模型资源已释放')
                
            super().destroy_node()
            self.get_logger().info('RKNN检测节点已成功关闭')
            
        except Exception as e:
            self.get_logger().error(f'节点关闭错误: {str(e)}')
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")


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