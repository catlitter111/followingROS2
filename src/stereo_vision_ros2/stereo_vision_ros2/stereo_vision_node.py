# -*- coding: utf-8 -*-
"""
ROS2双目立体视觉节点
===================
基于OpenCV和Open3D的双目立体视觉系统ROS2节点实现
严格保持与原始程序相同的逻辑和参数，不得随意修改算法实现

作者: Stereo Vision Team (ROS2移植版本)
原始作者: young (简化版)
优化: 添加WLS滤波以改善视差图质量
"""

import cv2
import numpy as np
import os
import time
import sys
import threading
import math
import traceback
from functools import lru_cache
from PIL import Image, ImageDraw, ImageFont

# ROS2相关导入
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

# 导入自定义服务
from stereo_vision_interfaces.srv import GetDistance

# 尝试导入Open3D，如果失败则提供替代方案
try:
    import open3d as o3d
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("警告：Open3D未安装，3D点云可视化功能将被禁用")


# -------------------------------- 配置参数类（与原程序完全一致）--------------------------------
class StereoConfig:
    """立体视觉系统配置类 - 与原程序参数完全一致"""

    def __init__(self):
        # 相机内参和外参 - 与原程序完全一致
        self.baseline = 25.100  # 基线距离
        self.focal_length = 663  # 焦距
        self.cx = 317  # 光心x坐标
        self.cy = 210  # 光心y坐标

        # SGBM算法参数 - 与原程序完全一致
        self.minDisparity = 3
        self.numDisparities = 16  # 必须是16的倍数
        self.blockSize = 7
        self.P1 = 1176
        self.P2 = 4704
        self.disp12MaxDiff = 4
        self.preFilterCap = 31
        self.uniquenessRatio = 10
        self.speckleWindowSize = 100
        self.speckleRange = 32
        self.mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY

        # WLS滤波器参数 - 与原程序完全一致
        self.wls_lambda = 8000.0  # 滤波强度
        self.wls_sigma = 1.5  # 颜色相似性敏感度

        # 深度筛选范围 - 与原程序完全一致
        self.min_disparity_threshold = 1100
        self.max_disparity_threshold = 1570

        # 点云参数 - 与原程序完全一致
        self.min_distance_mm = 100.0
        self.max_distance_mm = 10000.0
        self.max_x_mm = 5000.0
        self.max_y_mm = 5000.0
        self.scale_correction = 1.0

        # 相机参数 - 与原程序完全一致
        self.camera_id = 1
        self.frame_width = 1280
        self.frame_height = 480
        self.fps_limit = 30

        # 测距功能 - 与原程序完全一致
        self.enable_click_measure = True
        self.measure_points = []
        self.max_measure_points = 10

        # 临时存储视差图尺寸，供鼠标回调使用
        self.last_disp_size = (0, 0)

        # 输出目录
        self.output_dir = "output"
        os.makedirs(self.output_dir, exist_ok=True)


# -------------------------------- 相机校正配置（与原程序完全一致）--------------------------------
class stereoCamera(object):
    """双目相机参数类 - 与原程序参数完全一致"""

    def __init__(self):
        # 左相机内参 - 与原程序完全一致
        self.cam_matrix_left = np.array([[660.1946,0,326.3185], [0,660.8720,207.1556], [0, 0, 1]])

        # 右相机内参 - 与原程序完全一致
        self.cam_matrix_right = np.array([[665.1635,0,319.9729], [0,665.7919,212.9630], [0, 0, 1]])

        # 左右相机畸变系数:[k1, k2, p1, p2, k3] - 与原程序完全一致
        self.distortion_l = np.array([[-0.0682,0.1546,0,0,0]])
        self.distortion_r = np.array([[-0.0749,0.1684,0,0,0]])

        # 旋转矩阵 - 与原程序完全一致
        self.R = np.array([[1.0,6.140854786327222e-04,-0.0022],
              [-6.240288417695294e-04,1,-0.0046],
              [0.0022,0.0046,1]])

        # 平移矩阵 - 转换为列向量形式 - 与原程序完全一致
        self.T = np.array([[-25.0961], [-0.0869], [-0.1893]])

        # 焦距和基线距离 - 与原程序完全一致
        self.focal_length = 663
        self.baseline = abs(self.T[0][0])

        # Q矩阵（视差到深度的映射矩阵）
        self.Q = None  # 在getRectifyTransform中计算


# -------------------------------- 图像处理函数（与原程序完全一致）--------------------------------
def preprocess(img1, img2):
    """
    图像预处理：将彩色图转为灰度图并应用直方图均衡化以增强对比度
    与原程序算法完全一致
    """
    try:
        # 转换为灰度图
        if img1.ndim == 3:
            img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        else:
            img1_gray = img1.copy()

        if img2.ndim == 3:
            img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        else:
            img2_gray = img2.copy()

        # 应用CLAHE增强对比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        img1_eq = clahe.apply(img1_gray)
        img2_eq = clahe.apply(img2_gray)

        return img1_eq, img2_eq
    except Exception as e:
        print(f"图像预处理错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def undistortion(image, camera_matrix, dist_coeff):
    """
    消除图像畸变，使用相机内参和畸变系数
    与原程序算法完全一致
    """
    try:
        h, w = image.shape[:2]
        # 计算最佳新相机矩阵
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w, h), 1, (w, h))
        # 校正畸变
        undistorted = cv2.undistort(image, camera_matrix, dist_coeff, None, new_camera_matrix)

        # 如果ROI有效，裁剪图像
        x, y, w, h = roi
        if w > 0 and h > 0:
            undistorted = undistorted[y:y + h, x:x + w]

        return undistorted
    except Exception as e:
        print(f"图像去畸变错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def getRectifyTransform(height, width, config):
    """
    获取畸变校正和立体校正的映射变换矩阵及重投影矩阵
    与原程序算法完全一致
    """
    try:
        left_K = config.cam_matrix_left
        right_K = config.cam_matrix_right
        left_dist = config.distortion_l
        right_dist = config.distortion_r
        R = config.R
        T = config.T

        # 计算立体校正参数
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            left_K, left_dist, right_K, right_dist,
            (width, height), R, T,
            flags=cv2.CALIB_ZERO_DISPARITY,  # 使校正图像的主点具有相同的像素坐标
            alpha=0.5  # 0表示裁剪所有不相交的像素，1表示保留所有像素
        )

        # 保存Q矩阵
        config.Q = Q

        # 生成映射矩阵
        map1x, map1y = cv2.initUndistortRectifyMap(
            left_K, left_dist, R1, P1, (width, height), cv2.CV_32FC1
        )
        map2x, map2y = cv2.initUndistortRectifyMap(
            right_K, right_dist, R2, P2, (width, height), cv2.CV_32FC1
        )

        return map1x, map1y, map2x, map2y, Q
    except Exception as e:
        print(f"立体校正变换计算错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def rectifyImage(image1, image2, map1x, map1y, map2x, map2y):
    """
    对图像应用畸变校正和立体校正
    与原程序算法完全一致
    """
    try:
        rectified_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_AREA)
        rectified_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_AREA)
        return rectified_img1, rectified_img2
    except Exception as e:
        print(f"图像校正错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def stereoMatchSGBM_WLS(left_image, right_image, config):
    """
    使用SGBM算法结合WLS滤波器计算高质量视差图
    与原程序算法完全一致
    """
    try:
        # 导入ximgproc模块
        from cv2 import ximgproc

        # 确保输入图像是灰度图
        if left_image.ndim != 2:
            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        if right_image.ndim != 2:
            right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # 创建左图的SGBM匹配器
        left_matcher = cv2.StereoSGBM_create(
            minDisparity=config.minDisparity,
            numDisparities=config.numDisparities,
            blockSize=config.blockSize,
            P1=config.P1,
            P2=config.P2,
            disp12MaxDiff=config.disp12MaxDiff,
            preFilterCap=config.preFilterCap,
            uniquenessRatio=config.uniquenessRatio,
            speckleWindowSize=config.speckleWindowSize,
            speckleRange=config.speckleRange,
            mode=config.mode
        )

        # 创建右图的匹配器（将minDisparity参数取反）
        right_matcher = cv2.StereoSGBM_create(
            minDisparity=-config.numDisparities + config.minDisparity,
            numDisparities=config.numDisparities,
            blockSize=config.blockSize,
            P1=config.P1,
            P2=config.P2,
            disp12MaxDiff=config.disp12MaxDiff,
            preFilterCap=config.preFilterCap,
            uniquenessRatio=config.uniquenessRatio,
            speckleWindowSize=config.speckleWindowSize,
            speckleRange=config.speckleRange,
            mode=config.mode
        )

        # 计算左右视差图
        left_disp = left_matcher.compute(left_image, right_image)
        right_disp = right_matcher.compute(right_image, left_image)

        # 转换为浮点数
        left_disp = left_disp.astype(np.float32) / 16.0
        right_disp = right_disp.astype(np.float32) / 16.0

        # 创建WLS滤波器
        wls_filter = ximgproc.createDisparityWLSFilter(left_matcher)

        # 设置WLS滤波器参数
        wls_filter.setLambda(config.wls_lambda)  # 平滑程度
        wls_filter.setSigmaColor(config.wls_sigma)  # 颜色相似性敏感度

        # 应用WLS滤波器
        filtered_disp = wls_filter.filter(left_disp, left_image, disparity_map_right=right_disp)

        # 应用形态学处理改善质量
        kernel = np.ones((3, 3), np.uint8)
        filtered_disp = cv2.morphologyEx(filtered_disp, cv2.MORPH_CLOSE, kernel)

        # 过滤小视差值和无效值
        min_valid_disp = 1.0
        filtered_disp[filtered_disp < min_valid_disp] = 0

        # 归一化到0-255以便显示
        disp_normalized = cv2.normalize(filtered_disp, None, alpha=0, beta=255,
                                        norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # 创建伪彩色视差图
        disp_color = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)

        return filtered_disp, disp_normalized, disp_color
    except Exception as e:
        print(f"SGBM立体匹配错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def reprojectTo3D(disparity, Q):
    """
    将视差图转换为3D点云
    与原程序算法完全一致
    """
    try:
        # 过滤太小的视差值，这些值会导致点投射到非常远的地方
        filtered_disp = disparity.copy()
        min_disparity = 1.0  # 设置最小视差阈值
        filtered_disp[filtered_disp < min_disparity] = 0

        # 使用OpenCV的reprojectImageTo3D进行重投影
        points_3d = cv2.reprojectImageTo3D(filtered_disp, Q)

        # 过滤深度值异常的点
        max_depth = 10000.0  # 最大深度阈值（毫米）
        mask = (points_3d[:, :, 2] > 0) & (points_3d[:, :, 2] < max_depth)

        # 对异常点设置为无效值
        points_3d[~mask] = [0, 0, 0]

        return points_3d
    except Exception as e:
        print(f"3D重投影错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def measure_distance(points_3d, x, y):
    """
    测量指定像素点到相机的距离
    与原程序算法完全一致
    """
    try:
        h, w = points_3d.shape[:2]

        # 检查坐标是否在有效范围内
        if not (0 <= x < w and 0 <= y < h):
            return None

        # 获取点的3D坐标
        point_3d = points_3d[y, x]

        # 检查点的有效性
        if np.all(np.isfinite(point_3d)) and not np.all(point_3d == 0):
            # 计算欧几里得距离
            distance = np.sqrt(np.sum(point_3d ** 2))
            return distance / 1000.0  # 转换为米

        return None
    except Exception as e:
        print(f"距离测量错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        return None


# -------------------------------- ROS2双目视觉节点 --------------------------------
class StereoVisionNode(Node):
    """
    ROS2双目立体视觉节点
    严格保持与原始程序相同的逻辑和参数
    """

    def __init__(self):
        """初始化ROS2双目视觉节点"""
        super().__init__('stereo_vision_node')
        
        try:
            # 初始化配置参数（与原程序完全一致）
            self.config = StereoConfig()
            self.stereo_config = stereoCamera()
            
            # 初始化CV桥接器
            self.bridge = CvBridge()
            
            # 初始化相机
            self.cap = None
            self.init_camera()
            
            # 预先计算校正变换矩阵
            height, width = self.config.frame_height, self.config.frame_width // 2
            self.map1x, self.map1y, self.map2x, self.map2y, self.Q = getRectifyTransform(
                height, width, self.stereo_config)
            
            # 用于存储当前的3D点云数据
            self.current_points_3d = None
            self.points_3d_lock = threading.Lock()
            
            # 创建发布者
            self.left_image_pub = self.create_publisher(
                ImageMsg, 
                '/stereo_vision/left_image', 
                10
            )
            
            self.disparity_pub = self.create_publisher(
                ImageMsg, 
                '/stereo_vision/disparity', 
                10
            )
            
            # 创建距离查询服务
            self.distance_service = self.create_service(
                GetDistance,
                '/stereo_vision/get_distance',
                self.get_distance_callback
            )
            
            # 创建定时器用于图像采集和处理
            self.timer = self.create_timer(1.0 / self.config.fps_limit, self.timer_callback)
            
            # 启动标志
            self.running = True
            
            self.get_logger().info('双目视觉节点初始化成功')
            self.get_logger().info(f'相机分辨率: {self.config.frame_width}x{self.config.frame_height}')
            self.get_logger().info(f'发布话题: /stereo_vision/left_image, /stereo_vision/disparity')
            self.get_logger().info(f'距离查询服务: /stereo_vision/get_distance')
            
        except Exception as e:
            self.get_logger().error(f'节点初始化失败: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            raise

    def init_camera(self):
        """
        初始化双目摄像头
        与原程序相机初始化逻辑完全一致
        """
        try:
            # 初始化相机
            self.cap = cv2.VideoCapture(self.config.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)

            # 如果指定相机打开失败，尝试使用默认相机
            if not self.cap.isOpened():
                self.get_logger().warn(f'无法打开相机ID {self.config.camera_id}，尝试使用默认相机...')
                self.config.camera_id = 0
                self.cap = cv2.VideoCapture(self.config.camera_id)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)

                if not self.cap.isOpened():
                    raise RuntimeError("错误：无法打开相机！")
                    
            self.get_logger().info(f'相机初始化成功，使用相机ID: {self.config.camera_id}')
            
        except Exception as e:
            self.get_logger().error(f'相机初始化错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            raise

    def timer_callback(self):
        """
        定时器回调函数，执行图像采集和处理
        严格按照原程序的处理流程
        """
        try:
            if not self.running or self.cap is None:
                return
                
            # 读取一帧（与原程序完全一致）
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('无法获取图像，尝试重新连接...')
                return

            # 调整图像大小（如果需要）
            if frame.shape[1] != self.config.frame_width or frame.shape[0] != self.config.frame_height:
                frame = cv2.resize(frame, (self.config.frame_width, self.config.frame_height))

            # 分割左右图像（与原程序完全一致）
            mid_x = frame.shape[1] // 2
            left_half = frame[:, :mid_x]
            right_half = frame[:, mid_x:]

            # 消除畸变（与原程序完全一致）
            iml = undistortion(left_half, self.stereo_config.cam_matrix_left, self.stereo_config.distortion_l)
            imr = undistortion(right_half, self.stereo_config.cam_matrix_right, self.stereo_config.distortion_r)

            # 预处理图像（与原程序完全一致）
            iml_, imr_ = preprocess(iml, imr)

            # 图像校正（与原程序完全一致）
            iml_rectified, imr_rectified = rectifyImage(iml_, imr_, self.map1x, self.map1y, self.map2x, self.map2y)

            # 计算视差图（使用WLS滤波器改善质量，与原程序完全一致）
            disparity, disp_normalized, disp_color = stereoMatchSGBM_WLS(
                iml_rectified,
                imr_rectified,
                self.config
            )

            # 计算3D点云（与原程序完全一致）
            points_3d = reprojectTo3D(disparity, self.Q)

            # 应用比例校正（与原程序完全一致）
            if self.config.scale_correction != 1.0:
                points_3d *= self.config.scale_correction

            # 线程安全地更新3D点云数据
            with self.points_3d_lock:
                self.current_points_3d = points_3d.copy()

            # 发布左图像
            self.publish_left_image(left_half)
            
            # 发布视差图
            self.publish_disparity_image(disp_color)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')

    def publish_left_image(self, left_image):
        """发布左摄像头图像"""
        try:
            # 创建ROS图像消息
            img_msg = self.bridge.cv2_to_imgmsg(left_image, encoding='bgr8')
            img_msg.header = Header()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'stereo_left'
            
            # 发布图像
            self.left_image_pub.publish(img_msg)
            
        except Exception as e:
            self.get_logger().error(f'左图像发布错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')

    def publish_disparity_image(self, disparity_image):
        """发布视差图像"""
        try:
            # 创建ROS图像消息
            disp_msg = self.bridge.cv2_to_imgmsg(disparity_image, encoding='bgr8')
            disp_msg.header = Header()
            disp_msg.header.stamp = self.get_clock().now().to_msg()
            disp_msg.header.frame_id = 'stereo_disparity'
            
            # 发布视差图
            self.disparity_pub.publish(disp_msg)
            
        except Exception as e:
            self.get_logger().error(f'视差图发布错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')

    def get_distance_callback(self, request, response):
        """
        距离查询服务回调函数
        与原程序的measure_distance函数逻辑完全一致
        """
        try:
            # 获取请求的坐标点
            x = request.x
            y = request.y
            
            self.get_logger().info(f'收到距离查询请求: 坐标({x}, {y})')
            
            # 线程安全地获取当前3D点云数据
            with self.points_3d_lock:
                if self.current_points_3d is None:
                    response.success = False
                    response.distance = 0.0
                    response.message = "没有可用的3D点云数据"
                    return response
                
                # 使用与原程序完全一致的距离测量算法
                distance = measure_distance(self.current_points_3d, x, y)
                
                if distance is not None:
                    response.success = True
                    response.distance = distance
                    response.message = f"测量成功，距离: {distance:.3f}米"
                    self.get_logger().info(f'距离测量成功: {distance:.3f}米')
                else:
                    response.success = False
                    response.distance = 0.0
                    response.message = "无效的坐标点或深度数据"
                    self.get_logger().warn(f'坐标({x}, {y})的距离测量失败')
                
        except Exception as e:
            response.success = False
            response.distance = 0.0
            response.message = f"距离查询服务错误: {str(e)}"
            self.get_logger().error(f'距离查询服务错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
        
        return response

    def destroy_node(self):
        """节点销毁时的清理工作"""
        try:
            self.get_logger().info('正在关闭双目视觉节点...')
            self.running = False
            
            # 释放相机资源
            if self.cap is not None:
                self.cap.release()
                self.get_logger().info('相机资源已释放')
                
            super().destroy_node()
            self.get_logger().info('双目视觉节点已成功关闭')
            
        except Exception as e:
            self.get_logger().error(f'节点关闭错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)
        
        # 创建节点
        node = StereoVisionNode()
        
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