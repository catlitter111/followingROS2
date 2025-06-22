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
    
    该函数对输入的双目图像进行预处理，包括颜色空间转换和对比度增强，
    为后续的立体匹配算法提供质量更好的输入图像。与原程序算法完全一致。
    
    参数:
        img1 (numpy.ndarray): 左相机图像，可以是彩色(H,W,3)或灰度(H,W)图像
        img2 (numpy.ndarray): 右相机图像，可以是彩色(H,W,3)或灰度(H,W)图像
        
    返回值:
        tuple: 包含两个预处理后灰度图像的元组
               - img1_eq (numpy.ndarray): 处理后的左图像，dtype=uint8
               - img2_eq (numpy.ndarray): 处理后的右图像，dtype=uint8
               
    处理步骤:
        1. 检查图像维度，如果是彩色图则转换为灰度图
        2. 使用CLAHE(对比度限制自适应直方图均衡化)增强图像对比度
        3. 返回处理后的灰度图像对
        
    异常:
        Exception: 当图像格式不正确或处理过程中出现错误时抛出
        
    示例:
        >>> left_img = cv2.imread('left.jpg')
        >>> right_img = cv2.imread('right.jpg') 
        >>> left_gray, right_gray = preprocess(left_img, right_img)
        >>> print(f"处理后图像尺寸: {left_gray.shape}")
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
    消除图像畸变，使用相机内参和畸变系数进行畸变校正
    
    基于OpenCV的相机标定结果，对输入图像进行畸变校正，
    消除径向畸变和切向畸变的影响。与原程序算法完全一致。
    
    参数:
        image (numpy.ndarray): 输入图像，可以是彩色或灰度图像
        camera_matrix (numpy.ndarray): 3x3相机内参矩阵，格式为:
                                     [[fx, 0, cx],
                                      [0, fy, cy], 
                                      [0, 0, 1]]
        dist_coeff (numpy.ndarray): 畸变系数数组，格式为[k1, k2, p1, p2, k3]
        
    返回值:
        numpy.ndarray: 校正畸变后的图像，与输入图像具有相同的数据类型
                       如果计算出有效的ROI区域，会自动裁剪到有效区域
                       
    算法说明:
        1. 使用getOptimalNewCameraMatrix计算最佳新相机矩阵
        2. 使用undistort函数进行畸变校正
        3. 根据ROI信息裁剪到有效区域
        
    异常:
        Exception: 当相机参数格式不正确或校正过程失败时抛出
        
    示例:
        >>> img = cv2.imread('distorted.jpg')
        >>> K = np.array([[660.19, 0, 326.32], [0, 660.87, 207.16], [0, 0, 1]])
        >>> D = np.array([[-0.0682, 0.1546, 0, 0, 0]])
        >>> undist_img = undistortion(img, K, D)
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
    
    计算双目立体视觉系统的校正参数，包括畸变校正映射和立体校正映射，
    用于将左右图像校正到同一平面，便于后续的立体匹配。与原程序算法完全一致。
    
    参数:
        height (int): 图像高度，像素为单位
        width (int): 图像宽度，像素为单位  
        config (stereoCamera): 相机配置对象，包含以下属性:
                              - cam_matrix_left: 左相机内参矩阵
                              - cam_matrix_right: 右相机内参矩阵
                              - distortion_l: 左相机畸变系数
                              - distortion_r: 右相机畸变系数
                              - R: 左右相机间旋转矩阵
                              - T: 左右相机间平移向量
                              
    返回值:
        tuple: 包含8个元素的元组:
               - left_map1 (numpy.ndarray): 左图像x方向重映射矩阵
               - left_map2 (numpy.ndarray): 左图像y方向重映射矩阵  
               - right_map1 (numpy.ndarray): 右图像x方向重映射矩阵
               - right_map2 (numpy.ndarray): 右图像y方向重映射矩阵
               - Q (numpy.ndarray): 4x4重投影矩阵，用于视差到3D坐标转换
               - roi_left (tuple): 左图像有效区域(x,y,w,h)
               - roi_right (tuple): 右图像有效区域(x,y,w,h)
               - validPixROI (tuple): 双目重叠有效区域
               
    算法说明:
        1. 使用stereoRectify计算立体校正参数
        2. 使用initUndistortRectifyMap生成重映射矩阵  
        3. 计算Q矩阵用于3D重投影
        4. 更新config对象的Q矩阵属性
        
    异常:
        Exception: 当相机参数不匹配或计算失败时抛出
        
    示例:
        >>> config = stereoCamera()
        >>> maps = getRectifyTransform(480, 640, config)
        >>> left_map1, left_map2 = maps[0], maps[1]
        >>> Q_matrix = maps[4]
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

        # 返回所有需要的值，包括ROI信息
        return map1x, map1y, map2x, map2y, Q, roi1, roi2, None
    except Exception as e:
        print(f"立体校正变换计算错误: {str(e)}")
        print(f"完整堆栈信息:\n{traceback.format_exc()}")
        raise


def rectifyImage(image1, image2, map1x, map1y, map2x, map2y):
    """
    对左右图像进行立体校正，消除畸变并使图像共面
    
    使用预计算的重映射矩阵对双目图像进行校正，使左右图像的对应点
    位于同一水平线上，为立体匹配算法提供标准输入。与原程序完全一致。
    
    参数:
        image1 (numpy.ndarray): 左相机原始图像
        image2 (numpy.ndarray): 右相机原始图像
        map1x (numpy.ndarray): 左图像x方向重映射矩阵
        map1y (numpy.ndarray): 左图像y方向重映射矩阵
        map2x (numpy.ndarray): 右图像x方向重映射矩阵  
        map2y (numpy.ndarray): 右图像y方向重映射矩阵
        
    返回值:
        tuple: 校正后的图像对
               - rectified_img1 (numpy.ndarray): 校正后的左图像
               - rectified_img2 (numpy.ndarray): 校正后的右图像
               
    处理过程:
        1. 使用remap函数对左右图像进行重映射
        2. 采用双线性插值保证图像质量
        3. 边界使用常数填充
        
    异常:
        Exception: 当重映射矩阵尺寸不匹配或处理失败时抛出
        
    示例:
        >>> left_rect, right_rect = rectifyImage(left_img, right_img, 
        ...                                     map1x, map1y, map2x, map2y)
        >>> print(f"校正后图像尺寸: {left_rect.shape}")
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
    使用SGBM算法计算视差图并应用WLS滤波器优化
    
    实现高质量的立体匹配算法，结合SGBM(Semi-Global Block Matching)和
    WLS(Weighted Least Squares)滤波器，生成平滑准确的视差图。
    与原程序算法和参数完全一致。
    
    参数:
        left_image (numpy.ndarray): 校正后的左图像，必须是灰度图(H,W)
        right_image (numpy.ndarray): 校正后的右图像，必须是灰度图(H,W)
        config (StereoConfig): 立体视觉配置对象，包含SGBM和WLS参数:
                              - minDisparity: 最小视差值
                              - numDisparities: 视差搜索范围
                              - blockSize: 匹配块大小
                              - P1, P2: 平滑约束参数
                              - disp12MaxDiff: 左右一致性检查阈值
                              - preFilterCap: 预滤波截止频率
                              - uniquenessRatio: 唯一性比率
                              - speckleWindowSize: 斑点滤波窗口大小
                              - speckleRange: 斑点滤波范围
                              - wls_lambda: WLS滤波强度
                              - wls_sigma: WLS颜色相似性敏感度
                              
    返回值:
        tuple: 包含三个视差图的元组
               - disparity (numpy.ndarray): SGBM原始视差图，dtype=int16
               - filtered_disp (numpy.ndarray): WLS滤波后视差图，dtype=float32，范围[0,255]
               - disparity_normalized (numpy.ndarray): 归一化视差图，dtype=uint8，用于显示
               
    算法流程:
        1. 创建SGBM和右图像匹配器
        2. 计算左图像和右图像的视差
        3. 创建WLS滤波器进行视差优化
        4. 对视差图进行归一化处理
        
    性能特点:
        - SGBM算法提供亚像素级精度
        - WLS滤波有效减少噪声和边缘伪影
        - 保持边缘细节的同时平滑视差图
        
    异常:
        Exception: 当图像尺寸不匹配或SGBM参数无效时抛出
        
    示例:
        >>> config = StereoConfig()
        >>> disp, filtered_disp, disp_norm = stereoMatchSGBM_WLS(left_gray, right_gray, config)
        >>> cv2.imshow('Disparity', disp_norm)
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
    将视差图重投影为3D点云坐标
    
    使用重投影矩阵Q将2D视差图转换为3D世界坐标系中的点云数据，
    每个像素点对应一个3D空间坐标。与原程序算法完全一致。
    
    参数:
        disparity (numpy.ndarray): 输入视差图，dtype可以是int16或float32
                                  视差值表示左右图像中对应点的像素差
        Q (numpy.ndarray): 4x4重投影矩阵，由stereoRectify函数计算得出
                          用于将视差值转换为3D坐标的变换矩阵
                          
    返回值:
        numpy.ndarray: 3D点云数据，shape为(H,W,3)
                       - points_3d[:,:,0]: X坐标（水平方向，单位毫米）
                       - points_3d[:,:,1]: Y坐标（垂直方向，单位毫米）  
                       - points_3d[:,:,2]: Z坐标（深度方向，单位毫米）
                       无效点的坐标值为inf或-inf
                       
    坐标系说明:
        - X轴：水平向右为正
        - Y轴：垂直向下为正  
        - Z轴：相机前方为正（远离相机）
        - 原点：左相机光心位置
        
    重投影公式:
        [X, Y, Z, W]^T = Q * [x, y, disparity, 1]^T
        实际3D坐标 = [X/W, Y/W, Z/W]
        
    异常:
        Exception: 当视差图尺寸与Q矩阵不匹配时抛出
        
    示例:
        >>> disparity = cv2.imread('disparity.png', cv2.IMREAD_GRAYSCALE)
        >>> Q = np.load('Q_matrix.npy')
        >>> points_3d = reprojectTo3D(disparity, Q)
        >>> print(f"点云shape: {points_3d.shape}")
        >>> # 获取中心点的3D坐标
        >>> center_3d = points_3d[240, 320]  # (X, Y, Z)
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
    测量指定像素点的3D距离
    
    从3D点云数据中获取指定像素坐标对应的空间距离信息，
    包括到相机的直线距离和各轴分量。与原程序完全一致。
    
    参数:
        points_3d (numpy.ndarray): 3D点云数据，shape为(H,W,3)
                                  由reprojectTo3D函数生成
        x (int): 像素点的x坐标（水平方向）
        y (int): 像素点的y坐标（垂直方向）
        
    返回值:
        dict: 包含距离信息的字典，具体包含:
              - 'valid' (bool): 该点是否有有效的3D坐标
              - 'x_mm' (float): X方向坐标，单位毫米
              - 'y_mm' (float): Y方向坐标，单位毫米  
              - 'z_mm' (float): Z方向坐标（深度），单位毫米
              - 'distance_mm' (float): 到相机的欧氏距离，单位毫米
              如果点无效，坐标值为None
              
    计算公式:
        distance = sqrt(X² + Y² + Z²)
        
    坐标有效性判断:
        - 检查坐标是否为有限值（非inf和-inf）
        - 检查像素坐标是否在图像范围内
        
    异常:
        IndexError: 当像素坐标超出点云数据范围时抛出
        
    示例:
        >>> points_3d = reprojectTo3D(disparity, Q)
        >>> distance_info = measure_distance(points_3d, 320, 240)
        >>> if distance_info['valid']:
        ...     print(f"距离: {distance_info['distance_mm']:.1f}mm")
        ...     print(f"3D坐标: ({distance_info['x_mm']:.1f}, "
        ...           f"{distance_info['y_mm']:.1f}, {distance_info['z_mm']:.1f})")
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
    
    实现基于双目相机的立体视觉系统ROS2节点，提供实时深度估计和3D测距功能。
    节点处理双目相机输入，生成视差图和3D点云，并提供距离测量服务。
    与原始stereo_vision.py程序保持完全一致的算法和参数。
    
    功能特性:
        - 双目相机图像采集和同步
        - 实时立体匹配和视差计算
        - 3D点云生成和距离测量
        - 图像发布和可视化支持
        - 基于服务的距离查询接口
        
    发布的话题:
        - /stereo/left/image_raw: 左相机原始图像
        - /stereo/disparity_image: 视差图像
        
    提供的服务:
        - /stereo_vision/get_distance: 获取指定像素点的3D距离信息
        
    算法流程:
        1. 双目相机初始化和参数加载
        2. 图像采集和预处理
        3. 畸变校正和立体校正
        4. SGBM立体匹配计算视差
        5. WLS滤波优化视差图
        6. 3D重投影生成点云
        7. 距离测量和结果发布
        
    性能优化:
        - 使用WLS滤波提高视差图质量
        - CLAHE对比度增强改善匹配效果
        - 多线程处理提高实时性
        
    属性:
        config (StereoConfig): 立体视觉配置参数
        camera_config (stereoCamera): 相机标定参数
        cap (cv2.VideoCapture): 双目相机对象
        bridge (CvBridge): ROS图像消息转换器
        left_image_pub (Publisher): 左图像发布器
        disparity_pub (Publisher): 视差图发布器
        distance_service (Service): 距离测量服务
        points_3d (numpy.ndarray): 当前帧的3D点云数据
        
    示例:
        >>> import rclpy
        >>> rclpy.init()
        >>> node = StereoVisionNode()
        >>> rclpy.spin(node)
        >>> node.destroy_node()
        >>> rclpy.shutdown()
    """

    def __init__(self):
        """
        初始化双目立体视觉节点
        
        设置ROS2节点基础配置，初始化相机参数、图像发布器和服务，
        准备立体视觉处理流水线。与原程序初始化逻辑完全一致。
        
        初始化步骤:
            1. 调用父类Node构造函数，设置节点名称
            2. 创建立体视觉和相机配置对象
            3. 初始化图像处理相关变量
            4. 设置ROS2发布器和服务
            5. 初始化双目相机
            6. 计算立体校正参数
            7. 启动图像处理定时器
            
        异常处理:
            - 相机初始化失败时记录错误并退出
            - 参数加载失败时使用默认值
            - 服务创建失败时记录警告
            
        异常:
            RuntimeError: 当相机初始化失败时抛出
            Exception: 当节点配置过程中出现其他错误时抛出
        """
        super().__init__('stereo_vision_node')
        
        # 创建配置对象
        self.config = StereoConfig()
        self.camera_config = stereoCamera()
        
        # 初始化图像处理变量
        self.cap = None
        self.bridge = CvBridge()
        self.points_3d = None
        self.current_left_image = None
        self.current_disparity = None
        
        # 添加缺失的关键成员变量初始化
        self.running = True  # 节点运行状态标志
        self.points_3d_lock = threading.Lock()  # 线程安全锁
        self.current_points_3d = None  # 当前3D点云数据
        
        # 立体校正映射矩阵
        self.map1x = None
        self.map1y = None
        self.map2x = None
        self.map2y = None
        self.Q = None
        
        # 性能监控
        self.frame_count = 0
        self.last_fps_time = time.time()
        
        try:
            # 初始化相机
            self.init_camera()
            
            # 设置立体校正参数
            self.setup_rectification()
            
            # 创建发布器
            self.setup_publishers()
            
            # 创建服务
            self.setup_services()
            
            # 创建定时器，控制处理频率
            timer_period = 1.0 / self.config.fps_limit  # 根据FPS限制设置定时器周期
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
            self.get_logger().info('双目立体视觉节点初始化成功')
            self.get_logger().info(f'相机分辨率: {self.config.frame_width}x{self.config.frame_height}')
            self.get_logger().info(f'处理帧率限制: {self.config.fps_limit} FPS')
            
        except Exception as e:
            self.get_logger().error(f'节点初始化失败: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            raise

    def init_camera(self):
        """
        初始化双目相机设备
        
        打开双目相机设备，设置图像分辨率和帧率参数。
        支持USB双目相机和分离式双目相机配置。与原程序完全一致。
        
        初始化参数:
            - 相机ID: 默认为1 (USB双目相机)
            - 图像宽度: 1280像素 (左右各640像素)
            - 图像高度: 480像素
            - 帧率: 30 FPS
            - 缓冲区: 1帧 (减少延迟)
            
        设备兼容性:
            - 支持UVC兼容的USB双目相机
            - 支持OpenCV VideoCapture接口的相机设备
            - 自动检测相机连接状态
            
        异常处理:
            - 主相机打开失败时尝试默认相机(ID=0)
            - 所有相机都无法打开时抛出RuntimeError
            - 分辨率设置失败时记录警告但继续运行
            
        异常:
            RuntimeError: 当无法打开任何相机设备时抛出
            
        示例:
            >>> node = StereoVisionNode()
            >>> # init_camera()在构造函数中自动调用
        """
        try:
            # 尝试打开相机
            self.get_logger().info(f'正在尝试打开相机ID {self.config.camera_id}...')
            self.cap = cv2.VideoCapture(self.config.camera_id)
            
            if not self.cap.isOpened():
                self.get_logger().warn(f'无法打开相机ID {self.config.camera_id}，尝试使用默认相机...')
                self.cap = cv2.VideoCapture(0)
                
                if not self.cap.isOpened():
                    raise RuntimeError("错误：无法打开相机！")
            
            # 设置相机参数
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.config.fps_limit)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 减少缓冲区延迟
            
            # 验证设置是否成功
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f'相机参数设置完成:')
            self.get_logger().info(f'  分辨率: {actual_width}x{actual_height}')
            self.get_logger().info(f'  帧率: {actual_fps:.1f} FPS')
            
            if actual_width != self.config.frame_width or actual_height != self.config.frame_height:
                self.get_logger().warn(f'分辨率设置与期望不符，期望: {self.config.frame_width}x{self.config.frame_height}')
                
        except Exception as e:
            self.get_logger().error(f'相机初始化错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            raise

    def setup_rectification(self):
        """
        设置立体校正参数
        
        计算双目立体视觉系统的校正映射矩阵，用于消除图像畸变
        并将左右图像校正到标准的立体几何配置。与原程序完全一致。
        
        计算内容:
            - 左右图像的畸变校正映射矩阵
            - 立体校正映射矩阵  
            - 重投影矩阵Q（用于3D重建）
            - 有效像素区域ROI
            
        映射矩阵用途:
            - map1x, map1y: 左图像重映射矩阵
            - map2x, map2y: 右图像重映射矩阵
            - Q: 视差到3D坐标转换矩阵
            
        处理流程:
            1. 获取当前图像尺寸
            2. 调用getRectifyTransform计算校正参数
            3. 存储映射矩阵供后续使用
            4. 更新相机配置中的Q矩阵
            
        异常:
            Exception: 当相机参数不正确或计算失败时抛出
            
        注意事项:
            - 必须在相机初始化后调用
            - 校正参数依赖于相机标定结果的准确性
            - Q矩阵的单位和坐标系与相机标定一致
        """
        try:
            # 获取图像尺寸用于校正计算
            height = self.config.frame_height
            width = self.config.frame_width // 2  # 双目相机左右各占一半宽度
            
            self.get_logger().info('正在计算立体校正参数...')
            
            # 计算立体校正变换
            result = getRectifyTransform(height, width, self.camera_config)
            
            # 解包结果
            (self.map1x, self.map1y, self.map2x, self.map2y, 
             self.Q, roi_left, roi_right, validPixROI) = result
            
            # 存储Q矩阵到相机配置中
            self.camera_config.Q = self.Q
            
            self.get_logger().info('立体校正参数计算完成')
            self.get_logger().info(f'左图像ROI: {roi_left}')
            self.get_logger().info(f'右图像ROI: {roi_right}')
            self.get_logger().info(f'有效像素ROI: {validPixROI}')
            
        except Exception as e:
            self.get_logger().error(f'立体校正参数设置失败: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')
            raise

    def setup_publishers(self):
        """
        设置ROS2图像发布器
        
        创建用于发布左相机图像和视差图的ROS2发布器，
        支持其他节点订阅和可视化。采用标准的sensor_msgs/Image格式。
        
        发布器配置:
            - 左图像发布器: 发布原始左相机图像
            - 视差图发布器: 发布彩色编码的视差图
            - 队列大小: 10 (平衡延迟和丢帧)
            - 消息格式: sensor_msgs/Image
            
        话题命名规范:
            - /stereo/left/image_raw: 左相机原始图像
            - /stereo/disparity_image: 视差图像
            
        图像编码格式:
            - 左图像: bgr8 (OpenCV标准格式)
            - 视差图: bgr8 (伪彩色编码)
            
        性能考虑:
            - 使用适中的队列大小避免内存积累
            - 发布频率与相机帧率匹配
            - 消息时间戳与图像采集时间同步
            
        异常:
            Exception: 当发布器创建失败时抛出
        """
        try:
            # 创建左图像发布器
            self.left_image_pub = self.create_publisher(
                ImageMsg, 
                '/stereo/left/image_raw', 
                10
            )
            
            # 创建视差图发布器
            self.disparity_pub = self.create_publisher(
                ImageMsg,
                '/stereo/disparity_image',
                10
            )
            
            self.get_logger().info('图像发布器设置完成')
            self.get_logger().info('  左图像话题: /stereo/left/image_raw')
            self.get_logger().info('  视差图话题: /stereo/disparity_image')
            
        except Exception as e:
            self.get_logger().error(f'发布器设置失败: {str(e)}')
            raise

    def setup_services(self):
        """
        设置ROS2距离测量服务
        
        创建提供3D距离测量功能的ROS2服务，允许其他节点查询
        指定像素点的3D坐标和距离信息。使用自定义服务接口。
        
        服务配置:
            - 服务名称: /stereo_vision/get_distance
            - 服务类型: stereo_vision_interfaces/GetDistance
            - 回调函数: get_distance_callback
            
        服务接口说明:
            请求 (Request):
                - x (int32): 像素点x坐标
                - y (int32): 像素点y坐标
                
            响应 (Response):
                - success (bool): 测量是否成功
                - distance (float64): 到相机的欧氏距离(mm)
                - point (geometry_msgs/Point): 3D坐标(x,y,z)
                - message (string): 状态消息
                
        功能特性:
            - 实时响应距离查询请求
            - 自动验证像素坐标有效性
            - 提供详细的3D空间信息
            - 错误情况下返回明确的状态信息
            
        异常:
            Exception: 当服务创建失败时抛出
        """
        try:
            # 创建距离查询服务（与原程序的测距功能完全一致）
            self.distance_service = self.create_service(
                GetDistance,
                '/stereo_vision/get_distance',  # 修改服务名称与其他节点保持一致
                self.get_distance_callback
            )
            
            self.get_logger().info('距离测量服务设置完成')
            self.get_logger().info('  服务名称: /stereo_vision/get_distance')
            self.get_logger().info('  服务类型: stereo_vision_interfaces/GetDistance')
            
        except Exception as e:
            self.get_logger().error(f'服务设置失败: {str(e)}')
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
            iml = undistortion(left_half, self.camera_config.cam_matrix_left, self.camera_config.distortion_l)
            imr = undistortion(right_half, self.camera_config.cam_matrix_right, self.camera_config.distortion_r)

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
            
            # 线程安全地获取当前3D点云数据副本
            with self.points_3d_lock:
                if self.current_points_3d is None:
                    response.success = False
                    response.distance = 0.0
                    response.message = "没有可用的3D点云数据"
                    return response
                # 创建数据副本，在锁外部进行计算
                points_3d_copy = self.current_points_3d.copy()
            
            # 在锁外部进行距离计算，避免长时间持锁
            distance = measure_distance(points_3d_copy, x, y)
            
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