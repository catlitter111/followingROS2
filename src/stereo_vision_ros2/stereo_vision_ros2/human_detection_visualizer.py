#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人体检测可视化节点
================
订阅双目相机图像，调用检测服务识别人体，
并在图像上标注边界框和距离信息

功能：
- 实时检测画面中的人体
- 计算人体中心点到相机的距离
- 在图像上绘制边界框和距离标注
- 发布标注后的图像供显示

作者: Stereo Vision Team
日期: 2024-12-24
版本: 1.0.0
"""

import cv2
import numpy as np
import traceback
from typing import List, Tuple, Optional, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# 导入自定义服务
from stereo_vision_interfaces.srv import DetectImageWithConfidence, DetermineBodyPosition, GetDistance


class HumanDetectionVisualizer(Node):
    """
    人体检测可视化节点
    
    订阅左相机图像，使用RKNN检测服务识别人体位置，
    调用距离测量服务获取深度信息，并在图像上绘制标注。
    
    Attributes:
        bridge (CvBridge): ROS图像消息转换器
        show_distance (bool): 是否显示距离信息
        show_confidence (bool): 是否显示置信度
        box_thickness (int): 边界框线条粗细
        font_scale (float): 字体大小比例
        process_every_n_frames (int): 处理频率（每n帧处理一次）
        frame_counter (int): 帧计数器
        colors (Dict[str, Tuple[int, int, int]]): 颜色配置字典
    """
    
    def __init__(self):
        """
        初始化人体检测可视化节点
        
        设置节点参数、创建服务客户端、设置订阅器和发布器。
        
        Raises:
            Exception: 节点初始化失败时抛出
        """
        super().__init__('human_detection_visualizer')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 节点参数
        self.declare_parameter('show_distance', True)
        self.declare_parameter('show_confidence', True)
        self.declare_parameter('box_thickness', 2)
        self.declare_parameter('font_scale', 0.8)
        self.declare_parameter('process_every_n_frames', 1)  # 处理频率控制
        
        # 获取参数
        self.show_distance = self.get_parameter('show_distance').value
        self.show_confidence = self.get_parameter('show_confidence').value
        self.box_thickness = self.get_parameter('box_thickness').value
        self.font_scale = self.get_parameter('font_scale').value
        self.process_every_n_frames = self.get_parameter('process_every_n_frames').value
        
        # 帧计数器
        self.frame_counter = 0
        
        # 创建服务客户端
        self.setup_service_clients()
        
        # 创建订阅器和发布器
        self.setup_pubsub()
        
        # 颜色定义（BGR格式）
        self.colors = {
            'person_box': (0, 255, 0),      # 绿色边界框
            'distance_bg': (0, 0, 0),       # 黑色背景
            'distance_text': (255, 255, 255), # 白色文字
            'confidence_text': (255, 255, 0), # 黄色置信度
        }
        
        self.get_logger().info('人体检测可视化节点初始化完成')
        
    def setup_service_clients(self) -> None:
        """
        设置ROS2服务客户端
        
        创建用于图像检测、身体位置判断和距离查询的服务客户端，
        并等待服务可用。
        
        Returns:
            None
        """
        # 图像检测服务客户端
        self.detect_client = self.create_client(
            DetectImageWithConfidence,
            '/detect_image_with_confidence'
        )
        
        # 身体位置判断服务客户端
        self.body_position_client = self.create_client(
            DetermineBodyPosition,
            '/determine_body_position'
        )
        
        # 距离查询服务客户端
        self.distance_client = self.create_client(
            GetDistance,
            '/stereo_vision/get_distance'
        )
        
        # 等待服务可用
        self.get_logger().info('等待服务可用...')
        
        services_ready = True
        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('图像检测服务不可用')
            services_ready = False
            
        if not self.body_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('身体位置判断服务不可用')
            services_ready = False
            
        if not self.distance_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('距离查询服务不可用')
            services_ready = False
            
        if services_ready:
            self.get_logger().info('所有服务已就绪')
        else:
            self.get_logger().warn('部分服务不可用，功能可能受限')
            
    def setup_pubsub(self) -> None:
        """
        设置订阅器和发布器
        
        创建图像订阅器用于接收左相机图像，
        创建发布器用于发布标注后的图像。
        
        Returns:
            None
        """
        # 订阅左相机图像
        self.image_sub = self.create_subscription(
            ImageMsg,
            '/stereo/left/image_raw',
            self.image_callback,
            10
        )
        
        # 发布标注后的图像
        self.annotated_image_pub = self.create_publisher(
            ImageMsg,
            '/stereo_vision/annotated_image',
            10
        )
        
        self.get_logger().info('订阅话题: /stereo/left/image_raw')
        self.get_logger().info('发布话题: /stereo_vision/annotated_image')
        
    def image_callback(self, msg: ImageMsg) -> None:
        """
        图像消息回调函数
        
        接收相机图像，执行人体检测和标注，发布处理结果。
        
        Args:
            msg (ImageMsg): ROS图像消息
            
        Returns:
            None
        """
        try:
            # 控制处理频率
            self.frame_counter += 1
            if self.frame_counter % self.process_every_n_frames != 0:
                return
                
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 检测人体
            body_positions = self.detect_human_bodies(msg)
            
            # 在图像上绘制标注
            annotated_image = self.draw_annotations(cv_image, body_positions)
            
            # 发布标注后的图像
            self.publish_annotated_image(annotated_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
            self.get_logger().error(f'堆栈信息:\n{traceback.format_exc()}')
            
    def detect_human_bodies(self, image_msg: ImageMsg) -> List[Dict[str, any]]:
        """
        检测图像中的人体
        
        调用RKNN检测服务识别服装，推算人体位置，
        并获取每个人体的距离信息。
        
        Args:
            image_msg (ImageMsg): 输入的ROS图像消息
            
        Returns:
            List[Dict[str, any]]: 人体检测结果列表，每个字典包含：
                - 'bbox' (List[int]): 边界框坐标 [xmin, ymin, xmax, ymax]
                - 'confidence' (float): 检测置信度 (0.0-1.0)
                - 'distance' (Optional[float]): 到相机的距离（米），可能为None
                - 'center' (Tuple[int, int]): 边界框中心点坐标 (x, y)
        """
        body_positions = []
        
        try:
            # 调用图像检测服务
            detect_request = DetectImageWithConfidence.Request()
            detect_request.image = image_msg
            
            detect_future = self.detect_client.call_async(detect_request)
            rclpy.spin_until_future_complete(self, detect_future, timeout_sec=1.0)
            
            if not detect_future.done():
                self.get_logger().warn('图像检测服务超时')
                return body_positions
                
            detect_response = detect_future.result()
            
            if not detect_response.success:
                self.get_logger().warn(f'图像检测失败: {detect_response.message}')
                return body_positions
                
            # 处理每对服装检测结果
            for i in range(detect_response.pairs_count):
                # 获取上下装位置
                upper_pos = None
                lower_pos = None
                upper_conf = 0.0
                lower_conf = 0.0
                
                if i < len(detect_response.upper_positions):
                    point = detect_response.upper_positions[i]
                    if point.x >= 0:  # 有效坐标
                        xmax = int(point.z) & 0xFFFF
                        ymax = int(point.z) >> 16
                        upper_pos = [int(point.x), int(point.y), xmax, ymax]
                        if i < len(detect_response.upper_confidences):
                            upper_conf = detect_response.upper_confidences[i]
                            
                if i < len(detect_response.lower_positions):
                    point = detect_response.lower_positions[i]
                    if point.x >= 0:  # 有效坐标
                        xmax = int(point.z) & 0xFFFF
                        ymax = int(point.z) >> 16
                        lower_pos = [int(point.x), int(point.y), xmax, ymax]
                        if i < len(detect_response.lower_confidences):
                            lower_conf = detect_response.lower_confidences[i]
                            
                # 调用身体位置判断服务
                if upper_pos or lower_pos:
                    body_bbox = self.get_body_position(
                        image_msg, upper_pos, lower_pos, upper_conf, lower_conf
                    )
                    if body_bbox:
                        body_positions.append(body_bbox)
                        
        except Exception as e:
            self.get_logger().error(f'人体检测错误: {str(e)}')
            self.get_logger().error(f'堆栈信息:\n{traceback.format_exc()}')
            
        return body_positions
        
    def get_body_position(self, 
                         image_msg: ImageMsg, 
                         upper_pos: Optional[List[int]], 
                         lower_pos: Optional[List[int]],
                         upper_conf: float,
                         lower_conf: float) -> Optional[Dict[str, any]]:
        """
        获取完整的人体位置信息
        
        根据检测到的上下装位置，推算完整的人体边界框，
        并查询中心点的距离信息。
        
        Args:
            image_msg (ImageMsg): 输入图像消息
            upper_pos (Optional[List[int]]): 上衣位置 [xmin, ymin, xmax, ymax]，可为None
            lower_pos (Optional[List[int]]): 下装位置 [xmin, ymin, xmax, ymax]，可为None
            upper_conf (float): 上衣检测置信度
            lower_conf (float): 下装检测置信度
            
        Returns:
            Optional[Dict[str, any]]: 人体位置信息字典，包含：
                - 'bbox': 边界框坐标
                - 'confidence': 置信度
                - 'distance': 距离（米）
                - 'center': 中心点坐标
            失败时返回None
        """
        try:
            # 准备身体位置判断请求
            body_request = DetermineBodyPosition.Request()
            body_request.image = image_msg
            
            # 设置上衣坐标
            if upper_pos:
                body_request.upper_clothes_coord.x = float(upper_pos[0])
                body_request.upper_clothes_coord.y = float(upper_pos[1])
                body_request.upper_clothes_coord.z = float((upper_pos[3] << 16) | upper_pos[2])
            else:
                body_request.upper_clothes_coord.x = -1.0
                
            # 设置下装坐标
            if lower_pos:
                body_request.lower_clothes_coord.x = float(lower_pos[0])
                body_request.lower_clothes_coord.y = float(lower_pos[1])
                body_request.lower_clothes_coord.z = float((lower_pos[3] << 16) | lower_pos[2])
            else:
                body_request.lower_clothes_coord.x = -1.0
                
            # 调用服务
            body_future = self.body_position_client.call_async(body_request)
            rclpy.spin_until_future_complete(self, body_future, timeout_sec=0.5)
            
            if not body_future.done():
                self.get_logger().warn('身体位置判断服务超时')
                return None
                
            body_response = body_future.result()
            
            if not body_response.success or len(body_response.body_positions) == 0:
                return None
                
            # 解析身体位置
            body_point = body_response.body_positions[0]
            xmax = int(body_point.z) & 0xFFFF
            ymax = int(body_point.z) >> 16
            bbox = [int(body_point.x), int(body_point.y), xmax, ymax]
            
            # 计算中心点
            center_x = (bbox[0] + bbox[2]) // 2
            center_y = (bbox[1] + bbox[3]) // 2
            
            # 获取距离信息
            distance = self.get_distance_at_point(center_x, center_y)
            
            # 计算平均置信度
            confidence = max(upper_conf, lower_conf)
            
            return {
                'bbox': bbox,
                'confidence': confidence,
                'distance': distance,
                'center': (center_x, center_y)
            }
            
        except Exception as e:
            self.get_logger().error(f'获取身体位置错误: {str(e)}')
            return None
            
    def get_distance_at_point(self, x: int, y: int) -> Optional[float]:
        """
        获取指定像素点的距离
        
        调用双目视觉距离测量服务，获取指定坐标点到相机的距离。
        
        Args:
            x (int): 像素点的x坐标
            y (int): 像素点的y坐标
            
        Returns:
            Optional[float]: 距离值（米），查询失败时返回None
        """
        try:
            # 准备距离查询请求
            distance_request = GetDistance.Request()
            distance_request.x = x
            distance_request.y = y
            
            # 调用服务
            distance_future = self.distance_client.call_async(distance_request)
            rclpy.spin_until_future_complete(self, distance_future, timeout_sec=0.1)
            
            if not distance_future.done():
                return None
                
            distance_response = distance_future.result()
            
            if distance_response.success:
                return distance_response.distance
            else:
                return None
                
        except Exception as e:
            self.get_logger().error(f'距离查询错误: {str(e)}')
            return None
            
    def draw_annotations(self, 
                        image: np.ndarray, 
                        body_positions: List[Dict[str, any]]) -> np.ndarray:
        """
        在图像上绘制人体检测标注
        
        绘制边界框、距离信息、置信度等标注信息。
        
        Args:
            image (np.ndarray): 输入图像，BGR格式
            body_positions (List[Dict[str, any]]): 人体检测结果列表
            
        Returns:
            np.ndarray: 标注后的图像，BGR格式
        """
        annotated = image.copy()
        
        for body_info in body_positions:
            bbox = body_info['bbox']
            confidence = body_info['confidence']
            distance = body_info['distance']
            center = body_info['center']
            
            # 绘制边界框
            cv2.rectangle(
                annotated,
                (bbox[0], bbox[1]),
                (bbox[2], bbox[3]),
                self.colors['person_box'],
                self.box_thickness
            )
            
            # 准备标签文本
            label_parts = []
            if distance is not None and self.show_distance:
                label_parts.append(f'{distance:.2f}m')
            if self.show_confidence:
                label_parts.append(f'{confidence:.0%}')
                
            if label_parts:
                label = ' | '.join(label_parts)
                
                # 计算文本尺寸
                font = cv2.FONT_HERSHEY_SIMPLEX
                (text_width, text_height), baseline = cv2.getTextSize(
                    label, font, self.font_scale, thickness=2
                )
                
                # 绘制文本背景
                text_x = bbox[0]
                text_y = bbox[1] - 10
                if text_y < text_height:
                    text_y = bbox[3] + text_height + 10
                    
                cv2.rectangle(
                    annotated,
                    (text_x, text_y - text_height - 5),
                    (text_x + text_width + 10, text_y + 5),
                    self.colors['distance_bg'],
                    -1
                )
                
                # 绘制文本
                cv2.putText(
                    annotated,
                    label,
                    (text_x + 5, text_y),
                    font,
                    self.font_scale,
                    self.colors['distance_text'],
                    thickness=2,
                    lineType=cv2.LINE_AA
                )
                
            # 绘制中心点
            cv2.circle(annotated, center, 5, self.colors['person_box'], -1)
            
        # 添加统计信息
        info_text = f'Detected: {len(body_positions)} person(s)'
        cv2.putText(
            annotated,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            self.colors['confidence_text'],
            thickness=2,
            lineType=cv2.LINE_AA
        )
        
        return annotated
        
    def publish_annotated_image(self, image: np.ndarray, header: Header) -> None:
        """
        发布标注后的图像
        
        将OpenCV图像转换为ROS消息并发布。
        
        Args:
            image (np.ndarray): 标注后的图像，BGR格式
            header (Header): 原始图像的消息头，用于时间戳同步
            
        Returns:
            None
        """
        try:
            # 转换为ROS消息
            annotated_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            annotated_msg.header = header
            
            # 发布
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布图像错误: {str(e)}')


def main(args=None):
    """
    节点主函数
    
    初始化ROS2，创建并运行人体检测可视化节点。
    
    Args:
        args: ROS2初始化参数，默认为None
        
    Returns:
        None
    """
    try:
        rclpy.init(args=args)
        node = HumanDetectionVisualizer()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到键盘中断信号，正在关闭...')
    except Exception as e:
        print(f'程序错误: {str(e)}')
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print('程序已退出')


if __name__ == '__main__':
    main()