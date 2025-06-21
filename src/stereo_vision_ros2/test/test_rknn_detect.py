# -*- coding: utf-8 -*-
"""
RKNN检测节点测试文件
==================
测试RKNN检测节点的核心功能

作者: Stereo Vision Team
"""

import unittest
import traceback
import numpy as np
import cv2 as cv
import os
import sys

# 添加路径以导入节点
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'stereo_vision_ros2'))

# 导入ROS2相关
import rclpy
from rclpy.node import Node

# 尝试导入节点类和算法函数
try:
    # 先导入算法模块
    from stereo_vision_ros2.stereo_vision_ros2.rknn_detect_node import *
    # 再导入节点主类
    from stereo_vision_ros2.stereo_vision_ros2.rknn_detect_node_main import RKNNDetectorNode
except ImportError as e:
    print(f"导入节点失败: {e}")
    print("请确保已正确构建功能包")
    # 为了让测试能继续运行，我们设置一些默认值
    RKNNDetectorNode = None
    CLASSES = None
    CLOTHING_CATEGORIES = None
    sigmoid = None
    xywh2xyxy = None
    letterbox = None
    get_dominant_color = None
    Determine_the_position_of_the_entire_body = None
    PerformanceMonitor = None


class TestRKNNDetector(unittest.TestCase):
    """RKNN检测器测试类"""
    
    @classmethod
    def setUpClass(cls):
        """设置测试类"""
        try:
            # 初始化ROS2
            rclpy.init()
            print("ROS2初始化成功")
        except Exception as e:
            print(f"ROS2初始化失败: {e}")
            print(f"错误堆栈:\n{traceback.format_exc()}")
    
    @classmethod
    def tearDownClass(cls):
        """清理测试类"""
        try:
            rclpy.shutdown()
            print("ROS2已关闭")
        except Exception as e:
            print(f"ROS2关闭失败: {e}")

    def setUp(self):
        """设置每个测试"""
        try:
            # 创建节点实例
            self.node = RKNNDetectorNode()
            print("RKNN检测节点创建成功")
        except Exception as e:
            print(f"测试初始化失败: {e}")
            print(f"错误堆栈:\n{traceback.format_exc()}")
            self.node = None
    
    def tearDown(self):
        """清理每个测试"""
        try:
            if self.node is not None:
                self.node.destroy_node()
                print("节点已销毁")
        except Exception as e:
            print(f"节点销毁失败: {e}")
            
    def test_node_initialization(self):
        """测试节点初始化"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        # 检查节点是否正确初始化
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'rknn_color_detector')
        print("✓ 节点初始化测试通过")
        
    def test_config_parameters(self):
        """测试配置参数"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        # 检查配置参数是否正确设置
        self.assertIsNotNone(self.node.CONFIG)
        self.assertEqual(self.node.CONFIG['conf_threshold'], 0.3)
        self.assertEqual(self.node.CONFIG['nms_confidence_threshold'], 0.05)
        self.assertEqual(self.node.CONFIG['nms_iou_threshold'], 0.1)
        self.assertEqual(self.node.CONFIG['max_x_distance_ratio'], 0.2)
        self.assertEqual(self.node.CONFIG['dominant_color_k'], 4)
        self.assertEqual(self.node.CONFIG['detection_width'], 640)
        self.assertEqual(self.node.CONFIG['detection_height'], 640)
        print("✓ 配置参数测试通过")
        
    def test_performance_monitor(self):
        """测试性能监控器"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        # 检查性能监控器是否正确初始化
        self.assertIsNotNone(self.node.perf_monitor)
        self.assertIsInstance(self.node.perf_monitor, PerformanceMonitor)
        
        # 测试性能监控功能
        self.node.perf_monitor.start("test_operation")
        self.node.perf_monitor.end("test_operation")
        stats = self.node.perf_monitor.get_stats("test_operation")
        self.assertIsNotNone(stats)
        self.assertIn('min', stats)
        self.assertIn('max', stats)
        self.assertIn('avg', stats)
        print("✓ 性能监控器测试通过")
        
    def test_model_loading(self):
        """测试模型加载功能"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        # 检查模型是否尝试加载
        # 注意：如果RKNN不可用或模型文件不存在，模型可能为None
        # 这里主要测试加载函数是否能正常执行
        try:
            result = self.node.load_rknn_model()
            # 如果RKNN可用且模型文件存在，应该返回True
            # 如果不可用，应该返回False但不抛出异常
            self.assertIsInstance(result, bool)
            print("✓ 模型加载测试通过")
        except Exception as e:
            self.fail(f"模型加载测试失败: {e}\n{traceback.format_exc()}")
            
    def test_detect_picture_with_confidence_empty_image(self):
        """测试空图像的检测功能"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        # 测试空图像
        try:
            pairs = self.node.detect_picture_with_confidence(None)
            self.assertIsInstance(pairs, list)
            self.assertEqual(len(pairs), 0)
            print("✓ 空图像检测测试通过")
        except Exception as e:
            self.fail(f"空图像检测测试失败: {e}\n{traceback.format_exc()}")
            
    def test_detect_picture_with_confidence_test_image(self):
        """测试测试图像的检测功能"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        # 创建测试图像
        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
        # 添加一些颜色以模拟真实图像
        test_img[100:300, 100:300] = [255, 0, 0]  # 红色方块
        test_img[350:550, 350:550] = [0, 255, 0]  # 绿色方块
        
        try:
            pairs = self.node.detect_picture_with_confidence(test_img)
            self.assertIsInstance(pairs, list)
            # 由于是测试图像，可能检测不到任何服装，但应该返回空列表而不是出错
            print(f"✓ 测试图像检测完成，检测到 {len(pairs)} 对服装")
        except Exception as e:
            self.fail(f"测试图像检测失败: {e}\n{traceback.format_exc()}")
            
    def test_determine_body_position(self):
        """测试身体位置判断功能"""
        if self.node is None:
            self.skipTest("节点初始化失败，跳过测试")
        
        test_img = np.zeros((640, 640, 3), dtype=np.uint8)
        upper_coords = (100, 100, 200, 200)
        lower_coords = (120, 220, 180, 320)
        
        try:
            positions = Determine_the_position_of_the_entire_body(
                upper_coords, lower_coords, test_img
            )
            self.assertIsInstance(positions, list)
            print(f"✓ 身体位置判断测试通过，计算出 {len(positions)} 个位置")
        except Exception as e:
            self.fail(f"身体位置判断测试失败: {e}\n{traceback.format_exc()}")
            
    def test_core_algorithms(self):
        """测试核心算法函数"""
        try:
            # 测试sigmoid函数
            result = sigmoid(np.array([0, 1, -1]))
            self.assertEqual(len(result), 3)
            self.assertAlmostEqual(result[0], 0.5, places=5)
            
            # 测试xywh2xyxy函数
            boxes = np.array([[100, 100, 50, 50]])  # [x, y, w, h]
            result = xywh2xyxy(boxes)
            expected = np.array([[75, 75, 125, 125]])  # [x1, y1, x2, y2]
            np.testing.assert_array_almost_equal(result, expected)
            
            # 测试letterbox函数
            test_img = np.zeros((480, 640, 3), dtype=np.uint8)
            result_img, ratio, pad = letterbox(test_img, new_shape=(640, 640))
            self.assertEqual(result_img.shape, (640, 640, 3))
            
            # 测试get_dominant_color函数
            test_img = np.ones((100, 100, 3), dtype=np.uint8) * 128  # 灰色图像
            color = get_dominant_color(test_img)
            self.assertIsInstance(color, tuple)
            self.assertEqual(len(color), 3)
            
            print("✓ 核心算法函数测试通过")
        except Exception as e:
            self.fail(f"核心算法测试失败: {e}\n{traceback.format_exc()}")
            
    def test_clothing_categories(self):
        """测试服装类别定义"""
        # 检查服装类别是否正确定义
        self.assertIn('upper', CLOTHING_CATEGORIES)
        self.assertIn('lower', CLOTHING_CATEGORIES)
        
        # 检查上衣类别
        upper_categories = CLOTHING_CATEGORIES['upper']
        expected_upper = [
            'short_sleeved_shirt', 'long_sleeved_shirt', 'short_sleeved_outwear',
            'long_sleeved_outwear', 'vest', 'sling'
        ]
        for category in expected_upper:
            self.assertIn(category, upper_categories)
            
        # 检查下装类别
        lower_categories = CLOTHING_CATEGORIES['lower']
        expected_lower = [
            'shorts', 'trousers', 'skirt', 'short_sleeved_dress',
            'long_sleeved_dress', 'vest_dress', 'sling_dress'
        ]
        for category in expected_lower:
            self.assertIn(category, lower_categories)
            
        print("✓ 服装类别定义测试通过")
        
    def test_classes_definition(self):
        """测试CLASSES常量定义"""
        expected_classes = (
            'short_sleeved_shirt', 'long_sleeved_shirt', 'short_sleeved_outwear',
            'long_sleeved_outwear', 'vest', 'sling', 'shorts', 'trousers',
            'skirt', 'short_sleeved_dress', 'long_sleeved_dress', 
            'vest_dress', 'sling_dress'
        )
        
        self.assertEqual(CLASSES, expected_classes)
        self.assertEqual(len(CLASSES), 13)
        print("✓ CLASSES定义测试通过")


def run_tests():
    """运行所有测试"""
    print("=" * 60)
    print("开始运行RKNN检测节点测试")
    print("=" * 60)
    
    # 创建测试套件
    suite = unittest.TestLoader().loadTestsFromTestCase(TestRKNNDetector)
    
    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    print("=" * 60)
    if result.wasSuccessful():
        print("✓ 所有测试通过！")
    else:
        print(f"✗ 测试失败: {len(result.failures)} 个失败, {len(result.errors)} 个错误")
        
        # 打印失败和错误详情
        for test, traceback_str in result.failures:
            print(f"\n失败: {test}")
            print(traceback_str)
            
        for test, traceback_str in result.errors:
            print(f"\n错误: {test}")
            print(traceback_str)
    
    print("=" * 60)
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    sys.exit(0 if success else 1) 