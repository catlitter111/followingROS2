#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
服务测试脚本
用于验证所有服务是否正常工作
"""

import rclpy
from rclpy.node import Node
import sys
import os
import traceback

from stereo_vision_interfaces.srv import GetDistance, DetectImageWithConfidence, DetermineBodyPosition

def test_service_imports():
    """测试服务导入"""
    try:
        print("测试服务导入...")
        
        # 测试GetDistance服务
        print("✓ GetDistance服务导入成功")
        
        # 测试DetectImageWithConfidence服务
        print("✓ DetectImageWithConfidence服务导入成功")
        
        # 测试DetermineBodyPosition服务
        print("✓ DetermineBodyPosition服务导入成功")
        
        print("所有服务导入测试通过！")
        return True
        
    except Exception as e:
        print(f"✗ 服务导入失败: {e}")
        print(f"错误堆栈:\n{traceback.format_exc()}")
        return False

def test_service_creation():
    """测试服务请求和响应对象创建"""
    try:
        print("\n测试服务对象创建...")
        
        # 测试GetDistance
        get_distance_req = GetDistance.Request()
        get_distance_req.x = 100
        get_distance_req.y = 200
        print(f"✓ GetDistance请求对象创建成功: x={get_distance_req.x}, y={get_distance_req.y}")
        
        # 测试DetectImageWithConfidence
        detect_req = DetectImageWithConfidence.Request()
        print("✓ DetectImageWithConfidence请求对象创建成功")
        
        # 测试DetermineBodyPosition
        body_pos_req = DetermineBodyPosition.Request()
        print("✓ DetermineBodyPosition请求对象创建成功")
        
        print("所有服务对象创建测试通过！")
        return True
        
    except Exception as e:
        print(f"✗ 服务对象创建失败: {e}")
        print(f"错误堆栈:\n{traceback.format_exc()}")
        return False

def main():
    """主函数"""
    print("开始服务测试...")
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 测试服务导入
        import_success = test_service_imports()
        
        # 测试服务对象创建
        creation_success = test_service_creation()
        
        if import_success and creation_success:
            print("\n🎉 所有测试通过！服务接口正常工作。")
            return 0
        else:
            print("\n❌ 部分测试失败！")
            return 1
            
    except Exception as e:
        print(f"测试过程中发生错误: {e}")
        print(f"错误堆栈:\n{traceback.format_exc()}")
        return 1
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main()) 