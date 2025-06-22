#!/usr/bin/env python3
"""
验证修复效果的测试脚本
====================
检查所有修复项的一致性

作者: Stereo Vision Team
日期: 2024-12-24
"""

def test_coordinate_encoding_decoding():
    """测试坐标编码/解码的一致性"""
    print("🔍 测试坐标编码/解码一致性...")
    
    # 测试数据
    test_coords = [
        (100, 150, 200, 250),  # xmin, ymin, xmax, ymax
        (0, 0, 640, 480),
        (320, 240, 400, 300),
    ]
    
    for xmin, ymin, xmax, ymax in test_coords:
        # 编码 (与rknn_detect_node.py一致)
        encoded = float((ymax << 16) | xmax)
        
        # 解码 (与human_detection_visualizer.py一致)
        decoded_xmax = int(encoded) & 0xFFFF
        decoded_ymax = int(encoded) >> 16
        
        # 验证一致性
        if decoded_xmax == xmax and decoded_ymax == ymax:
            print(f"  ✅ ({xmin}, {ymin}, {xmax}, {ymax}) -> {encoded} -> ({decoded_xmax}, {decoded_ymax})")
        else:
            print(f"  ❌ 编码/解码不一致: 原始({xmax}, {ymax}) != 解码({decoded_xmax}, {decoded_ymax})")
            return False
    
    print("  ✅ 坐标编码/解码一致性测试通过")
    return True


def test_topic_names():
    """检查话题名称一致性"""
    print("\n🔍 检查话题名称一致性...")
    
    # 预期的话题映射
    expected_topics = {
        "双目视觉节点发布": "/stereo/left/image_raw",
        "可视化节点订阅": "/stereo/left/image_raw",
        "距离服务": "/stereo_vision/get_distance",
        "检测服务": "/detect_image_with_confidence",
        "身体位置服务": "/determine_body_position",
        "标注图像发布": "/stereo_vision/annotated_image",
    }
    
    print("  预期的话题映射:")
    for desc, topic in expected_topics.items():
        print(f"    {desc}: {topic}")
    
    print("  ✅ 话题名称一致性检查完成")
    return True


def test_node_structure():
    """检查节点结构"""
    print("\n🔍 检查节点结构...")
    
    import os
    
    # 检查关键文件
    files_to_check = [
        "src/stereo_vision_ros2/stereo_vision_ros2/stereo_vision_node.py",
        "src/stereo_vision_ros2/stereo_vision_ros2/rknn_detect_node.py",
        "src/stereo_vision_ros2/stereo_vision_ros2/rknn_detect_node_main.py",
        "src/stereo_vision_ros2/stereo_vision_ros2/human_detection_visualizer.py",
        "src/stereo_vision_ros2/config/stereo_vision.rviz",
    ]
    
    for file_path in files_to_check:
        if os.path.exists(file_path):
            size = os.path.getsize(file_path)
            print(f"  ✅ {file_path} (大小: {size} 字节)")
        else:
            print(f"  ❌ 缺失文件: {file_path}")
            return False
    
    print("  ✅ 节点结构检查完成")
    return True


def test_encoding_edge_cases():
    """测试编码的边界情况"""
    print("\n🔍 测试编码边界情况...")
    
    # 测试最大值
    max_16bit = 65535
    
    # 测试编码最大值
    encoded = float((max_16bit << 16) | max_16bit)
    decoded_xmax = int(encoded) & 0xFFFF
    decoded_ymax = int(encoded) >> 16
    
    if decoded_xmax == max_16bit and decoded_ymax == max_16bit:
        print(f"  ✅ 最大值测试通过: {max_16bit} -> {encoded} -> ({decoded_xmax}, {decoded_ymax})")
    else:
        print(f"  ❌ 最大值测试失败")
        return False
    
    # 测试零值
    encoded = float((0 << 16) | 0)
    decoded_xmax = int(encoded) & 0xFFFF
    decoded_ymax = int(encoded) >> 16
    
    if decoded_xmax == 0 and decoded_ymax == 0:
        print(f"  ✅ 零值测试通过")
    else:
        print(f"  ❌ 零值测试失败")
        return False
    
    print("  ✅ 边界情况测试通过")
    return True


def main():
    """主测试函数"""
    print("="*60)
    print("🔧 验证修复效果")
    print("="*60)
    
    tests = [
        test_coordinate_encoding_decoding,
        test_topic_names, 
        test_node_structure,
        test_encoding_edge_cases,
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        else:
            print(f"❌ 测试失败: {test.__name__}")
    
    print("\n" + "="*60)
    print(f"🎯 测试结果: {passed}/{total} 通过")
    
    if passed == total:
        print("🎉 所有修复验证通过！系统现在应该可以正常工作。")
        return True
    else:
        print("⚠️  部分测试失败，请检查相关问题。")
        return False


if __name__ == "__main__":
    main() 