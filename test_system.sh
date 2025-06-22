#!/bin/bash
# ROS2双目视觉系统测试脚本

echo "=========================================="
echo "ROS2双目视觉系统完整测试"
echo "=========================================="

# 检查工作空间是否已激活
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2环境未激活，请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

if [ ! -f "install/setup.bash" ]; then
    echo "❌ 工作空间未构建，请运行: colcon build"
    exit 1
fi

echo "✅ 激活工作空间..."
source install/setup.bash

echo ""
echo "📋 1. 检查节点可执行文件..."
echo "检查双目视觉节点:"
ros2 run stereo_vision_ros2 stereo_vision_node --help || echo "❌ 双目视觉节点启动失败"

echo "检查RKNN检测节点:"
ros2 run stereo_vision_ros2 rknn_detect_node --help || echo "❌ RKNN检测节点启动失败"

echo "检查可视化节点:"
ros2 run stereo_vision_ros2 human_detection_visualizer --help || echo "❌ 可视化节点启动失败"

echo ""
echo "📋 2. 检查服务接口..."
echo "检查可用服务接口:"
ros2 interface list | grep stereo_vision_interfaces

echo ""
echo "📋 3. 检查必要文件..."
echo "检查RKNN模型文件:"
if [ -f "src/stereo_vision_ros2/data/best3.rknn" ]; then
    echo "✅ RKNN模型文件存在"
else
    echo "❌ RKNN模型文件缺失"
fi

echo "检查RViz配置文件:"
if [ -f "src/stereo_vision_ros2/config/stereo_vision.rviz" ]; then
    echo "✅ RViz配置文件存在"
else
    echo "❌ RViz配置文件缺失"
fi

echo ""
echo "📋 4. 测试launch文件语法..."
echo "检查主launch文件:"
ros2 launch stereo_vision_ros2 stereo_vision_detection.launch.py --help || echo "❌ Launch文件语法错误"

echo ""
echo "=========================================="
echo "测试完成！如有❌标记请检查对应问题"
echo "=========================================="

echo ""
echo "🚀 启动完整系统命令:"
echo "ros2 launch stereo_vision_ros2 stereo_vision_detection.launch.py"
echo ""
echo "🎯 仅启动可视化的命令:"
echo "ros2 launch stereo_vision_ros2 stereo_vision_detection.launch.py enable_rviz:=false" 