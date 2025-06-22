#!/usr/bin/env python3
"""
ROS2双目视觉和人体检测Launch文件
================================
启动双目摄像头、人体衣服检测节点，并显示处理后的图像

功能：
- 启动双目视觉节点进行距离测量
- 启动RKNN检测节点进行人体服装检测
- 启动可视化节点显示标注后的图像
- 可选启动RViz进行3D可视化

使用方法：
ros2 launch stereo_vision_ros2 stereo_vision_detection.launch.py

参数：
- camera_id: 双目相机设备ID (默认: 1)
- fps_limit: 处理帧率限制 (默认: 30)
- enable_rviz: 是否启动RViz (默认: false)
- enable_rqt: 是否启动rqt_image_view (默认: true)

作者: Stereo Vision Team
日期: 2024-12-24
版本: 1.0.0
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    生成ROS2 Launch描述
    
    创建并配置所有必要的节点和参数，构建完整的launch描述。
    
    Returns:
        LaunchDescription: 包含所有节点和配置的launch描述对象
    """
    
    # 声明Launch参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='双目相机设备ID'
    )
    
    fps_limit_arg = DeclareLaunchArgument(
        'fps_limit',
        default_value='30',
        description='处理帧率限制'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='是否启动RViz进行3D可视化'
    )
    
    enable_rqt_arg = DeclareLaunchArgument(
        'enable_rqt',
        default_value='true',
        description='是否启动rqt_image_view显示图像'
    )
    
    # 获取参数值
    camera_id = LaunchConfiguration('camera_id')
    fps_limit = LaunchConfiguration('fps_limit')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_rqt = LaunchConfiguration('enable_rqt')
    
    # 双目视觉节点
    stereo_vision_node = Node(
        package='stereo_vision_ros2',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
        parameters=[{
            'camera_id': camera_id,
            'fps_limit': fps_limit,
        }],
        remappings=[
            # 话题重映射（如需要）
        ]
    )
    
    # RKNN检测节点
    rknn_detect_node = Node(
        package='stereo_vision_ros2',
        executable='rknn_detect_node',
        name='rknn_detect_node',
        output='screen',
        parameters=[{
            # RKNN相关参数
        }]
    )
    
    # 人体检测可视化节点
    human_detection_visualizer = Node(
        package='stereo_vision_ros2',
        executable='human_detection_visualizer',
        name='human_detection_visualizer',
        output='screen',
        parameters=[{
            'show_distance': True,
            'show_confidence': True,
            'box_thickness': 2,
            'font_scale': 0.8,
        }]
    )
    
    # RViz可视化（可选）
    rviz_config_file = os.path.join(
        get_package_share_directory('stereo_vision_ros2'),
        'config',
        'stereo_vision.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(enable_rviz),
        output='screen'
    )
    
    # rqt_image_view显示处理后的图像
    rqt_image_view = ExecuteProcess(
        cmd=['rqt_image_view', '/stereo_vision/annotated_image'],
        condition=IfCondition(enable_rqt),
        output='screen'
    )
    
    # 返回Launch描述
    return LaunchDescription([
        # Launch参数
        camera_id_arg,
        fps_limit_arg,
        enable_rviz_arg,
        enable_rqt_arg,
        
        # 节点
        stereo_vision_node,
        rknn_detect_node,
        human_detection_visualizer,
        rviz_node,
        rqt_image_view,
    ])