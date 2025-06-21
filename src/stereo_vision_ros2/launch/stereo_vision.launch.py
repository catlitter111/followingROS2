# -*- coding: utf-8 -*-
"""
ROS2双目立体视觉启动文件
======================
用于启动双目视觉节点和相关功能
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """生成launch描述"""
    
    # 声明启动参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='双目摄像头设备ID'
    )
    
    fps_limit_arg = DeclareLaunchArgument(
        'fps_limit', 
        default_value='30',
        description='图像采集帧率限制'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false', 
        description='是否启动RViz可视化'
    )
    
    # 双目视觉节点
    stereo_vision_node = Node(
        package='stereo_vision_ros2',
        executable='stereo_vision_node',
        name='stereo_vision_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'fps_limit': LaunchConfiguration('fps_limit'),
        }],
        remappings=[
            # 可以在这里重映射话题名称
        ]
    )
    
    # RViz可视化节点（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', '$(find stereo_vision_ros2)/config/stereo_vision.rviz']
    )
    
    # 启动信息
    start_info = LogInfo(
        msg=[
            '启动ROS2双目立体视觉系统\n',
            '相机ID: ', LaunchConfiguration('camera_id'), '\n',
            '帧率限制: ', LaunchConfiguration('fps_limit'), ' FPS\n',
            '发布话题:\n',
            '  - /stereo_vision/left_image (左摄像头图像)\n',
            '  - /stereo_vision/disparity (视差图)\n',
            '服务:\n',
            '  - /stereo_vision/get_distance (距离查询服务)\n'
        ]
    )
    
    return LaunchDescription([
        # 启动参数
        camera_id_arg,
        fps_limit_arg, 
        enable_rviz_arg,
        
        # 启动信息
        start_info,
        
        # 节点
        stereo_vision_node,
        rviz_node,
    ]) 