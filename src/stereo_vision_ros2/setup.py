from setuptools import setup
import os
from glob import glob

package_name = 'stereo_vision_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含launch文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含数据文件（RKNN模型等）
        (os.path.join('share', package_name, 'data'), glob('data/*')),
        # 包含配置文件（如果有）
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # ROS2可执行文件位置
        (os.path.join('lib', package_name), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stereo Vision Team',
    maintainer_email='team@stereo-vision.com',
    description='ROS2双目立体视觉功能包 - 基于OpenCV和Open3D的双目立体视觉系统',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_vision_node = stereo_vision_ros2.stereo_vision_node:main',
            'rknn_detect_node = stereo_vision_ros2.rknn_detect_node:main',
            'human_detection_visualizer = stereo_vision_ros2.human_detection_visualizer:main',
        ],
    },
)