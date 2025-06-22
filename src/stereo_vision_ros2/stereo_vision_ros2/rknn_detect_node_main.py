# -------------------------------- ROS2 RKNN检测节点主入口 --------------------------------
"""
ROS2 RKNN检测节点主入口文件
==========================
简化的入口点，直接使用rknn_detect_node.py中的完整实现

作者: Stereo Vision Team
日期: 2024-12-24
版本: 1.0.0
"""

import sys
import os

# 导入完整的RKNN检测节点实现
from rknn_detect_node import RKNNDetectorNode, main

if __name__ == '__main__':
    main() 