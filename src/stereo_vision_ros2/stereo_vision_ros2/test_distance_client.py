# -*- coding: utf-8 -*-
"""
双目视觉距离查询服务测试客户端
============================
用于测试距离查询服务的功能

使用方法:
    python3 test_distance_client.py [x] [y]
    
示例:
    python3 test_distance_client.py 320 240
"""

import sys
import traceback
import rclpy
from rclpy.node import Node

# 临时的服务请求/响应类（因为服务可能还未构建）
class GetDistanceRequest:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

class GetDistanceResponse:
    def __init__(self):
        self.success = False
        self.distance = 0.0
        self.message = ""


class DistanceTestClient(Node):
    """距离查询服务测试客户端"""

    def __init__(self):
        super().__init__('distance_test_client')
        self.get_logger().info('距离查询测试客户端启动')
        
        # 注意：当服务接口构建完成后，可以使用正式的服务客户端
        # from stereo_vision_ros2.srv import GetDistance
        # self.cli = self.create_client(GetDistance, '/stereo_vision/get_distance')

    def test_distance_query(self, x, y):
        """测试距离查询功能"""
        try:
            self.get_logger().info(f'测试查询坐标点 ({x}, {y}) 的距离')
            
            # 这里是模拟的测试代码
            # 实际使用时需要等待服务构建完成
            
            # while not self.cli.wait_for_service(timeout_sec=1.0):
            #     self.get_logger().info('距离查询服务不可用，等待中...')
            
            # request = GetDistance.Request()
            # request.x = x
            # request.y = y
            
            # future = self.cli.call_async(request)
            # rclpy.spin_until_future_complete(self, future)
            
            # if future.result() is not None:
            #     response = future.result()
            #     if response.success:
            #         self.get_logger().info(f'距离查询成功: {response.distance:.3f}米')
            #         self.get_logger().info(f'服务消息: {response.message}')
            #     else:
            #         self.get_logger().warn(f'距离查询失败: {response.message}')
            # else:
            #     self.get_logger().error('服务调用失败')
            
            # 临时模拟响应
            self.get_logger().info('注意：这是模拟测试响应')
            self.get_logger().info('请先构建功能包以启用真实的服务接口')
            self.get_logger().info(f'模拟查询坐标 ({x}, {y})')
            
        except Exception as e:
            self.get_logger().error(f'距离查询测试错误: {str(e)}')
            self.get_logger().error(f'完整堆栈信息:\n{traceback.format_exc()}')


def main(args=None):
    """主函数"""
    try:
        # 解析命令行参数
        if len(sys.argv) >= 3:
            x = int(sys.argv[1])
            y = int(sys.argv[2])
        else:
            # 使用默认测试坐标（图像中心）
            x = 320
            y = 240
        
        print(f'测试距离查询服务 - 坐标: ({x}, {y})')
        
        # 初始化ROS2
        rclpy.init(args=args)
        
        # 创建测试客户端
        client = DistanceTestClient()
        
        # 执行测试
        client.test_distance_query(x, y)
        
        # 简短的spin让日志输出
        rclpy.spin_once(client, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        print('\n收到键盘中断信号')
    except ValueError:
        print('错误：请提供有效的整数坐标')
        print('使用方法: python3 test_distance_client.py [x] [y]')
    except Exception as e:
        print(f'测试客户端错误: {str(e)}')
        print(f'完整堆栈信息:\n{traceback.format_exc()}')
    finally:
        try:
            if 'client' in locals():
                client.destroy_node()
            rclpy.shutdown()
            print('测试客户端已退出')
        except Exception as e:
            print(f'清理资源时发生错误: {str(e)}')


if __name__ == '__main__':
    main() 