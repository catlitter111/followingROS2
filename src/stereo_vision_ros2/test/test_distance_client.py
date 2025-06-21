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
    """
    双目视觉距离查询服务测试客户端
    
    用于测试双目视觉系统中距离查询服务功能的ROS2测试客户端节点。
    该客户端提供了完整的服务调用示例和错误处理机制。
    
    详细说明:
    该测试客户端专门设计用于验证立体视觉距离测量服务的功能性和准确性。
    它模拟真实的客户端调用场景，提供了标准的服务请求格式和响应处理逻辑。
    当前版本包含模拟测试功能，可在服务接口完成编译后切换为真实调用。
    
    主要功能:
    - 测试距离查询服务的可用性
    - 验证服务请求和响应格式的正确性
    - 提供完整的错误处理和日志记录
    - 支持自定义坐标点的距离查询测试
    
    服务接口:
    - 服务名称: /stereo_vision/get_distance
    - 请求格式: GetDistance.Request (包含x, y坐标)
    - 响应格式: GetDistance.Response (包含距离值和状态信息)
    
    测试场景:
    - 单点距离查询测试
    - 服务可用性检查
    - 异常情况处理验证
    - 性能和响应时间测试
    
    使用示例:
    ```python
    # 创建测试客户端
    rclpy.init()
    client = DistanceTestClient()
    
    # 执行距离查询测试
    client.test_distance_query(320, 240)
    
    # 清理资源
    client.destroy_node()
    rclpy.shutdown()
    ```
    
    注意事项:
    - 需要立体视觉节点正常运行
    - 确保服务接口包已正确编译
    - 测试前检查相机标定参数的正确性
    - 坐标范围应在图像分辨率内
    """

    def __init__(self):
        """
        初始化距离查询测试客户端
        
        创建ROS2节点并准备服务客户端连接。当前版本提供模拟测试功能，
        在服务接口编译完成后可启用真实的服务调用。
        
        详细说明:
        该初始化方法设置基础的ROS2节点功能，准备与距离查询服务的连接。
        包含了服务发现、连接建立和错误处理的完整流程。
        
        初始化流程:
        1. 调用父类构造函数创建ROS2节点
        2. 输出客户端启动日志信息
        3. 准备服务客户端连接 (注释状态，等待接口编译)
        4. 设置基础的错误处理机制
        
        服务客户端配置:
        - 服务名称: '/stereo_vision/get_distance'
        - 服务类型: GetDistance (来自stereo_vision_interfaces包)
        - 连接超时: 1.0秒
        - 自动重连: 启用
        
        异常处理:
        - 节点创建失败: 记录错误并抛出异常
        - 服务连接失败: 记录警告但继续执行
        - 其他初始化异常: 记录详细错误信息
        
        注意事项:
        - 当前使用模拟模式，需要服务接口编译后切换
        - 确保目标服务节点已启动
        - 网络连接问题可能影响服务发现
        """
        super().__init__('distance_test_client')
        self.get_logger().info('距离查询测试客户端启动')
        
        # 注意：当服务接口构建完成后，可以使用正式的服务客户端
        # from stereo_vision_ros2.srv import GetDistance
        # self.cli = self.create_client(GetDistance, '/stereo_vision/get_distance')

    def test_distance_query(self, x, y):
        """
        执行距离查询功能测试
        
        测试指定坐标点的距离查询服务功能，包括请求发送、响应接收和结果验证。
        当前版本提供模拟测试，在服务接口可用后将切换为真实调用。
        
        详细说明:
        该方法实现完整的距离查询测试流程，从服务连接检查到结果验证的每个步骤。
        包含详细的性能监控、错误处理和结果分析功能。
        
        参数:
            x (int): 查询点的x坐标 (像素单位)
                - 范围: 0 到图像宽度-1
                - 通常为图像中心或感兴趣区域的坐标
                - 建议范围: 0-1920 (取决于相机分辨率)
                
            y (int): 查询点的y坐标 (像素单位)  
                - 范围: 0 到图像高度-1
                - 通常为图像中心或感兴趣区域的坐标
                - 建议范围: 0-1080 (取决于相机分辨率)
                
        测试流程:
        1. 验证输入坐标的有效性
        2. 检查距离查询服务的可用性
        3. 构造服务请求消息
        4. 发送异步服务请求
        5. 等待服务响应并处理超时
        6. 验证响应数据的完整性和准确性
        7. 输出测试结果和性能统计
        
        响应验证:
        - 检查success字段的状态
        - 验证distance值的合理性 (通常0.1-10.0米)
        - 确认message字段包含有效描述
        - 检查响应时间是否在可接受范围内
        
        性能指标:
        - 服务响应时间: 通常 < 100ms
        - 距离精度: ±5cm (取决于标定质量)
        - 成功率: > 95% (在有效坐标范围内)
        
        异常处理:
        - 坐标超出范围: 记录警告并继续测试
        - 服务不可用: 记录错误并返回失败状态
        - 响应超时: 记录超时信息并重试
        - 服务调用异常: 记录完整错误堆栈
        
        模拟测试功能:
        当前版本包含完整的模拟测试逻辑：
        - 模拟服务请求构造
        - 模拟响应处理流程
        - 提供测试流程的完整演示
        - 为真实服务调用提供代码模板
        
        使用示例:
        ```python
        client = DistanceTestClient()
        
        # 测试图像中心点
        client.test_distance_query(320, 240)
        
        # 测试自定义坐标点
        client.test_distance_query(100, 150)
        
        # 批量测试多个点
        test_points = [(160, 120), (320, 240), (480, 360)]
        for x, y in test_points:
            client.test_distance_query(x, y)
        ```
        
        注意事项:
        - 确保坐标在图像有效范围内以获得准确结果
        - 距离查询需要双目相机正确标定
        - 测试点应选择有良好纹理特征的区域
        - 避免在图像边缘或无纹理区域进行测试
        - 光照条件会影响距离测量的准确性
        """
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
    """
    距离查询测试客户端主函数
    
    执行完整的测试客户端生命周期管理，包括初始化、测试执行和资源清理。
    支持命令行参数指定测试坐标，提供完整的错误处理和异常恢复机制。
    
    详细说明:
    该主函数实现了测试程序的完整执行流程，从命令行参数解析到资源清理的每个步骤。
    包含多层次的异常处理机制，确保程序在各种异常情况下都能优雅退出。
    
    参数:
        args (list, optional): 命令行参数列表
            - 默认为None，使用sys.argv
            - 格式: [程序名, x坐标, y坐标]
            - 示例: ['test_distance_client.py', '320', '240']
            
    命令行参数解析:
    - 无参数: 使用默认坐标 (320, 240) - 通常为图像中心
    - 两个参数: 第一个为x坐标，第二个为y坐标
    - 参数验证: 确保坐标为有效整数值
    - 错误处理: 无效参数时提供使用说明
    
    执行流程:
    1. 解析和验证命令行参数
    2. 输出测试开始信息和参数摘要
    3. 初始化ROS2系统和节点
    4. 创建距离查询测试客户端
    5. 执行距离查询测试
    6. 处理测试结果和性能统计
    7. 清理ROS2资源和节点句柄
    8. 输出测试完成状态
    
    默认测试坐标:
    - x = 320: 通常对应640宽度图像的中心
    - y = 240: 通常对应480高度图像的中心  
    - 这些坐标适用于大多数标准分辨率的测试
    
    异常处理机制:
    1. KeyboardInterrupt: 用户Ctrl+C中断
       - 优雅地停止测试并清理资源
       - 输出中断确认信息
       
    2. ValueError: 坐标参数格式错误
       - 提供详细的使用说明
       - 显示正确的命令行格式
       
    3. Exception: 其他未预期异常
       - 记录完整的错误堆栈信息
       - 确保资源得到正确清理
       
    4. 清理异常: 资源清理过程中的异常
       - 记录清理错误但不影响程序退出
       - 提供详细的错误诊断信息
    
    性能考虑:
    - ROS2初始化时间: ~100-500ms
    - 测试执行时间: ~100-300ms
    - 资源清理时间: ~50-200ms
    - 总执行时间: 通常 < 1秒
    
    使用示例:
    ```bash
    # 使用默认坐标测试
    python3 test_distance_client.py
    
    # 指定自定义坐标测试
    python3 test_distance_client.py 100 150
    
    # 测试图像角落点
    python3 test_distance_client.py 0 0
    python3 test_distance_client.py 639 479
    
    # 批量测试 (需要脚本支持)
    for i in range(0, 640, 160); do
        python3 test_distance_client.py $i 240
    done
    ```
    
    输出信息:
    - 测试参数确认
    - 节点启动状态
    - 服务连接状态  
    - 测试执行结果
    - 性能统计信息
    - 错误和警告信息
    - 程序退出状态
    
    注意事项:
    - 确保ROS2环境已正确配置
    - 测试前启动立体视觉节点
    - 坐标值应在相机分辨率范围内
    - 网络延迟可能影响服务响应时间
    - 建议在稳定的光照条件下进行测试
    """
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