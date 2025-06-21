# -------------------------------- ROS2 RKNN检测节点 --------------------------------
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rknn_detect_node import *

class RKNNDetectorNode(Node):
    """
    ROS2 RKNN颜色检测节点
    严格保持与原始程序rknn_colour_detect.py相同的逻辑和参数
    """

    def __init__(self):
        """初始化ROS2 RKNN检测节点"""
        super().__init__('rknn_color_detector')
        
        try:
            # 初始化配置参数 - 与源程序完全一致
            self.CONFIG = CONFIG.copy()
            
            # 初始化性能监控器 - 保持源程序的性能监控功能
            self.perf_monitor = PerformanceMonitor()
            
            # 初始化CV桥接器
            self.bridge = CvBridge()
            
            # 模型变量
            self.model = None
            
            # 加载RKNN模型
            self.load_rknn_model()
            
            # 设置服务
            self.setup_services()
            
            self.get_logger().info("RKNN检测节点初始化成功")
            self.get_logger().info(f"模型路径: {self.CONFIG['rknn_model_path']}")
            self.get_logger().info(f"检测置信度阈值: {self.CONFIG['conf_threshold']}")
            self.get_logger().info(f"NMS置信度阈值: {self.CONFIG['nms_confidence_threshold']}")
            self.get_logger().info(f"NMS IoU阈值: {self.CONFIG['nms_iou_threshold']}")
            
        except Exception as e:
            self.get_logger().error(f"节点初始化失败: {str(e)}")
            self.get_logger().error(f"详细错误堆栈:\n{traceback.format_exc()}")
            raise

    def load_rknn_model(self):
        """加载RKNN模型，完全复制源程序的load_model()逻辑"""
        try:
            if not RKNN_AVAILABLE:
                self.get_logger().error("RKNN Lite未安装，无法加载模型")
                return False
                
            # 使用相对路径访问模型文件
            current_file_path = os.path.abspath(__file__)
            current_dir = os.path.dirname(current_file_path)
            model_file_path = os.path.join(current_dir, '../../data', self.CONFIG['rknn_model_path'])
            
            # 检查模型文件是否存在
            if not os.path.exists(model_file_path):
                self.get_logger().error(f'模型文件不存在: {model_file_path}')
                return False
            
            # 创建RKNN对象
            rknn = RKNNLite()
            
            # 加载RKNN模型
            self.get_logger().info('正在加载RKNN模型...')
            ret = rknn.load_rknn(model_file_path)
            if ret != 0:
                self.get_logger().error(f'加载RKNN模型失败: {ret}')
                return False

            # 初始化运行时环境
            self.get_logger().info('初始化运行时环境...')
            ret = rknn.init_runtime()
            if ret != 0:
                self.get_logger().error(f'初始化运行时环境失败: {ret}')
                return False

            self.model = rknn
            self.get_logger().info(f"模型已成功加载: {model_file_path}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")
            return False

    def setup_services(self):
        """设置ROS2服务"""
        try:
            # 创建图像检测服务
            # 注意：由于服务定义可能还未构建，我们使用简单的回调
            # self.detect_image_service = self.create_service(
            #     DetectImageWithConfidence,
            #     '/detect_image_with_confidence',
            #     self.detect_image_callback
            # )
            
            # 创建身体位置判断服务
            # self.body_position_service = self.create_service(
            #     DetermineBodyPosition,
            #     '/determine_body_position',
            #     self.determine_body_position_callback
            # )
            
            self.get_logger().info('服务设置完成')
            # self.get_logger().info('图像检测服务: /detect_image_with_confidence')
            # self.get_logger().info('身体位置判断服务: /determine_body_position')
            
        except Exception as e:
            self.get_logger().error(f'服务设置失败: {str(e)}')
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")

    def detect_picture_with_confidence(self, img):
        """
        使用RKNN检测图像中的服装，并包含置信度信息
        与原程序的detect_picture_with_confidence函数完全一致
        """
        # 记录检测开始时间(用于性能监控)
        detect_start_time = time.time()

        if self.model is None:
            logger.error("模型未加载，无法进行检测")
            return []

        if img is None or img.size == 0:
            logger.error("提供给detect_picture_with_confidence的图像无效")
            return []

        try:
            # 保存原始图像副本
            original_img = img.copy()
            logger.debug(f"原始图像尺寸: {img.shape}")

            # 预处理图像 - 使用letterbox而不是简单缩放，保持宽高比
            self.perf_monitor.start("preprocessing")
            img_for_detection, ratio, pad = letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE))
            logger.debug(f"预处理后图像尺寸: {img_for_detection.shape}, 缩放比例: {ratio}, 填充: {pad}")

            # 转换颜色空间从BGR到RGB（RKNN模型通常使用RGB输入）
            img_for_detection = cv.cvtColor(img_for_detection, cv.COLOR_BGR2RGB)

            # 扩展维度以匹配模型输入要求
            img_for_detection = np.expand_dims(img_for_detection, 0)
            logger.debug(f"模型输入尺寸: {img_for_detection.shape}")
            self.perf_monitor.end("preprocessing")

            # 运行RKNN推理
            logger.debug('开始RKNN推理...')
            self.perf_monitor.start("inference")
            outputs = self.model.inference(inputs=[img_for_detection])
            self.perf_monitor.end("inference")

            # 处理RKNN输出
            self.perf_monitor.start("postprocessing")
            input0_data = outputs[0]
            input1_data = outputs[1]
            input2_data = outputs[2]

            # 重塑输出以匹配YOLOv5后处理期望格式
            input0_data = input0_data.reshape([3, -1] + list(input0_data.shape[-2:]))
            input1_data = input1_data.reshape([3, -1] + list(input1_data.shape[-2:]))
            input2_data = input2_data.reshape([3, -1] + list(input2_data.shape[-2:]))

            # 转置以匹配YOLOv5后处理期望格式
            input_data = [
                np.transpose(input0_data, (2, 3, 0, 1)),
                np.transpose(input1_data, (2, 3, 0, 1)),
                np.transpose(input2_data, (2, 3, 0, 1))
            ]

            # YOLOv5后处理
            logger.debug('正在执行YOLOv5后处理...')
            boxes, classes, scores = yolov5_post_process(input_data)

            # 将坐标从letterbox空间转回原始图像空间
            if boxes is not None:
                # 获取原始图像尺寸
                orig_h, orig_w = original_img.shape[:2]
                # 获取缩放比例和填充信息
                r = ratio[0]
                dw, dh = pad

                # 调整所有边界框坐标
                for i in range(len(boxes)):
                    boxes[i][0] = (boxes[i][0] - dw) / r
                    boxes[i][1] = (boxes[i][1] - dh) / r
                    boxes[i][2] = (boxes[i][2] - dw) / r
                    boxes[i][3] = (boxes[i][3] - dh) / r

                    # 确保坐标在有效范围内
                    boxes[i][0] = max(0, min(orig_w - 1, boxes[i][0]))
                    boxes[i][1] = max(0, min(orig_h - 1, boxes[i][1]))
                    boxes[i][2] = max(0, min(orig_w, boxes[i][2]))
                    boxes[i][3] = max(0, min(orig_h, boxes[i][3]))

            self.perf_monitor.end("postprocessing")

            # 初始化结果容器
            upper_clothing = []
            lower_clothing = []
            pairs = []

            # 如果没有检测到任何物体
            if boxes is None:
                logger.debug("未检测到任何服装")
                return pairs

            logger.debug(f"检测到 {len(boxes)} 个物体，开始分类和处理...")

            # 处理检测结果
            for i, (box, score, cls) in enumerate(zip(boxes, scores, classes)):
                # 获取坐标
                xmin, ymin, xmax, ymax = box
                height, width = original_img.shape[:2]
                xmin = max(0, min(width - 1, int(xmin)))
                ymin = max(0, min(height - 1, int(ymin)))
                xmax = max(0, min(width, int(xmax)))
                ymax = max(0, min(height, int(ymax)))

                confidence = score
                class_name = CLASSES[cls]

                logger.debug(f"物体 {i + 1}: 类别={class_name}, 置信度={confidence:.4f}, 坐标=[{xmin}, {ymin}, {xmax}, {ymax}]")

                # 分类检测结果
                if class_name in CLOTHING_CATEGORIES['upper']:
                    upper_clothing.append((xmin, ymin, xmax, ymax, confidence, 'upper'))
                    logger.debug(f"将 {class_name} 分类为上衣")
                elif class_name in CLOTHING_CATEGORIES['lower']:
                    lower_clothing.append((xmin, ymin, xmax, ymax, confidence, 'lower'))
                    logger.debug(f"将 {class_name} 分类为下装")

            # 应用NMS
            logger.debug(f"NMS前上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")
            upper_clothing = apply_nms(upper_clothing)
            lower_clothing = apply_nms(lower_clothing)
            logger.debug(f"NMS后上衣数量: {len(upper_clothing)}, 下装数量: {len(lower_clothing)}")

            # 匹配服装项目
            logger.debug("开始匹配上下装...")
            match_start = time.time()
            match_clothing_items_with_confidence(upper_clothing, lower_clothing, original_img, pairs)
            logger.debug(f"匹配完成，找到 {len(pairs)} 对服装，耗时: {(time.time() - match_start) * 1000:.2f}ms")

            # 获取颜色
            logger.debug("开始识别服装颜色...")
            color_start = time.time()
            identify_clothing_colors_with_confidence(pairs, original_img)
            logger.debug(f"颜色识别完成，耗时: {(time.time() - color_start) * 1000:.2f}ms")

            # 计算总处理时间
            total_time = time.time() - detect_start_time
            logger.debug(f"检测完成，总耗时: {total_time * 1000:.2f}ms, 约 {1 / total_time:.1f} FPS")

            return pairs

        except Exception as e:
            logger.error(f"图像检测出错: {str(e)}")
            logger.error(f"错误堆栈:\n{traceback.format_exc()}")
            return []

    def destroy_node(self):
        """节点销毁时的清理工作"""
        try:
            self.get_logger().info('正在关闭RKNN检测节点...')
            
            # 释放RKNN模型资源
            if self.model is not None:
                self.model.release()
                self.get_logger().info('RKNN模型资源已释放')
                
            super().destroy_node()
            self.get_logger().info('RKNN检测节点已成功关闭')
            
        except Exception as e:
            self.get_logger().error(f'节点关闭错误: {str(e)}')
            self.get_logger().error(f"错误堆栈:\n{traceback.format_exc()}")


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)
        
        # 创建节点
        node = RKNNDetectorNode()
        
        # 运行节点
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('\n收到键盘中断信号，正在关闭...')
    except Exception as e:
        print(f'程序发生错误: {str(e)}')
        print(f'完整堆栈信息:\n{traceback.format_exc()}')
    finally:
        try:
            # 清理资源
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
            print('程序已退出')
        except Exception as e:
            print(f'清理资源时发生错误: {str(e)}')
            print(f'完整堆栈信息:\n{traceback.format_exc()}')


if __name__ == '__main__':
    main() 