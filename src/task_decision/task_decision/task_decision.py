import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import ControlDetection
import time

# 导入正确的消息和服务类型
from robot_control_interfaces.srv import FruitDetectionControl
from robot_control_interfaces.msg import FruitInfo


class TaskDecision(Node):
    def __init__(self):
        # ... (你原来的 __init__ 代码)
        super().__init__('task_decision')
        self.get_logger().info("Task Decision Node has been started.")
        
        # 创建二维码扫描控制服务客户端
        self.qr_scan_client = self.create_client(
            ControlDetection,
            'qr_scan_control'
        )
        
        # 【新增】创建水果检测服务客户端
        self.fruit_detection_client = self.create_client(
            FruitDetectionControl,
            'fruit_detection_control'
        )
        
        # 【新增】订阅水果检测结果话题
        self.fruit_detection_subscriber = self.create_subscription(
            FruitInfo,
            'fruit_detection',
            self.fruit_detection_callback,
            10
        )
        
        # 用于存储最新的检测结果
        self.latest_fruit_info = None
        self.get_logger().info("Subscribed to fruit_detection topic")
    
    def fruit_detection_callback(self, msg: FruitInfo):
        """实时接收水果检测结果的回调函数
        
        Args:
            msg: FruitInfo消息，包含is_ripe, fruit_type, offset_x, offset_y
        """
        self.latest_fruit_info = msg
        self.get_logger().debug(
            f'Received fruit detection: is_ripe={msg.is_ripe}, type={msg.fruit_type}, '
            f'offset=({msg.offset_x:.1f}, {msg.offset_y:.1f})'
        )
    # 调用水果检测服务的方法
    def call_fruit_detection_service(self, enable, model_mode=0):
        """调用水果检测服务
        
        Args:
            enable: bool, True=开启检测, False=关闭检测
            model_mode: int, 模型模式 (0=仅成熟度, 1=仅种类, 2=两阶段)
            
        Returns:
            response: FruitDetectionControl.Response 或 None
        """
        # 等待服务可用
        if not self.fruit_detection_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Fruit detection service not available!')
            return None
        
        # 创建请求
        request = FruitDetectionControl.Request()
        request.enable = enable
        request.model_mode = model_mode
        
        # 发送请求
        self.get_logger().info(f'Calling fruit detection service: enable={enable}, mode={model_mode}')
        future = self.fruit_detection_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Fruit detection response: success={response.success}, message={response.message}')
            return response
        else:
            self.get_logger().error('Fruit detection service call failed!')
            return None
        
    def call_qr_scan_service(self, enable, qr_type=''):
        """调用二维码扫描控制服务
        
        Args:
            enable: bool, True=开启扫描, False=关闭扫描
            qr_type: str, 'B'=水果列表, 'C'=数字序列或单个水果名
            
        Returns:
            response: ControlDetection.Response 或 None
        """
        # 等待服务可用
        if not self.qr_scan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('QR scan service not available!')
            return None
        
        # 创建请求
        request = ControlDetection.Request()
        request.enable = enable
        request.qr_type = qr_type
        
        # 发送请求
        self.get_logger().info(f'Calling QR scan service: enable={enable}, type={qr_type}')
        future = self.qr_scan_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Service response: success={response.success}, message={response.message}')
            return response
        else:
            self.get_logger().error('Service call failed!')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = TaskDecision()
    
    try:
        # # ======================================= 测试二维码扫描服务 START ==========================
        # node.get_logger().info("="*50)
        # node.get_logger().info("Testing QR code scan service...")
        # node.get_logger().info("="*50)
        
        # # 测试1: 启用B类型二维码扫描（水果列表）
        # node.get_logger().info("\n[Test 1] Enabling B type QR code scan (fruit list)...")
        # response = node.call_qr_scan_service(enable=True, qr_type='B')
        # time.sleep(10)  # 等待2秒观察扫描结果

        # # 测试2: 切换到C类型二维码扫描（数字序列或单个水果）
        # node.get_logger().info("\n[Test 2] Switching to C type QR code scan (number sequence or single fruit)...")
        # response = node.call_qr_scan_service(enable=True, qr_type='C')
        # time.sleep(2)  # 等待2秒观察扫描结果


        # # 测试3: 关闭扫描
        # node.get_logger().info("\n[Test 3] Disabling QR code scan...")
        # response = node.call_qr_scan_service(enable=False, qr_type='')
        
        # node.get_logger().info("\n" + "="*50)
        # node.get_logger().info("QR code scan service test completed!")
        # node.get_logger().info("="*50)
        
        # # 继续运行节点（可以持续接收二维码扫描结果）
        # node.get_logger().info("\nNode is running... Press Ctrl+C to exit.")


#====================================================测试扫描二维码 END ========================================

    #=========================================fruit detection start================================

        # node.get_logger().info("\n" + "="*50)
        node.get_logger().info("Starting fruit detection process...")
        # node.get_logger().info("="*50)


        # ===== 步骤1：模式0 - 成熟度检测 =====
        node.get_logger().info("\n[Step 1] Mode 0: Ripeness detection only...")
        node.get_logger().info("Waiting for fruit detector publisher...")
        while node.get_publishers_info_by_topic('fruit_detection') == []:
            time.sleep(0.1)
        node.get_logger().info("Publisher found!")
        # 清空之前的检测结果
        node.latest_fruit_info = None
        
        # 调用服务启动成熟度检测
        response = node.call_fruit_detection_service(enable=True, model_mode=0)
        if not response or not response.success:
            node.get_logger().error('Failed to enable ripeness detection')
        else:
            node.get_logger().info('成熟度检测服务已启动，等待检测结果...')
        
        # 等待检测服务处理图像并通过话题发布结果
        node.get_logger().info("等待成熟度检测中...")
        detection_timeout = 5.0  # 最长等待5秒
        start_time = time.time()
        
        # 使用spin循环等待话题数据
        while (time.time() - start_time) < detection_timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest_fruit_info is not None:
                break
        
        # 判断是否检测到成熟水果
        ripeness_detected = False
        node.get_logger().warn(f'Latest fruit info: {node.latest_fruit_info}')
        
        if node.latest_fruit_info is not None and node.latest_fruit_info.is_ripe:
            node.get_logger().info(f'✓ 检测到成熟水果！is_ripe={node.latest_fruit_info.is_ripe}')
            ripeness_detected = True
        else:
            node.get_logger().info('✗ 未检测到成熟水果')

        # ===== 步骤2：如果检测到成熟，立即切换到模式1进行种类识别 =====
        if ripeness_detected:
            node.get_logger().info("\n[Step 2] Switching to Mode 1: Fruit type classification...")
            
            # 清空之前的检测结果
            node.latest_fruit_info = None
            
            # 调用服务切换到种类识别模式
            response = node.call_fruit_detection_service(enable=True, model_mode=1)
            if not response or not response.success:
                node.get_logger().error('Failed to switch to fruit type classification')
            else:
                node.get_logger().info('种类识别服务已启动，等待分类结果...')
            
            # 等待种类识别服务处理图像
            node.get_logger().info("等待水果种类识别中...")
            start_time = time.time()
            
            # 使用spin循环等待话题数据
            classification_received = False  # 新增一个标志位
            while (time.time() - start_time) < detection_timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                
                # 核心修改：不仅要检查消息是否存在，还要检查消息内容是否是我们想要的
                if node.latest_fruit_info is not None and node.latest_fruit_info.fruit_type > 0:
                    classification_received = True
                    break  # 只有收到有效的分类结果才退出循环
                
            # 根据最新的话题数据进行决策
            # 判断条件改为检查标志位
            if classification_received:
                # 定义水果类型映射
                fruit_names = {1: "辣椒", 2: "南瓜", 3: "洋葱", 4: "番茄"}
                fruit_name = fruit_names.get(node.latest_fruit_info.fruit_type, "未知")
                
                node.get_logger().info(
                    f'✓ 成熟水果类型识别成功! 类型={fruit_name}(编号{node.latest_fruit_info.fruit_type}), '
                    f'偏移=({node.latest_fruit_info.offset_x:.1f}, {node.latest_fruit_info.offset_y:.1f})'
                )
                node.get_logger().info('→ 执行抓取程序...')
                # 此处可以添加调用机械臂执行抓取的逻辑
            else:
                node.get_logger().info('✗ 种类识别失败 (超时或结果无效)')
                node.get_logger().info('→ 可能需要调整相机角度或重新检测')
        else:
            node.get_logger().info('→ 回归中位，准备执行下一步操作')
            # 此处可以添加机械臂回归中位的逻辑


        # 步骤3: 任务结束后，务必禁用检测以节省资源
        node.get_logger().info("\n[Step 3] Disabling fruit detection...")
        node.call_fruit_detection_service(enable=False)

        # node.get_logger().info("\n" + "="*50)
        node.get_logger().info("Fruit detection process completed!")
        # node.get_logger().info("="*50)
        
#=====================================fruit detection end=================================
        
        

        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()