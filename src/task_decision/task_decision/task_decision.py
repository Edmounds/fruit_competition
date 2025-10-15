import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import ControlDetection
import time

class TaskDecision(Node):
    def __init__(self):
        super().__init__('task_decision')
        self.get_logger().info("Task Decision Node has been started.")
        
        # 创建二维码扫描控制服务客户端
        self.qr_scan_client = self.create_client(
            ControlDetection,
            'qr_scan_control'
        )
        
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
        # ========== 测试二维码扫描服务 ==========
        node.get_logger().info("="*50)
        node.get_logger().info("Testing QR code scan service...")
        node.get_logger().info("="*50)
        
        # 测试1: 启用B类型二维码扫描（水果列表）
        node.get_logger().info("\n[Test 1] Enabling B type QR code scan (fruit list)...")
        response = node.call_qr_scan_service(enable=True, qr_type='B')
        time.sleep(10)  # 等待2秒观察扫描结果
        
        # # 测试2: 切换到C类型二维码扫描（数字序列或单个水果）
        # node.get_logger().info("\n[Test 2] Switching to C type QR code scan (number sequence or single fruit)...")
        # response = node.call_qr_scan_service(enable=True, qr_type='C')
        # time.sleep(2)  # 等待2秒观察扫描结果
        
        # 测试3: 关闭扫描
        node.get_logger().info("\n[Test 3] Disabling QR code scan...")
        response = node.call_qr_scan_service(enable=False, qr_type='')
        
        node.get_logger().info("\n" + "="*50)
        node.get_logger().info("QR code scan service test completed!")
        node.get_logger().info("="*50)
        
        # 继续运行节点（可以持续接收二维码扫描结果）
        node.get_logger().info("\nNode is running... Press Ctrl+C to exit.")
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()