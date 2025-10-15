#!/usr/bin/env python3
"""
测试脚本：演示如何在主控脚本中控制二维码扫描

使用方法：
    ros2 run qr_recognition test_qr_control --ros-args -p enable:=true
"""

import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import ControlDetection


class QRControlClient(Node):
    """二维码扫描控制客户端示例
    
    演示如何从主控脚本中调用service来控制二维码扫描
    """
    
    def __init__(self):
        super().__init__('qr_control_client')
        
        # 创建service客户端
        self.client = self.create_client(
            ControlDetection, 
            'qr_scan_control'
        )
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 qr_scan_control 服务...')
            
        self.get_logger().info('已连接到 qr_scan_control 服务')
    
    def control_scanning(self, enable: bool):
        """控制二维码扫描的开启/关闭
        
        Args:
            enable: True=开启扫描, False=关闭扫描
            
        Returns:
            bool: 操作是否成功
        """
        request = ControlDetection.Request()
        request.enable = enable
        
        # 发送异步请求
        future = self.client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Service调用成功: {response.message} (success={response.success})'
            )
            return response.success
        else:
            self.get_logger().error('Service调用失败')
            return False


def main(args=None):
    """主函数：演示service调用"""
    rclpy.init(args=args)
    
    client = QRControlClient()
    
    # 示例1：开启扫描
    client.get_logger().info('=== 开启二维码扫描 ===')
    client.control_scanning(True)
    
    # 等待一段时间（实际使用中可能在这期间执行其他任务）
    import time
    time.sleep(3.0)
    
    # 示例2：关闭扫描
    client.get_logger().info('=== 关闭二维码扫描 ===')
    client.control_scanning(False)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
