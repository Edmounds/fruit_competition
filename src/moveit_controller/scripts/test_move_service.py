#!/usr/bin/env python3
"""
MoveIt位置移动服务客户端示例

演示如何调用 /move_to_position 服务来控制机械臂移动到指定的三维坐标
"""

import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import MoveToPosition


class MoveItServiceClient(Node):
    """MoveIt服务客户端节点"""
    
    def __init__(self):
        super().__init__('moveit_service_client')
        self.client = self.create_client(MoveToPosition, '/move_to_position')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务 /move_to_position 可用...')
        
        self.get_logger().info('服务已连接！')
    
    def send_move_request(self, x: float, y: float, z: float):
        """
        发送移动请求
        
        Args:
            x: 目标X坐标 (米)
            y: 目标Y坐标 (米)
            z: 目标Z坐标 (米)
        """
        request = MoveToPosition.Request()
        request.x = x
        request.y = y
        request.z = z
        
        self.get_logger().info(f'发送移动请求到位置: [{x:.3f}, {y:.3f}, {z:.3f}]')
        
        # 异步调用服务
        future = self.client.call_async(request)
        return future


def main():
    rclpy.init()
    
    client = MoveItServiceClient()
    
    # 示例1: 移动到位置 [0.3, 0.0, 0.4]
    future = client.send_move_request(0.1, 0.0, 0.27)

    # 等待结果
    rclpy.spin_until_future_complete(client, future)
    
    if future.result() is not None:
        response = future.result()
        if response.success:
            client.get_logger().info(f'✓ 成功: {response.message}')
        else:
            client.get_logger().error(f'✗ 失败: {response.message}')
    else:
        client.get_logger().error('服务调用失败')
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
