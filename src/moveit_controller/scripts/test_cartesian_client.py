#!/usr/bin/env python3
"""
笛卡尔控制器测试客户端

功能：发送笛卡尔空间移动请求到 cartesian_controller

使用示例：
    # 向下移动 10cm
    python3 test_cartesian_client.py 0 0 -0.1
    
    # 向右移动 5cm，向前移动 3cm
    python3 test_cartesian_client.py 0.05 0.03 0
"""

import sys
import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import MoveToPosition


class CartesianTestClient(Node):
    """笛卡尔移动测试客户端"""
    
    def __init__(self):
        super().__init__('cartesian_test_client')
        
        # 创建服务客户端
        self.client = self.create_client(MoveToPosition, 'move_to_position')
        
        # 等待服务可用
        self.get_logger().info('等待 /move_to_position 服务...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务未就绪，继续等待...')
        
        self.get_logger().info('服务已连接！')
    
    def send_request(self, x: float, y: float, z: float):
        """
        发送移动请求
        
        Args:
            x: X轴方向移动距离（米）
            y: Y轴方向移动距离（米）
            z: Z轴方向移动距离（米）
        """
        request = MoveToPosition.Request()
        request.x = x
        request.y = y
        request.z = z
        
        self.get_logger().info(f'发送请求: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # 异步调用服务
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        # 获取响应
        response = future.result()
        
        if response.success:
            self.get_logger().info(f'✅ 成功: {response.message}')
        else:
            self.get_logger().error(f'❌ 失败: {response.message}')
        
        return response.success


def main(args=None):
    """主函数"""
    
    # 解析命令行参数
    if len(sys.argv) != 4:
        print('用法: python3 test_cartesian_client.py <x> <y> <z>')
        print('示例: python3 test_cartesian_client.py 0 0 -0.1  # 向下移动10cm')
        sys.exit(1)
    
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    except ValueError:
        print('错误: 参数必须是数字')
        sys.exit(1)
    
    # 初始化 ROS 2
    rclpy.init(args=args)
    
    # 创建客户端并发送请求
    client = CartesianTestClient()
    success = client.send_request(x, y, z)
    
    # 清理
    client.destroy_node()
    rclpy.shutdown()
    
    # 返回状态码
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
