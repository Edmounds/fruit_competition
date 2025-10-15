#!/usr/bin/env python3
"""
定位节点 - 从TF树获取机器人位姿并发布
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Pose2D, TransformStamped
import math
from typing import Optional


class LocalizationNode(Node):
    """
    定位节点类
    
    功能：
        - 监听TF变换（map -> base_link）
        - 将3D位姿转换为2D位姿（x, y, theta）
        - 发布到pose_data话题
    """
    
    def __init__(self):
        """
        初始化定位节点
        """
        super().__init__('localization_node')
        
        # 创建TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建发布器
        self.pose_publisher = self.create_publisher(
            Pose2D,
            'pose_data',
            10
        )
        
        # 设置定时器，10Hz频率
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('定位节点已启动')
    
    def quaternion_to_yaw(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """
        将四元数转换为yaw角（绕Z轴旋转）
        
        Args:
            qx: 四元数x分量
            qy: 四元数y分量
            qz: 四元数z分量
            qw: 四元数w分量
            
        Returns:
            float: yaw角（弧度）
        """
        # 四元数转欧拉角（yaw）
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def timer_callback(self):
        """
        定时器回调函数 - 获取并发布位姿
        """
        start_time = self.get_clock().now()
        self.get_logger().debug(f'StartTime: {start_time.nanoseconds}')
        
        try:
            # 查找从map到base_link的变换
            # 获取最新可用的变换（时间戳为0）
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                Time(),
                timeout=Duration(seconds=1)
            )
            
            # 提取位置信息
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # 提取四元数
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # 计算yaw角
            theta = self.quaternion_to_yaw(qx, qy, qz, qw)
            
            # 创建并发布Pose2D消息
            pose_msg = Pose2D()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = theta
            
            self.pose_publisher.publish(pose_msg)
            
            # 打印详细信息
            self.get_logger().info(
                f'x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, '
                f'qx: {qx:.3f}, qy: {qy:.3f}, qz: {qz:.3f}, qw: {qw:.3f}, '
                f'theta: {theta:.3f}'
            )
            
        except Exception as ex:
            self.get_logger().warning(f'无法获取TF变换: {ex}')
        
        end_time = self.get_clock().now()
        self.get_logger().debug(f'EndTime: {end_time.nanoseconds}')


def main(args=None):
    """
    主函数
    
    Args:
        args: 命令行参数
    """
    rclpy.init(args=args)
    
    node = LocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
