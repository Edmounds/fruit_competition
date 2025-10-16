"""
电机控制模块

提供 Motor 类用于控制移动底盘电机，通过发布 /cmd_vel 话题
来实现优雅的速度控制接口。
"""

from geometry_msgs.msg import Twist
from rclpy.node import Node


class Motor:
    """
    移动底盘电机控制器
    
    该类提供对移动底盘电机的简单控制接口，通过发布 /cmd_vel 话题
    来控制机器人的线速度和角速度。所有复杂的 Twist 消息构造被封装
    在内部，提供简洁的 API。
    
    Attributes:
        _node (Node): ROS 2 节点引用
        _publisher (Publisher): /cmd_vel 话题发布者
    
    Example:
        >>> # 初始化电机控制器
        >>> motor = Motor(node)
        >>> # 控制机器人前进
        >>> motor.move_forward(speed=0.5)
        >>> # 控制机器人旋转
        >>> motor.turn(angular_speed=1.57)
        >>> # 停止
        >>> motor.stop()
    """
    
    def __init__(self, node: Node):
        """
        初始化电机控制器
        
        Args:
            node (Node): ROS 2 节点实例，用于创建发布者
        """
        self._node = node
        
        # 创建 /cmd_vel 发布者
        self._publisher = node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        node.get_logger().info("✓ 电机控制器已初始化")
    
    def move(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        """
        发布速度指令到电机
        
        这是最底层的控制方法，直接控制线速度和角速度。
        
        Args:
            linear_x (float): 线速度 (m/s)，正值表示前进，负值表示后退
            angular_z (float): 角速度 (rad/s)，正值表示左转，负值表示右转
            
        Example:
            >>> motor.move(linear_x=0.5, angular_z=0.2)  # 前进且轻微右转
        """
        # 创建 Twist 消息
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)
        
        # 发布
        self._publisher.publish(msg)
        self._node.get_logger().debug(
            f"电机指令: linear_x={linear_x:.2f} m/s, angular_z={angular_z:.2f} rad/s"
        )
    
    def move_forward(self, speed: float) -> None:
        """
        让机器人前进
        
        Args:
            speed (float): 前进速度 (m/s)，正数表示前进，负数表示后退
            
        Example:
            >>> motor.move_forward(0.5)   # 以 0.5 m/s 前进
            >>> motor.move_forward(-0.3)  # 以 0.3 m/s 后退
        """
        self.move(linear_x=speed, angular_z=0.0)
    
    def move_backward(self, speed: float) -> None:
        """
        让机器人后退
        
        Args:
            speed (float): 后退速度 (m/s)，通常为正数
            
        Example:
            >>> motor.move_backward(0.3)  # 以 0.3 m/s 后退
        """
        self.move(linear_x=-speed, angular_z=0.0)
    
    def turn_left(self, angular_speed: float) -> None:
        """
        让机器人左转
        
        Args:
            angular_speed (float): 角速度 (rad/s)，通常为正数
            
        Example:
            >>> motor.turn_left(1.57)  # 以 1.57 rad/s (90°/s) 左转
        """
        self.move(linear_x=0.0, angular_z=angular_speed)
    
    def turn_right(self, angular_speed: float) -> None:
        """
        让机器人右转
        
        Args:
            angular_speed (float): 角速度 (rad/s)，通常为正数
            
        Example:
            >>> motor.turn_right(1.57)  # 以 1.57 rad/s 右转
        """
        self.move(linear_x=0.0, angular_z=-angular_speed)
    
    def turn(self, angular_speed: float) -> None:
        """
        让机器人原地转圈
        
        Args:
            angular_speed (float): 角速度 (rad/s)，正值表示左转，负值表示右转
            
        Example:
            >>> motor.turn(1.57)   # 以 1.57 rad/s 左转
            >>> motor.turn(-1.57)  # 以 1.57 rad/s 右转
        """
        self.move(linear_x=0.0, angular_z=angular_speed)
    
    def move_forward_and_turn(self, linear_speed: float, angular_speed: float) -> None:
        """
        让机器人以指定速度前进并转向
        
        这是最灵活的方法，允许同时控制线速度和角速度。
        
        Args:
            linear_speed (float): 线速度 (m/s)
            angular_speed (float): 角速度 (rad/s)
            
        Example:
            >>> motor.move_forward_and_turn(0.5, 0.2)  # 前进同时轻微右转
        """
        self.move(linear_x=linear_speed, angular_z=angular_speed)
    
    def stop(self) -> None:
        """
        停止电机
        
        立即停止机器人的所有运动（线速度和角速度均为 0）。
        
        Example:
            >>> motor.stop()  # 停止运动
        """
        self.move(linear_x=0.0, angular_z=0.0)
        self._node.get_logger().info("电机已停止")
    
    def emergency_stop(self) -> None:
        """
        紧急停止电机
        
        与 stop() 相同，但会输出警告日志以表明这是紧急操作。
        
        Example:
            >>> motor.emergency_stop()  # 紧急停止
        """
        self.move(linear_x=0.0, angular_z=0.0)
        self._node.get_logger().warn("⚠️ 电机紧急停止！")
