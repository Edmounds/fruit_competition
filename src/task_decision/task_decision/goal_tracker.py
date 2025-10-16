"""
目标追踪器 - 订阅位姿数据并判断是否到达目标点

功能：
    - 订阅pose_data话题
    - 计算与目标点的距离和角度误差
    - 判断是否到达目标
    - 提供到达事件回调
"""

import math
from typing import Callable, Optional, Tuple
from geometry_msgs.msg import Pose2D
import rclpy
from rclpy.node import Node


class GoalTracker:
    """
    目标追踪器类
    
    功能：
        - 跟踪当前机器人位姿
        - 计算与目标的距离和角度误差
        - 判断是否到达目标
        - 提供到达成功的回调机制
    
    属性：
        distance_tolerance (float): 距离误差容限 (米，默认0.1m)
        angle_tolerance (float): 角度误差容限 (弧度，默认0.1rad≈5.7°)
    """
    
    def __init__(
        self,
        node: Node,
        distance_tolerance: float = 0.1,
        angle_tolerance: float = 0.1
    ):
        """
        初始化目标追踪器
        
        Args:
            node (Node): ROS 2 节点引用，用于创建订阅
            distance_tolerance (float): 位置误差容限，单位米 (默认0.1m)
            angle_tolerance (float): 角度误差容限，单位弧度 (默认0.1rad)
        """
        self.node = node
        self.distance_tolerance = distance_tolerance
        self.angle_tolerance = angle_tolerance
        
        # 当前位姿
        self._current_pose: Optional[Pose2D] = None
        
        # 目标位姿
        self._target_pose: Optional[Pose2D] = None
        self._target_name: str = ""
        
        # 到达状态
        self._reached: bool = False
        
        # 回调函数
        self._on_reached_callback: Optional[Callable[[str], None]] = None
        self._on_near_target_callback: Optional[Callable[[str, float, float], None]] = None
        
        # 创建订阅
        self.pose_subscription = node.create_subscription(
            Pose2D,
            'pose_data',
            self._pose_callback,
            10
        )
        
        node.get_logger().info(
            f'GoalTracker已初始化 (距离容限={distance_tolerance:.3f}m, '
            f'角度容限={angle_tolerance:.3f}rad)'
        )
    
    def _pose_callback(self, msg: Pose2D) -> None:
        """
        位姿订阅回调函数
        
        Args:
            msg (Pose2D): 机器人当前位姿
        """
        self._current_pose = msg
        
        # 如果设置了目标点，判断是否到达
        if self._target_pose is not None:
            distance, angle_error = self.get_error()
            
            # 检查是否到达
            is_reached = (
                distance <= self.distance_tolerance and
                angle_error <= self.angle_tolerance
            )
            
            # 触发到达回调
            if is_reached and not self._reached:
                self._reached = True
                self.node.get_logger().info(
                    f'✓ 已到达目标点 [{self._target_name}]'
                )
                if self._on_reached_callback:
                    self._on_reached_callback(self._target_name)
            
            # 触发接近目标回调（每次都检查，即使已到达）
            if not is_reached and self._on_near_target_callback:
                self._on_near_target_callback(self._target_name, distance, angle_error)
    
    def set_target(
        self,
        x: float,
        y: float,
        theta: float,
        target_name: str = "target"
    ) -> None:
        """
        设置目标位置
        
        Args:
            x (float): 目标X坐标 (米)
            y (float): 目标Y坐标 (米)
            theta (float): 目标朝向 (弧度)
            target_name (str): 目标名称，用于日志和回调 (默认"target")
        """
        pose2d = Pose2D()
        pose2d.x = x
        pose2d.y = y
        pose2d.theta = theta
        
        self.set_target_pose(pose2d, target_name)
    
    def set_target_pose(self, pose: Pose2D, target_name: str = "target") -> None:
        """
        设置目标位姿对象
        
        Args:
            pose (Pose2D): 目标位姿
            target_name (str): 目标名称
        """
        self._target_pose = pose
        self._target_name = target_name
        self._reached = False
        
        self.node.get_logger().info(
            f'设置目标点 [{target_name}]: x={pose.x:.3f}, y={pose.y:.3f}, '
            f'theta={pose.theta:.3f}'
        )
    
    def clear_target(self) -> None:
        """清除目标点"""
        self._target_pose = None
        self._target_name = ""
        self._reached = False
        self.node.get_logger().debug('目标点已清除')
    
    def get_current_pose(self) -> Optional[Pose2D]:
        """
        获取当前位姿
        
        Returns:
            Optional[Pose2D]: 当前位姿，或None（如果尚未接收到数据）
        """
        return self._current_pose
    
    def get_target_pose(self) -> Optional[Pose2D]:
        """
        获取目标位姿
        
        Returns:
            Optional[Pose2D]: 目标位姿，或None
        """
        return self._target_pose
    
    def get_error(self) -> Tuple[float, float]:
        """
        获取当前与目标的误差
        
        Returns:
            Tuple[float, float]: (距离误差(米), 角度误差(弧度))
                                 如果没有当前位姿或目标，返回(float('inf'), float('inf'))
        """
        if self._current_pose is None or self._target_pose is None:
            return (float('inf'), float('inf'))
        
        # 计算位置距离
        dx = self._target_pose.x - self._current_pose.x
        dy = self._target_pose.y - self._current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # 计算角度误差
        angle_error = abs(self._target_pose.theta - self._current_pose.theta)
        # 处理角度环绕（取最小角度）
        if angle_error > math.pi:
            angle_error = 2 * math.pi - angle_error
        
        return (distance, angle_error)
    
    def is_reached(self) -> bool:
        """
        检查是否已到达目标
        
        Returns:
            bool: True表示已到达，False表示未到达或无目标
        """
        return self._reached
    
    def is_near_target(self, distance_threshold: float = 0.2) -> bool:
        """
        检查是否接近目标（在一定距离内）
        
        Args:
            distance_threshold (float): 距离阈值 (米，默认0.2m)
            
        Returns:
            bool: True表示接近，False表示远离或无目标
        """
        if self._target_pose is None or self._current_pose is None:
            return False
        
        distance, _ = self.get_error()
        return distance <= distance_threshold
    
    def on_reached(self, callback: Callable[[str], None]) -> None:
        """
        注册到达目标的回调函数
        
        当机器人首次到达目标点时调用（仅一次）
        
        Args:
            callback (Callable): 回调函数，接收target_name作为参数
                                 签名: def callback(target_name: str) -> None
        
        Example:
            >>> def on_goal_reached(target_name: str):
            ...     print(f"到达 {target_name}")
            >>> tracker.on_reached(on_goal_reached)
        """
        self._on_reached_callback = callback
    
    def on_near_target(
        self,
        callback: Callable[[str, float, float], None]
    ) -> None:
        """
        注册接近目标的回调函数
        
        当机器人未到达但距离在容限外时调用（持续调用）
        
        Args:
            callback (Callable): 回调函数，接收(target_name, distance, angle_error)
                                 签名: def callback(name: str, dist: float, angle: float) -> None
        
        Example:
            >>> def on_near_goal(target_name: str, distance: float, angle: float):
            ...     print(f"接近 {target_name}: 距离={distance:.3f}m, 角度误差={angle:.3f}rad")
            >>> tracker.on_near_target(on_near_goal)
        """
        self._on_near_target_callback = callback
    
    def print_status(self) -> str:
        """
        打印追踪器状态
        
        Returns:
            str: 状态信息
        """
        status = "=== 目标追踪器状态 ===\n"
        
        if self._current_pose:
            status += f"当前位姿: x={self._current_pose.x:.3f}, y={self._current_pose.y:.3f}, theta={self._current_pose.theta:.3f}\n"
        else:
            status += "当前位姿: 未接收\n"
        
        if self._target_pose:
            status += f"目标位姿 [{self._target_name}]: x={self._target_pose.x:.3f}, y={self._target_pose.y:.3f}, theta={self._target_pose.theta:.3f}\n"
            distance, angle_error = self.get_error()
            status += f"误差: 距离={distance:.3f}m (容限{self.distance_tolerance}m), 角度={angle_error:.3f}rad (容限{self.angle_tolerance}rad)\n"
            status += f"到达状态: {'✓ 已到达' if self._reached else '✗ 未到达'}\n"
        else:
            status += "目标位姿: 未设置\n"
        
        return status
