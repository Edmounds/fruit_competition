#!/usr/bin/env python3
"""
MoveIt视觉伺服控制器启动文件

启动基于视觉检测的机械臂伺服控制节点，实现自动对齐和抓取功能
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # 创建MoveIt Servo节点
    moveit_cartesian_path_node = Node(
        package='moveit_controller',
        executable='cartesian_controller',
        name='cartesian_controller',
    )
    
    moveit_pose_controller_node = Node(
        package='moveit_controller',
        executable='moveit_pose_controller',
        name='moveit_pose_controller',
        output='screen',
        parameters=[
            # 可以在这里添加参数
        ]
    )    
    
    return LaunchDescription([
        moveit_cartesian_path_node,
        moveit_pose_controller_node
    ])
