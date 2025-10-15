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
    """
    生成Launch描述

    Returns:
        LaunchDescription: Launch描述对象
    """
    
    # 获取配置文件路径
    moveit_servo_config = PathJoinSubstitution([
        FindPackageShare('moveit_controller'),
        'config',
        'moveit_servo.yaml'
    ])
    
    # 声明launch参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value='fruit_detection',
        description='水果检测话题名称'
    )
    
    # 创建MoveIt Servo节点
    moveit_servo_node = Node(
        package='moveit_controller',
        executable='moveit_servo',
        name='moveit_servo_node',
        output='screen',
        parameters=[
            moveit_servo_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ],
        remappings=[
            ('fruit_detection', LaunchConfiguration('detection_topic'))
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        detection_topic_arg,
        moveit_servo_node
    ])
