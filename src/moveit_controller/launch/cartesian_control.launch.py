#!/usr/bin/env python3
"""
笛卡尔控制器完整启动文件

功能：
1. 启动 fruit_arm_driver（提供 joint_states 和执行轨迹）
2. 启动 cartesian_controller（提供笛卡尔空间移动服务）
3. 启动 robot_state_publisher（发布 TF 变换）

使用方法：
    ros2 launch moveit_controller cartesian_control.launch.py

然后调用服务：
    ros2 service call /move_to_position robot_control_interfaces/srv/MoveToPosition "{x: 0.0, y: 0.0, z: -0.1}"
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # 加载机器人描述（用于 robot_state_publisher）
    urdf_path = os.path.join(
        get_package_share_directory('car_description'),
        'urdf',
        'car.xacro'
    )
    
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    
    # 1. 启动 robot_state_publisher（订阅 joint_states，发布 TF）
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # 2. 启动 fruit_arm_driver（发布 joint_states，执行轨迹）
    fruit_arm_driver_node = Node(
        package='serial_pkg',
        executable='fruit_arm_driver',
        name='fruit_arm_driver',
        output='screen',
    )
    
    # 3. 启动 cartesian_controller（笛卡尔空间移动服务）
    cartesian_controller_node = Node(
        package='moveit_controller',
        executable='cartesian_controller',
        name='cartesian_controller',
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        fruit_arm_driver_node,
        cartesian_controller_node,
    ])
