#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/dm_imu',
        description='IMU串口设备路径'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='921600',
        description='IMU串口波特率'
    )
    
    source_imu_topic_arg = DeclareLaunchArgument(
        'source_imu_topic',
        default_value='imu/data',
        description='源IMU数据话题名称'
    )
    
    target_imu_topic_arg = DeclareLaunchArgument(
        'target_imu_topic',
        default_value='imu',
        description='目标IMU数据话题名称'
    )

    # IMU驱动节点
    imu_driver_node = Node(
        package='dm_imu',
        executable='dm_imu_node',
        name='dm_imu_driver',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud')
        }],
        output='screen'
    )

    # IMU TF变换节点
    imu_tf_transform_node = Node(
        package='dm_imu',
        executable='imu_tf_transform_node',
        name='imu_tf_transform',
        parameters=[{
            'source_imu_topic': LaunchConfiguration('source_imu_topic'),
            'target_imu_topic': LaunchConfiguration('target_imu_topic'),
            'source_frame': 'imu_link',
            'target_frame': 'imu_1',
            'base_frame': 'base_link'
        }],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        source_imu_topic_arg,
        target_imu_topic_arg,
        imu_driver_node,
        imu_tf_transform_node
    ]) 