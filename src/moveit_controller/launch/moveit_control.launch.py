"""
MoveIt控制器启动文件

该launch文件负责：
1. 加载moveit_controller.yaml配置文件中的参数
2. 启动moveit_control节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """生成launch描述
    Returns:
        LaunchDescription: 包含节点和参数配置的launch描述
    """
    moveit_control_config = os.path.join(
        get_package_share_directory('moveit_controller'),
        'config',
        'moveit_controller.yaml'
    ) 

    fruit_arm_launch_pkg_path = get_package_share_directory('arm_config')
    fruit_arm_launch_file_path = os.path.join(
        fruit_arm_launch_pkg_path,
        'launch',
        'fruit_arm.launch.py'
    )
    
    # 引入机械臂的launch文件
    fruit_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([fruit_arm_launch_file_path]),
    )

    # 创建moveit_control节点
    moveit_control = Node(
        package='moveit_controller',
        executable='moveit_control',
        name='moveit_control',
        output='screen',
        parameters=[
            moveit_control_config
                  ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # fruit_arm_launch,
        moveit_control,
    ])
