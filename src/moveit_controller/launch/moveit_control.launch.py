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
        moveit_control
    ])
