"""
启动MoveIt命名位姿控制服务节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成Launch描述"""
    
    # MoveIt命名位姿控制器节点
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
        moveit_pose_controller_node
    ])
