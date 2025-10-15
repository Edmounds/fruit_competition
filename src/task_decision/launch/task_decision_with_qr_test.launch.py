from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():
    

    
    qr_node = Node(
        package='qr_recognition',
        executable='qr_recognition_node',
        name='qr_recognition_node',
        output='screen',
    )
    
    task_decision_pack = Node(
        package='task_decision',
        executable='task_decision_node',
        name='task_decision_node',
        output='screen',
    )
    
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )

    return LaunchDescription([
        qr_node,
        task_decision_pack,
        # rviz_node   
    ])