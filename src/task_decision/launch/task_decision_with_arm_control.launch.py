from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    

    # fruit_launch_dir = os.path.join(
    #     get_package_share_directory('fruit_detector'),'launch')
    # fruit_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(fruit_launch_dir,'fruit_detector.launch.py'))
    # )
    
    
    fruit_launch_dir = os.path.join(
        get_package_share_directory('fruit_robot_launch'),'launch')
    fruit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(fruit_launch_dir,'fruit_start.launch.py'))
    )

    # qr_node = Node(
    #     package='qr_recognition',
    #     executable='qr_recognition_node',
    #     name='qr_recognition_node',
    #     output='screen',
    # )
    

    
    task_decision_pack = Node(
        package='task_decision',
        executable='task_decision_node',
        name='task_decision_node',
        output='screen',
    )
    # moveit_cartesian_path_node = Node(
    #     package='moveit_controller',
    #     executable='cartesian_controller',
    #     name='cartesian_controller',
    # )
    
    moveit_pose_controller_node = Node(
        package='moveit_controller',
        executable='moveit_pose_controller',
        name='moveit_pose_controller',
        output='screen',
    )   
    

    
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    # )

    return LaunchDescription([
        # qr_node,
        task_decision_pack,
        # gemini_launch,
        # rviz_node,
        fruit_launch,
        # moveit_cartesian_path_node,
        moveit_pose_controller_node
    ])