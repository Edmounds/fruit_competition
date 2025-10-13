"""
水果检测节点启动文件

订阅: /image (sensor_msgs/Image)
发布: /detections (vision_msgs/Detection2DArray)
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
from launch.substitutions import  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
      # 加载rviz配置文件
    rviz_config_file = os.path.join(
        get_package_share_directory('camera_controller'),
        'rviz',
        'camera.rviz'
    )
    # 4. 创建并配置 RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        emulate_tty=True,
    )
    # 使用gemini335launch
    gemini335_launch_file = PathJoinSubstitution(
        [FindPackageShare('camera_controller'), 'launch', 'gemini335.launch.py']
    )
    
    #launch文件
    gemini335_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gemini335_launch_file]),
    )

    # 定义水果检测节点
    fruit_detector_node = Node(
        package='camera_controller',
        executable='fruit_detector_node',
        name='fruit_detector_node',
        output='screen',
    )
    
    
    return LaunchDescription([
        gemini335_launch,
        fruit_detector_node,
        rviz_node
        
    ])