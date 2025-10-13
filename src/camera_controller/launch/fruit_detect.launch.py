import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    

    # 1. 找到 orbbec_camera 包的路径
    orbbec_camera_pkg_path = get_package_share_directory('camera_controller')

    # 2. 定义要引用的 launch 文件的完整路径
    orbbec_launch_file_path = os.path.join(
        orbbec_camera_pkg_path,
        'launch',
        'gemini335.launch.py'
    )

    # 3. 创建一个 IncludeLaunchDescription 动作来引入 Orbbec 相机的启动文件
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orbbec_launch_file_path]),
        # 如果需要，你可以在这里向 orbbec 的 launch 文件传递参数

    )
    
    fruit_detect_node = Node(
        package='camera_controller',
        executable='fruit_detect',
        name='fruit_detect',
        output='screen',
        parameters=[
            # 可以在这里添加参数
                  ],
        emulate_tty=True,
    )

    depth_getter_node = Node(
        package='camera_controller',
        executable='get_depth',
        name='get_depth',
        output='screen',
        parameters=[
            # 可以在这里添加参数
                  ],
        emulate_tty=True,
    )
    

  
    # 5. 将所有动作组合到 LaunchDescription 中并返回
    return LaunchDescription([
        orbbec_camera_launch,
        fruit_detect_node,
        depth_getter_node
    ])