import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():


    # 1. 找到 orbbec_camera 包的路径
    orbbec_camera_pkg_path = get_package_share_directory('orbbec_camera')

    # 2. 定义要引用的 launch 文件的完整路径
    orbbec_launch_file_path = os.path.join(
        orbbec_camera_pkg_path,
        'launch',
        'gemini_330_series.launch.py'
    )

    # 3. 创建一个 IncludeLaunchDescription 动作来引入 Orbbec 相机的启动文件
    orbbec_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([orbbec_launch_file_path]),
        # 如果需要，你可以在这里向 orbbec 的 launch 文件传递参数
        launch_arguments={'camera_name': 'camera',
                          'depth_registration': 'true',
                          # 'enable_color_auto_exposure': 'true',
                          # 'enable_color_auto_white_balance': 'true',
                          'enable_d2c_viewer':'true',
                          'enable_frame_sync':'true',
                          # 'enable_colored_point_cloud':'true',
                          'align_mode':'HW',
                          'ordered_pc':'true',
                            'color_width':'640',
                            'color_height':'480',
                            'color_fps':'60',
                            'depth_width':'640',
                            'depth_height':'480',
                            'depth_fps':'60'
                          }.items(),
    )

  
    # 5. 将所有动作组合到 LaunchDescription 中并返回
    return LaunchDescription([
        orbbec_camera_launch,
    ])