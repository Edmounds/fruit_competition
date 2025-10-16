from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import LogInfo

def generate_launch_description():

    # fruit_arm_launch_path = os.path.join(
    #     get_package_share_directory('arm_config'),
    #     'launch',
    #     'fruit_arm.launch.py'
    # )    

    
    lidar_launch_path = os.path.join(get_package_share_directory('lslidar_driver'),
                                                    'launch', 'lsn10p_launch.py'
    )
    
    lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch_path),
            )

    rviz_config_file = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'cartographer.rviz'
    )

    cartographer_localization_path= os.path.join(
        get_package_share_directory('localization'),
        'launch',
        'mapping.launch.py'
    )

    # cartographer_localization_path= os.path.join(
    #     get_package_share_directory('localization'),
    #     'launch',
    #     'mapping.launch.py'
    # )    

    cartographer_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cartographer_localization_path),
            )

    # fruit_arm_launch = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(fruit_arm_launch_path)
    #         )
    
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    return LaunchDescription([
        # fruit_arm_launch,
        lidar,
        cartographer_localization,
        rviz_node,
        

    ])

