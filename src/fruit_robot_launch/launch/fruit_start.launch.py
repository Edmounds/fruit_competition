from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import LogInfo

def generate_launch_description():

    #serial
    #        'serial_sender = serial_pkg.serial_sender:main',
    data_merger_node = Node(
        package='serial_pkg',
        executable='data_merger_node',
        name='data_merger_node',
        output='screen',
        parameters=[
            {'--args': '--ros-args --log-level info'}
        ],
        emulate_tty=True,
    )
    
    serial_sender = Node(
        package='serial_pkg',
        executable='serial_sender',
        name='serial_sender',
        output='screen',
        parameters=[
            {'port': '/dev/serial_ch340'}, 
            {'baudrate': 115200},
            {'timeout': 0.1},
            {'--args': '--ros-args --log-level debug'}
        ],
        emulate_tty=True,
    )
    
    # tf publisher
    
    # lidar# cartographer
    lidar_launch_path = os.path.join(get_package_share_directory('fruit_robot_launch'),
                                      'launch',
                                      'lidar.launch.py'
    )
    
    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lidar_launch_path),
            )

    # gemini camera
    
    #=============================================================fruit_arm=============================================================

    #moveit,同时启动serial_pkg中的fruit_arm_driver节点
    fruit_arm_launch_path = os.path.join(
        get_package_share_directory('arm_config'),
        'launch',
        'fruit_arm.launch.py'
    )
    
    moveit_control_config = os.path.join(
        get_package_share_directory('moveit_controller'),
        'config',
        'moveit_controller.yaml'
    )   
    
    fruit_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fruit_arm_launch_path)
    )
    
    
    moveit_control = Node(
        package='moveit_controller',
        executable='moveit_control',
        name='moveit_control',
        output='screen',
        parameters=[
            moveit_control_config
                ],
        emulate_tty=True
    )
    #=============================================================fruit_arm=============================================================
    # moveit_control
    
    # fruit_detector
    
    # task_decision
    
    # rviz
    
    #virtual_control
    manual_control_launch_path = os.path.join(
        get_package_share_directory('moveit_controller'),
        'launch',
        'manual_control.launch.py'
    )
    
    # Include the manual control launch file
    manual_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(manual_control_launch_path)
    ) 
    
    
    # 注册事件处理器：当 moveit_control 退出时仅打印日志，不关闭其他节点
    #ROS2 humble里面同一个launch文件,有一个node exit之后,默认会把其他node也exit掉
    moveit_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=moveit_control,
            on_exit=[
                LogInfo(msg="moveit_control has exited, but other nodes continue running.")
            ]
        )
    )
     
    return LaunchDescription([
        data_merger_node,
        fruit_arm,
        # moveit_control,
        moveit_exit_handler,
        serial_sender,
        lidar_launch,
        manual_control
    ])