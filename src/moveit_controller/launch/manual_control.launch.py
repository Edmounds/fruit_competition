from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():

    urdf_path = os.path.join(
    get_package_share_directory('car_description'),
    'urdf',
    'car.xacro'
    )
    
        # 从 URDF 加载机器人描述
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}

    
    return LaunchDescription([
            # 启动 joint_state_publisher_gui 节点
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            remappings=[
                ('/joint_states', '/arm_control_data')  # 重映射到 /arm_control_data
            ],
            parameters=[{
                'source_list': ['j1', 'j2', 'j3', 'j4']  # 只控制这四个关节
            }],
            output='screen',
        ),

        # 启动 robot_state_publisher 节点 (它会订阅 /joint_states 并发布 tf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
 
        # Node(
        #     package='serial_pkg',
        #     executable='serial_receiver',
        #     name='serial_receiver',
        #     output='screen',
        # ),

        #rqt_robot_steering
        Node(
            package='rqt_robot_steering',
            executable='rqt_robot_steering',
            name='rqt_robot_steering_node', 
            output='screen',
            # parameters=[{
            #     'default_stamped': True,
            #     'default_vx_max': 30.0,
            #     'default_vx_min': -30.0,
            #     'default_vw_min': -40.0,
            #     'default_vw_max': 40.0,
            # }]
        ),
        
        Node(
            package='serial_pkg',
            executable='data_merger_node',
            name='data_merger_node',
            output='screen',
        ),

        Node(
            package='serial_pkg',
            executable='serial_sender',
            name='serial_sender',
            output='screen', 
            parameters=[{'--args': '--ros-args --log-level debug'}]
        ),
    ])