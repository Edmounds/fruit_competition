"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 使用完整的配置构建方式
    moveit_config = MoveItConfigsBuilder("arm", package_name="arm_config")
    
    # 加载所有必要的配置文件
    moveit_config.robot_description(file_path="config/arm.urdf.xacro")
    moveit_config.robot_description_semantic(file_path="config/arm.srdf")
    moveit_config.robot_description_kinematics(file_path="config/kinematics.yaml")
    moveit_config.trajectory_execution(file_path="config/moveit_controllers.yaml")
    moveit_config.planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner", "chomp"])
    moveit_config.joint_limits(file_path="config/joint_limits.yaml")
    
    # 使用arm_control包中的moveit_config.yaml来配置moveit_cpp
    moveit_cpp_config_file = os.path.join(
        get_package_share_directory("arm_control"),
        "config",
        "moveit_config.yaml"
    )
    moveit_config.moveit_cpp(file_path=moveit_cpp_config_file)
    
    # 转换为MoveItConfigs对象
    configs = moveit_config.to_moveit_configs()

    moveit_py_node = Node(
        name="moveit_py",
        package="arm_control",
        executable="moveit_node",
        output="both",
        parameters=[configs.to_dict()],
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("arm_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            configs.robot_description,
            configs.robot_description_semantic,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[configs.robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("arm_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="log",
    )

    load_controllers = []
    for controller in [
        "arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="log",
            )
        ]

    # 加载move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("arm_config"), 'launch', 'move_group.launch.py')
        ]),
    )
    
    # 加载RViz
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("arm_config"), 'launch', 'moveit_rviz.launch.py')
        ]),
    )
    
    # 对于我们自己的moveit_node，我们不再使用move_group_launch和moveit_rviz_launch
    # 而是直接使用我们自己定义的节点
    return LaunchDescription(
        [
            robot_state_publisher,  # 首先发布机器人状态
            ros2_control_node,      # 然后启动控制节点
            static_tf,              # 设置TF转换
            rviz_node,              # 启动RViz可视化
            moveit_py_node,         # 最后启动我们的MoveItPy节点
        ] + load_controllers        # 加载控制器
    )