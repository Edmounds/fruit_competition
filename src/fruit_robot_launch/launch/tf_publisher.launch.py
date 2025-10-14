from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path    

    
    
def generate_launch_description():

    ld = LaunchDescription()

    virtual_joints_launch_path = os.path.join(get_package_share_directory('arm_config')+'/launch/static_virtual_joint_tfs.launch.py')
    rsp_launch_path = os.path.join(get_package_share_directory('arm_config')+'/launch/rsp.launch.py')

    if virtual_joints_launch_path:
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch_path)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rsp_launch_path)),
        )
    )
    
    return ld