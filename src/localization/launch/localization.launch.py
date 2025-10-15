import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='localization').find('localization')
    imu_launch_path = os.path.join(FindPackageShare(package='dm_imu').find('dm_imu')+'/launch/imu_with_transform.launch.py')
    
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([imu_launch_path]),
    )
    
    
    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 地图的分辨率
    # resolution = LaunchConfiguration('resolution', default='0.05')
    # # 地图的发布周期
    # publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='localization_2d.lua')
    
    configuration_basemap = LaunchConfiguration('configuration_basemap', default= os.path.join(pkg_share, 'maps/map.pbstream') )
    
    rviz_config_dir = os.path.join(pkg_share, 'config')+"/cartographer.rviz"
    print(f"rviz config in {rviz_config_dir}")
    
    #=====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename,
                   '-load_state_filename', configuration_basemap]
                   )

    # cartographer_occupancy_grid_node = Node(
    #     package='cartographer_ros',
    #     executable='cartographer_occupancy_grid_node',
    #     name='cartographer_occupancy_grid_node',
    #     output='screen',
    #     # parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        # parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    # ld.add_action(cartographer_occupancy_grid_node)
    ld.add_action(rviz_node)
    ld.add_action(imu_launch)

    return ld