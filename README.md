# 依赖包




# servo5/6开关
## 查看 servo5 当前值
ros2 param get /data_merger_node servo5

## 查看 servo6 当前值
ros2 param get /data_merger_node servo6


# lookup tf tree
ros2 run tf2_tools view_frames

# Cartographer
## 保存为.pbstream文件（包含完整的Cartographer状态）
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

## 然后写入文件
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/rc1/fruit_ws/src/localization/maps/map.pbstream'}"