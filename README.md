# 依赖包




# 切换 servo5
ros2 service call /toggle_servo5 std_srvs/srv/Trigger

# 切换 servo6
ros2 service call /toggle_servo6 std_srvs/srv/Trigger

# lookup tf tree
ros2 run tf2_tools view_frames

# Cartographer
## 保存为.pbstream文件（包含完整的Cartographer状态）
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

## 然后写入文件
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/rc1/fruit_ws/src/localization/maps/map.pbstream'}"