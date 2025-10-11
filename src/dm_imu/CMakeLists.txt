cmake_minimum_required(VERSION 3.8) # ROS2要求CMake 3.8+
project(dm_imu)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

# 1. 替换catkin为ament
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # 添加sensor_msgs依赖
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)      # 添加tf2_ros依赖
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED) # ROS2官方串口包
find_package(io_context REQUIRED) # IO上下文包

# 2. 添加C++标准要求（ROS2默认使用C++14/17）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 3. 定义可执行文件
add_executable(${PROJECT_NAME}_node
  src/bsp_crc.cpp
  src/imu_driver.cpp
  src/test_imu.cpp
)

# 添加IMU TF变换节点
add_executable(imu_tf_transform_node
  src/imu_tf_transform_node.cpp
)

# 添加头文件路径到包含目录
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 可以尝试使用ament_target_dependencies来简化依赖处理
# 它会自动处理包含目录和链接库
ament_target_dependencies(${PROJECT_NAME}_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"  # 添加sensor_msgs依赖
  "tf2_ros"      # 添加tf2_ros依赖
  "tf2"          # 添加tf2依赖
  "tf2_geometry_msgs"
  "serial_driver"
  "io_context"
)

# 为IMU TF变换节点添加依赖
ament_target_dependencies(imu_tf_transform_node
  "rclcpp"
  "sensor_msgs"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
)
# 如果某些依赖项不支持ament_target_dependencies，可以再回退到显式指定
# target_include_directories(${PROJECT_NAME}_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   ${rclcpp_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS}
#   ${rcl_interfaces_INCLUDE_DIRS}
#   ${sensor_msgs_INCLUDE_DIRS}
# )
# target_link_libraries(${PROJECT_NAME}_node
#   rclcpp
#   # std_msgs::std_msgs
#   # serial::serial  # 根据实际serial包名称调整
# )

# 4. 安装目标
install(TARGETS ${PROJECT_NAME}_node imu_tf_transform_node
  DESTINATION lib/${PROJECT_NAME}
)

# 5. 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  serial_driver
  io_context
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 6. 必须的ament宏，确保这是最后一个ament相关的调用
ament_package()

