cmake_minimum_required(VERSION 3.8)
project(moveit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(robot_control_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

# 添加可执行文件
add_executable(moveit_control
  src/moveit_control.cpp
)

ament_target_dependencies(moveit_control
  rclcpp
  moveit_ros_planning_interface
  robot_control_interfaces
)

# 添加 moveit_servo 可执行文件
add_executable(moveit_servo
  src/moveit_servo.cpp
)

ament_target_dependencies(moveit_servo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  robot_control_interfaces
)

add_executable(cartesian_path_demo
  src/cartesian_path_demo.cpp
)

ament_target_dependencies(cartesian_path_demo
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  robot_control_interfaces
)


# 为 moveit_servo 添加头文件搜索路径
target_include_directories(moveit_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# 为你的目标（可执行文件）添加头文件搜索路径
target_include_directories(moveit_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# target_include_directories(simple_servo_test PUBLIC
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>
# )

# 安装可执行文件
install(TARGETS
  moveit_control
  moveit_servo
  cartesian_path_demo
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装include目录下的所有头文件
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()