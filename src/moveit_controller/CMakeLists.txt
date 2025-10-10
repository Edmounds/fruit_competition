cmake_minimum_required(VERSION 3.8)
project(moveit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# 添加可执行文件
add_executable(moveit_control
  src/moveit_control.cpp
  src/utils.cpp
)


ament_target_dependencies(moveit_control rclcpp moveit_ros_planning_interface)

# 为你的目标（可执行文件）添加头文件搜索路径
target_include_directories(moveit_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 安装可执行文件
install(TARGETS
  moveit_control
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