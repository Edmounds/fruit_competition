cmake_minimum_required(VERSION 3.8)
project(fruit_detector)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_control_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED COMPONENTS Runtime
  PATHS /usr/lib/cmake/openvino2025.3.0
  NO_DEFAULT_PATH
)

# Include directories
include_directories(include)

# 创建水果检测器库（不再生成自己的消息接口）
add_library(fruit_detector_component SHARED
  src/fruit_detector.cpp
  src/openvino_detect.cpp
)

# 设置编译选项
target_compile_definitions(fruit_detector_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

# 添加依赖项
ament_target_dependencies(fruit_detector_component
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  image_transport
  std_msgs
  vision_msgs
  robot_control_interfaces
  OpenCV
  OpenVINO
)

# 不再需要链接自己的消息接口库，使用 robot_control_interfaces 的即可
# Explicitly link OpenVINO libraries
target_link_libraries(fruit_detector_component
  openvino::runtime
)

# 不再需要这个依赖，因为不生成自己的消息
# add_dependencies(fruit_detector_component ${PROJECT_NAME})

rclcpp_components_register_nodes(fruit_detector_component "fruit_detector::FruitDetector")

# Create the standalone executable
add_executable(fruit_detector_node
  src/fruit_detector_node.cpp
)
target_link_libraries(fruit_detector_node
  fruit_detector_component
)
ament_target_dependencies(fruit_detector_node
  rclcpp
)

# Install targets
install(TARGETS
  fruit_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  fruit_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install model files
install(
  DIRECTORY model
  DESTINATION share/${PROJECT_NAME}
)

ament_package() 