cmake_minimum_required(VERSION 3.8)
project(camera_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # C++17标准
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(ament_index_cpp REQUIRED)

# 包含目录
include_directories(include)

# 创建可执行文件
add_executable(fruit_detector_node
  src/detect_fruit.cpp
)

ament_target_dependencies(fruit_detector_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)

target_link_libraries(fruit_detector_node
  openvino::runtime
)

# 安装
install(TARGETS
  fruit_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
