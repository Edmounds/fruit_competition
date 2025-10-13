# Copilot Instructions for Fruit Robot ROS 2 Workspace

## 项目概览 (Project Overview)

这是一个基于ROS 2的水果采摘机器人系统，集成了机械臂控制、移动底盘导航、SLAM建图和YOLO视觉检测功能。

**核心功能模块**：
- **机械臂控制**: 4自由度机械臂，通过MoveIt规划和串口通信控制
- **移动底盘**: 差速驱动底盘，支持导航和自主移动
- **视觉感知**: YOLO目标检测用于水果识别
- **SLAM**: 环境建图和定位

## 核心包说明 (Key Packages)

### 控制与通信类
- **`serial_pkg`**: 串口通信核心，包含发送器(`serial_sender`)、接收器(`serial_receiver`)和机械臂驱动(`fruit_arm_driver`)
  - `fruit_arm_driver.py`: 关键节点，作为Action Server接收MoveIt轨迹并转换为串口命令
- **`robot_control_interfaces`**: 自定义消息和Action接口定义
  - `SerialData.msg`: 包含导航速度和5个舵机角度的控制消息
  - `MoveRobot.action`: 简单的距离移动Action（示例用）

### 机械臂相关
- **`arm_description`**: URDF/Xacro机器人描述文件和Gazebo仿真配置
  - 主文件: `urdf/arm.xacro`，引用 `materials.xacro` 和 `arm.gazebo`
  - STL网格文件在 `meshes/` 目录
- **`arm_config`**: MoveIt配置包（由MoveIt Setup Assistant生成，**不要手动编辑**）
  - 运动学、关节限位、控制器配置等
- **`moveit_controller`**: C++编写的MoveIt控制节点
  - 从YAML读取目标位姿参数，执行运动规划

### 应用类
- **`navigation`**: 导航功能包
- **`slam`**: SLAM建图包
- **`yolo_detect`**: YOLO视觉检测包
- **`fruit_robot_launch`**: 系统级launch文件集合
- **`example_action_rclpy`**: Action示例代码（学习参考）

## 关键架构与数据流 (Architecture & Data Flow)

### MoveIt到硬件的完整控制链
```
MoveIt规划器 → FollowJointTrajectory Action → fruit_arm_driver → SerialData消息 → 串口 → 下位机
```

**详细说明**：
1. MoveIt生成关节轨迹，通过`manipulator_controller`发送Action请求
2. `fruit_arm_driver`节点实现`FollowJointTrajectory` Action Server
   - Action名称: `joint_trajectory_controller/follow_joint_trajectory`
   - **必须使用MultiThreadedExecutor防止阻塞**
3. 将关节角度（弧度）转换为舵机脉冲值（0-1000或0-3600取决于协议）
4. 通过`control_data`话题发布`SerialData`消息
5. `serial_sender`节点订阅并打包为串口帧发送

### 控制器配置关键点
- **ROS 2 Control**: `arm_config/config/ros2_controllers.yaml`
  - 定义了`manipulator_controller` (JointTrajectoryController类型)
  - 关节名称: `j1, j2, j3, j4` (4个关节)
- **MoveIt Controller**: `arm_config/config/moveit_controllers.yaml`
  - 配置MoveIt如何与ros2_control交互
  - **注意**: 实际硬件控制通过自定义Action Server实现，不使用标准ros2_control硬件接口

## 自定义接口定义 (Custom Interfaces)

 - robot_control_interfaces/msg/SerialData.msg
 - robot_control_interfaces/msg/ArmControl.msg



### 阶段三：执行方案
**声明格式**：`【执行方案】`

**必须做的事**：
- 严格按照选定方案实现
- 修改后运行类型检查

**绝对禁止**：
- ❌ 提交代码（除非用户明确要求）
- 启动开发服务器

如果在这个阶段发现了拿不准的问题，请向我提问。

收到用户消息时，一般从【分析问题】阶段开始，除非用户明确指定阶段的名字。


## 串口通信协议 (Serial Communication Protocol)

### 帧结构
- **帧头**: `0xAA`
- **帧尾**: `0x55`
- **功能码**: 发送=`0x02`, 反馈=`0x03`
- **数据段**: 20字节
  - 字节3-6: `linear_x` (float32)
  - 字节7-10: `angular_z` (float32)
  - 字节11-12: `servo1` (uint16)
  - 字节13-14: `servo2` (uint16)
  - 字节15-16: `servo3` (uint16)
  - 字节17-18: `servo4` (uint16)
  - 字节19-20: `servo5` (uint16)

### 实现位置
- **发送**: `serial_pkg/serial_controller.py` 中的 `SerialServer` 类
- **数据打包**: `_prepare_send_frame()` 方法使用 `struct.pack()`
- **接收解析**: `_process_buffer()` 方法处理反馈数据

## 开发工作流 (Development Workflows)

### 构建系统
```bash
# 完整构建所有包
colcon build

# 构建特定包
colcon build --packages-select serial_pkg

# 构建时显示详细输出
colcon build --event-handlers console_direct+

# 仅构建Python包（跳过CMake）
colcon build --packages-select serial_pkg --symlink-install
```

### 环境设置
```bash
# 每次新终端都需要source
source install/setup.bash
```

### 常用启动命令
```bash
# 启动MoveIt + RViz可视化
ros2 launch arm_config fruit_arm.launch.py

# 启动Gazebo仿真
ros2 launch arm_description gazebo.launch.py

# 启动MoveIt控制节点
ros2 launch moveit_controller moveit_control.launch.py

# 运行串口节点
ros2 run serial_pkg fruit_arm_driver
ros2 run serial_pkg serial_sender
ros2 run serial_pkg serial_receiver
```

### 调试技巧
- **查看话题**: `ros2 topic list` / `ros2 topic echo /control_data`
- **查看Action**: `ros2 action list` / `ros2 action info /joint_trajectory_controller/follow_joint_trajectory`
- **查看TF树**: `ros2 run tf2_tools view_frames`
- **RViz配置**: 使用 `arm_config/config/moveit.rviz` 或 `arm_description/config/display.rviz`

## 关键规则 (Critical Rules)

### 禁止手动编辑
- ❌ **不要编辑 `arm_config` 包的配置文件**
  - 这些文件由MoveIt Setup Assistant生成
  - 如需修改，重新运行Setup Assistant: `ros2 launch arm_config setup_assistant.launch.py`

### 必须遵守的约定
- ✅ **使用 autoDocstring 扩展生成中文文档字符串**
  - 所有新建或修改的类和函数都必须添加
- ✅ **Action Server 必须使用 MultiThreadedExecutor**
  - 防止Action回调阻塞主循环，参考 `fruit_arm_driver.py`
- ✅ **Launch文件优先使用Python格式**
  - 不使用XML格式launch文件
- ✅ **关节角度转换注意单位**
  - MoveIt使用弧度 (radians)
  - 舵机使用脉冲值或角度值（根据协议，0-1000或0-3600）
  - 转换函数: `radians_to_servo_pulse()`

### MoveIt配置要点
- **运动组**: 通常命名为 `manipulator` 或类似
- **关节名称**: `j1, j2, j3, j4` (4关节机械臂)
- **末端执行器**: 通过 `servo5` 控制夹爪开合
- **规划器**: 配置在 `arm_config/config/` 中
- **URDF主文件**: `arm_description/urdf/arm.xacro`
  - 引用: `materials.xacro`, `arm.gazebo`
  - 网格文件: `arm_description/meshes/*.stl`

## 代码风格规范 (Code Style)

- **命名**: 使用小写+下划线（ROS 2标准）
  - 节点名: `serial_sender`, `fruit_arm_driver`
  - 话题名: `control_data`, `feedback_data`
- **注释**: 中英文混合，接口定义倾向中文
- **类型提示**: Python代码中使用类型注解
- **日志级别**: 
  - `info`: 关键状态变化
  - `warn`: 潜在问题
  - `debug`: 详细调试信息（数据值等）
  - `error`: 错误和异常

