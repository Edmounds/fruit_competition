# MoveIt 位置控制服务

## 功能说明

本包已改造为**ROS 2 服务模式**，提供 `/move_to_position` 服务接口，接收三维坐标输入，控制机械臂移动到指定位置。

## 服务接口

### 服务名称
`/move_to_position`

### 服务类型
`robot_control_interfaces/srv/MoveToPosition`

### 请求 (Request)
```
float64 x  # X坐标 (米)
float64 y  # Y坐标 (米)
float64 z  # Z坐标 (米)
```

### 响应 (Response)
```
bool success      # 是否成功
string message    # 结果信息
```

## 使用方法

### 1. 启动服务节点

```bash
# 确保已source工作空间
source install/setup.bash

# 启动MoveIt服务节点
ros2 run moveit_controller moveit_control
```

### 2. 调用服务

#### 方法A: 使用命令行

```bash
# 移动到指定位置 [x, y, z]
ros2 service call /move_to_position robot_control_interfaces/srv/MoveToPosition "{x: 0.3, y: 0.0, z: 0.4}"
```

#### 方法B: 使用Python客户端

```bash
# 运行测试脚本
ros2 run moveit_controller test_move_service.py
```

或者在你的Python代码中：

```python
import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import MoveToPosition

class MyClient(Node):
    def __init__(self):
        super().__init__('my_client')
        self.client = self.create_client(MoveToPosition, '/move_to_position')
        
    def move_to(self, x, y, z):
        request = MoveToPosition.Request()
        request.x = x
        request.y = y
        request.z = z
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

# 使用示例
rclpy.init()
client = MyClient()
response = client.move_to(0.3, 0.0, 0.4)

if response.success:
    print(f"成功: {response.message}")
else:
    print(f"失败: {response.message}")
```

#### 方法C: 使用C++客户端

```cpp
#include <rclcpp/rclcpp.hpp>
#include <robot_control_interfaces/srv/move_to_position.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_client");
    
    auto client = node->create_client<robot_control_interfaces::srv::MoveToPosition>(
        "/move_to_position");
    
    auto request = std::make_shared<robot_control_interfaces::srv::MoveToPosition::Request>();
    request->x = 0.3;
    request->y = 0.0;
    request->z = 0.4;
    
    auto future = client->async_send_request(request);
    
    if (rclcpp::spin_until_future_complete(node, future) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(node->get_logger(), "成功: %s", response->message.c_str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "失败: %s", response->message.c_str());
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
```

## 配置参数

在 `config/moveit_controller.yaml` 中可配置容差参数：

```yaml
moveit_control:
  ros__parameters:
    Tolerances:
      position: 0.05      # 位置容差 (米)
      orientation: 0.05   # 姿态容差 (弧度)
```

## 与原版本的区别

### 原版本 (参数模式)
- ❌ 从参数服务器读取目标位置
- ❌ 执行一次后退出
- ❌ 需要重新启动节点才能移动到新位置

### 新版本 (服务模式)
- ✅ 通过服务接口接收目标位置
- ✅ 持续运行，可响应多次请求
- ✅ 支持动态调用，无需重启节点
- ✅ 返回执行结果状态

## 完整启动示例

```bash
# 终端1: 启动MoveIt + RViz
ros2 launch arm_config fruit_arm.launch.py

# 终端2: 启动MoveIt控制服务
ros2 run moveit_controller moveit_control

# 终端3: 调用服务移动机械臂
ros2 service call /move_to_position robot_control_interfaces/srv/MoveToPosition "{x: 0.3, y: 0.0, z: 0.4}"

# 或使用测试脚本
ros2 run moveit_controller test_move_service.py
```

## 查看服务信息

```bash
# 列出所有服务
ros2 service list

# 查看服务类型
ros2 service type /move_to_position

# 查看服务接口定义
ros2 interface show robot_control_interfaces/srv/MoveToPosition
```

## 故障排除

1. **服务未找到**: 确保 `moveit_control` 节点正在运行
2. **规划失败**: 检查目标位置是否在机械臂工作空间内
3. **接口未找到**: 重新编译 `robot_control_interfaces` 包并 source 工作空间
