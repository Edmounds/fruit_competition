# MoveIt命名位姿控制服务使用说明

## 功能概述

该功能提供了一个ROS 2 Service接口，允许通过位姿名称来控制MoveIt机械臂移动到预定义的命名位姿。

### 可用位姿
根据MoveIt配置，当前支持以下命名位姿：
- `home` - 初始位置
- `harvest` - 采摘位置
- `scan` - 扫描位置
- `detect_left` - 左侧检测位置
- `detect_right` - 右侧检测位置
- `grab_left` - 左侧抓取位置
- `grab_right` - 右侧抓取位置

## 编译

```bash
cd ~/fruit_ws
colcon build --packages-select robot_control_interfaces moveit_controller
source install/setup.bash
```

## 使用方法

### 1. 启动MoveIt和RViz（在第一个终端）

```bash
source install/setup.bash
ros2 launch arm_config fruit_arm.launch.py
```

### 2. 启动命名位姿控制服务（在第二个终端）

```bash
source install/setup.bash
ros2 run moveit_controller moveit_pose_controller
```

或使用launch文件：

```bash
ros2 launch moveit_controller moveit_pose_controller.launch.py
```

### 3. 使用Python客户端调用服务

#### 方式A：使用交互式客户端（推荐）

```bash
source install/setup.bash
ros2 run moveit_controller interactive_move_to_pose_client.py
```

然后根据提示输入位姿编号或名称：
```
请选择位姿 > 1        # 移动到home
请选择位姿 > harvest  # 移动到harvest
请选择位姿 > list     # 显示所有可用位姿
请选择位姿 > q        # 退出
```

#### 方式B：使用简单客户端

```bash
source install/setup.bash
ros2 run moveit_controller move_to_pose_client.py
```

#### 方式C：使用命令行直接调用

```bash
source install/setup.bash
ros2 service call /move_to_pose robot_control_interfaces/srv/MoveToPose "{pose_name: 'home'}"
```

## Python代码示例

### 简单调用示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import MoveToPose

def main():
    rclpy.init()
    node = Node('simple_pose_client')
    
    # 创建客户端
    client = node.create_client(MoveToPose, '/move_to_pose')
    client.wait_for_service()
    
    # 创建请求
    request = MoveToPose.Request()
    request.pose_name = 'home'  # 修改为你想要的位姿名称
    
    # 发送请求
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    # 获取响应
    response = future.result()
    if response.success:
        print(f"成功: {response.message}")
    else:
        print(f"失败: {response.message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 封装类示例

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import MoveToPose

class ArmController:
    """机械臂控制器封装类"""
    
    def __init__(self, node: Node):
        self.node = node
        self.client = node.create_client(MoveToPose, '/move_to_pose')
        self.client.wait_for_service(timeout_sec=5.0)
    
    def move_to_pose(self, pose_name: str, timeout: float = 30.0) -> bool:
        """
        移动到指定命名位姿
        
        Args:
            pose_name: 位姿名称
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否成功
        """
        request = MoveToPose.Request()
        request.pose_name = pose_name
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        
        if future.result() is not None:
            response = future.result()
            self.node.get_logger().info(response.message)
            return response.success
        else:
            self.node.get_logger().error('服务调用失败')
            return False

# 使用示例
def main():
    rclpy.init()
    node = Node('my_robot_controller')
    
    # 创建控制器
    arm = ArmController(node)
    
    # 执行一系列动作
    poses = ['home', 'scan', 'detect_left', 'grab_left', 'home']
    
    for pose in poses:
        print(f"移动到: {pose}")
        if arm.move_to_pose(pose):
            print(f"✓ 成功到达 {pose}")
        else:
            print(f"✗ 移动到 {pose} 失败")
            break
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service接口定义

**Service名称**: `/move_to_pose`

**Service类型**: `robot_control_interfaces/srv/MoveToPose`

### 请求（Request）
```
string pose_name  # 位姿名称
```

### 响应（Response）
```
bool success      # 是否执行成功
string message    # 执行结果消息
```

## 测试检查

### 查看服务是否运行
```bash
ros2 service list | grep move_to_pose
```

### 查看服务类型
```bash
ros2 service type /move_to_pose
```

### 查看服务接口定义
```bash
ros2 interface show robot_control_interfaces/srv/MoveToPose
```

## 故障排查

### 问题1: 服务未找到
**现象**: `Service /move_to_pose not available`

**解决方案**:
1. 确认服务节点已启动：`ros2 node list | grep moveit_pose_controller`
2. 检查服务列表：`ros2 service list`
3. 重新启动服务节点

### 问题2: 位姿不存在
**现象**: `位姿 'xxx' 不存在`

**解决方案**:
1. 检查MoveIt配置中定义的位姿名称
2. 在RViz中查看可用的命名位姿
3. 启动服务节点时会打印所有可用位姿列表

### 问题3: 规划失败
**现象**: `规划失败`

**解决方案**:
1. 检查机械臂当前位置是否在奇异点附近
2. 尝试先移动到home位姿再移动到目标位姿
3. 检查MoveIt配置和关节限位设置

## 注意事项

1. **安全第一**: 首次使用时，确保机械臂周围没有障碍物
2. **顺序执行**: Service调用是阻塞的，需要等待上一个动作完成
3. **错误处理**: 建议在Python代码中添加异常处理和超时机制
4. **MoveIt必须运行**: 确保MoveIt节点和RViz已正常启动

## 扩展开发

如需添加新的位姿：
1. 在RViz的MoveIt MotionPlanning插件中调整机械臂到目标姿态
2. 在Poses标签页中点击"Save"保存新位姿
3. 位姿会保存在`arm_config/config/moveit.rviz`配置文件中
4. 或者使用MoveIt Setup Assistant重新配置

## 相关文件

- Service定义: `src/robot_control_interfaces/srv/MoveToPose.srv`
- C++服务端: `src/moveit_controller/src/moveit_pose_controller.cpp`
- Python客户端: `src/moveit_controller/src/move_to_pose_client.py`
- 交互式客户端: `src/moveit_controller/src/interactive_move_to_pose_client.py`
- Launch文件: `src/moveit_controller/launch/moveit_pose_controller.launch.py`
