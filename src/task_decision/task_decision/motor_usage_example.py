"""
电机控制使用示例

演示如何在 TaskDecision 节点中使用 Motor 类进行各种运动控制。
"""

def motor_usage_examples(node):
    """
    展示 Motor 类的各种使用方法
    
    Args:
        node: TaskDecision 节点实例
    """
    
    # ========== 基础运动控制 ==========
    
    # 前进
    node.motor.move_forward(speed=0.5)  # 以 0.5 m/s 前进
    
    # 后退
    node.motor.move_backward(speed=0.3)  # 以 0.3 m/s 后退
    
    # 左转
    node.motor.turn_left(angular_speed=1.57)  # 以 1.57 rad/s 左转
    
    # 右转
    node.motor.turn_right(angular_speed=1.57)  # 以 1.57 rad/s 右转
    
    # 停止
    node.motor.stop()
    
    # ========== 复合运动 ==========
    
    # 前进并轻微右转（如走曲线）
    node.motor.move_forward_and_turn(
        linear_speed=0.5,     # 0.5 m/s 前进
        angular_speed=-0.1    # -0.1 rad/s 右转（负值表示右转）
    )
    
    # 直接控制所有参数（最底层方法）
    node.motor.move(
        linear_x=0.5,    # 线速度 0.5 m/s
        angular_z=0.2    # 角速度 0.2 rad/s
    )
    
    # ========== 紧急停止 ==========
    
    # 紧急停止（会输出警告）
    node.motor.emergency_stop()


def motor_navigation_example(node):
    """
    在导航场景中使用 Motor 类的示例
    
    Args:
        node: TaskDecision 节点实例
    """
    import time
    
    # 场景：让机器人前进 3 秒，然后停止
    node.get_logger().info("开始前进...")
    node.motor.move_forward(speed=0.5)
    time.sleep(3)
    
    node.get_logger().info("停止运动...")
    node.motor.stop()
    time.sleep(1)
    
    # 左转 90 度（假设以 1 rad/s 的速度转 π/2 弧度需要约 1.57 秒）
    node.get_logger().info("开始左转...")
    node.motor.turn_left(angular_speed=1.0)
    time.sleep(1.57)
    
    node.get_logger().info("停止旋转...")
    node.motor.stop()


def motor_fruit_picking_example(node):
    """
    在采摘场景中使用 Motor 类的示例
    
    Args:
        node: TaskDecision 节点实例
    """
    import time
    
    # 场景：机器人在树下采摘水果的运动控制
    
    # 1. 缓慢靠近水果树
    node.get_logger().info("缓慢靠近果树...")
    node.motor.move_forward(speed=0.2)  # 0.2 m/s 是缓速
    time.sleep(2)
    node.motor.stop()
    
    # 2. 精细调整位置（前进 + 轻微转向）
    node.get_logger().info("微调位置...")
    node.motor.move_forward_and_turn(
        linear_speed=0.1,     # 非常缓速
        angular_speed=0.05    # 轻微调整
    )
    time.sleep(1)
    node.motor.stop()
    
    # 3. 原地旋转以寻找最佳采摘角度
    node.get_logger().info("旋转寻找采摘角度...")
    node.motor.turn(angular_speed=0.5)
    time.sleep(3.14)  # 旋转半圈
    node.motor.stop()


# 在 TaskDecision 的主程序中如何使用
"""
在 main() 函数中使用示例：

def main(args=None):
    rclpy.init(args=args)
    node = TaskDecision()
    
    try:
        # 使用示例 1：基础运动控制
        motor_usage_examples(node)
        
        # 使用示例 2：导航场景
        motor_navigation_example(node)
        
        # 使用示例 3：采摘场景
        # motor_fruit_picking_example(node)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.motor.stop()  # 确保电机停止
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
