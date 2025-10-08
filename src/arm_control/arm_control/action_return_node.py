import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# 导入Action和相关消息类型
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_control_interfaces.msg import SerialData


class SerialActionServer(Node):
    def __init__(self):
        super().__init__('serial_action_server')

        # ------------------- Action Server 初始化 -------------------
        # 重要：这里的Action名称必须和你的 ros2_control 控制器配置中的名称完全一致！
        # 通常是 <controller_name>/follow_joint_trajectory
        action_name = 'joint_trajectory_controller/follow_joint_trajectory'
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            moveit_return_data_action,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info(f'Action server "{action_name}" is ready.')
        
        #-----------------------Topic发布器-----------------------
        
        self.publisher_ = self.create_publisher(SerialData, 'serial_data', 10)

    # --- Action Server 回调函数 ---

    def goal_callback(self, goal_request):
        """接收到新的目标请求时的回调。"""
        self.get_logger().info('Received new trajectory goal request.')
        # 可以在这里添加逻辑来验证目标是否有效
        # 例如，检查 joint_names 是否与你的机器人匹配
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """当目标被接受后的回调，在这里启动执行。"""
        # 在一个新的线程中执行，以避免阻塞节点主循环
        self.get_logger().info('Goal accepted, starting execution...')
        goal_handle.execute()
        
    def cancel_callback(self, goal_handle):
        """当客户端请求取消目标时的回调。"""
        self.get_logger().info('Received cancel request for the goal.')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """核心执行函数：在这里处理轨迹并发送串口数据。"""
        self.get_logger().info('Executing goal...')
        
        # 从目标中提取轨迹信息
        trajectory = goal_handle.request.trajectory
        
        # 记录上一个点的时间，用于计算延时
        last_point_time = 0.0

        # 遍历轨迹中的每一个路径点
        for point in trajectory.points:
            # 检查是否有取消请求
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled!')
                return FollowJointTrajectory.Result()

            # --- 数据转换和映射 ---
            # point.positions 是一个包含所有关节角度的列表（单位：弧度）
            # 你需要将它映射到你的舵机上
            
            # !! 关键 !!: 这里的索引顺序必须与 MoveIt2/URDF 中定义的关节顺序一致
            # 假设顺序是: [servo1, servo2, servo3_gripper, servo4, servo5]
            # 假设 servo3 是夹爪，只有0/1状态
            
            # 将弧度转换为你的舵机脉冲值 (0-1000)
            servo1_val = self.radians_to_servo_pulse(point.positions[0])
            servo2_val = self.radians_to_servo_pulse(point.positions[1])
            # 对于夹爪，可以根据开合角度设定一个阈值
            servo3_val = 1 if point.positions[2] > 0.5 else 0 # 假设大于0.5弧度为闭合
            servo4_val = self.radians_to_servo_pulse(point.positions[3])
            servo5_val = self.radians_to_servo_pulse(point.positions[4])
            
            # --- 计算延时 ---
            # 计算当前点与上一个点之间的时间差，并延时
            current_point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_duration = current_point_time - last_point_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            last_point_time = current_point_time
            
      

            # (可选) 发布反馈信息
            feedback_msg = FollowJointTrajectory.Feedback()
            feedback_msg.joint_names = trajectory.joint_names
            feedback_msg.actual = point # 简化处理，实际应发回真实编码器值
            goal_handle.publish_feedback(feedback_msg)


        # 所有点都执行完毕
        goal_handle.succeed()
        self.get_logger().info('Goal execution succeeded!')
        
        # 返回结果
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESSFUL
        return result

    # --- 辅助函数 ---
    
    def radians_to_servo_pulse(self, rad):
        """
        !! 需要你根据实际情况标定 !!
        将弧度值转换为舵机脉冲值 (0-1000)。
        这是一个示例转换，你需要根据你的舵机进行精确标定。
        假设舵机中位(500)对应0弧度, 范围是 -pi/2 到 +pi/2
        """
        # 映射: [-pi/2, pi/2] -> [0, 1000]
        # pi/2 ≈ 1.57
        pulse = 500 + (rad / (math.pi / 2.0)) * 500
        
        # 限制范围
        return int(max(0, min(1000, pulse)))




def main(args=None):
    rclpy.init(args=args)
    node = SerialActionServer()
    
    # 使用多线程执行器，防止Action回调阻塞其他任务
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()