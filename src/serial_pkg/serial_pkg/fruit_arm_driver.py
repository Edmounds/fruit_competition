import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
# 导入Action和相关消息类型
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from robot_control_interfaces.msg import SerialData
from sensor_msgs.msg import JointState 
import time
import math
import threading

class FruitArmDriver(Node):
    def __init__(self):
        super().__init__('fruit_arm_driver')

        # ------------------- Action Server 初始化 -------------------
        action_name = 'manipulator_controller/follow_joint_trajectory' 
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info(f'Action server "{action_name}" is ready.')
        

        # ------------------- Joint State Publisher -------------------
        self.joint_state_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        #-----------------------Topic发布器-----------------------
        self.publisher_ = self.create_publisher(SerialData, 'control_data', 10)

        #-----------------------读取串口舵机角度-------------------
        # self.subscriber_ = self.create_subscription(SerialData, 'feedback_data', self.feedback_callback, 10)

        #-----------------------timer-------------------
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 存储最新的关节反馈值
        self.latest_joint_feedback = [0.0, 0.0, 0.0, 0.0]
        self.feedback_lock = threading.Lock()
        
        self.joint_names = ['j1', 'j2', 'j3', 'j4'] 
        
    
    def feedback_callback(self, msg):
        """
        处理从串口接收到的舵机反馈数据
        
        Args:
            msg (SerialData): 包含舵机反馈值的消息
        """
        # 将接收到的舵机反馈值存储起来
        with self.feedback_lock:
            self.latest_joint_feedback = [
                msg.servo1,
                msg.servo2,
                msg.servo3,
                msg.servo4,
            ]
        self.get_logger().debug(f"收到舵机反馈: {self.latest_joint_feedback}")
    
    def timer_callback(self):
        # 定时发布当前关节状态
        with self.feedback_lock:
            joints_state = self.latest_joint_feedback.copy()  # 使用最新接收到的关节状态

        self.publish_joint_states(self.joint_names, joints_state)

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
        
        # # 获取参数值
        # publish_joints = self.get_parameter('publish_joints').get_parameter_value().bool_value
        
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

            # 将弧度转换为舵机脉冲值
            # MoveIt传来的positions顺序与joint_names一一对应
            # 我们需要根据joint_names找到对应的舵机并转换
            
            servo_vals = {}
            joint_map = {
                'j1': 'servo1',
                'j2': 'servo2',
                'j3': 'servo3',
                'j4': 'servo4',
            }

            for i, joint_name in enumerate(trajectory.joint_names):
                servo_name = joint_map.get(joint_name)
                position = point.positions[i]
                if servo_name == 'servo1': # 270度
                    servo_vals[servo_name] = self.radians_to_servo_pulse(position, 270)
                elif servo_name in ['servo2', 'servo3', 'servo4']: # 240度
                    servo_vals[servo_name] = self.radians_to_servo_pulse(position, 240)

                servo_vals["servo5"] = 1

            # --- 计算延时 ---
            # 计算当前点与上一个点之间的时间差，并延时
            current_point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_duration = current_point_time - last_point_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            last_point_time = current_point_time

            # 使用get获取值，如果轨迹中没有某个舵机，则保持其当前值（或设为默认值）
            # 注意：这里简化为0，实际应用可能需要从一个状态变量中获取
            servo1_val = servo_vals.get('servo1', 0)
            servo2_val = servo_vals.get('servo2', 0)
            servo3_val = servo_vals.get('servo3', 0)
            servo4_val = servo_vals.get('servo4', 0)
            servo5_val = servo_vals.get('servo5', 0)

            self.get_logger().warn(f"Publishing to servos: {servo1_val}, {servo2_val}, {servo3_val}, {servo4_val}, {servo5_val}")

            # 发送串口数据
            msg = SerialData()
            msg.linear_x = 0.0
            msg.angular_z = 0.0

            msg.servo1 = servo1_val
            msg.servo2 = servo2_val
            msg.servo3 = servo3_val
            msg.servo4 = servo4_val
            msg.servo5 = servo5_val

            
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published SerialData: {msg}")
            
            # # (可选) 发布反馈信息
            # feedback_msg = FollowJointTrajectory.Feedback()
            # feedback_msg.joint_names = trajectory.joint_names
            # feedback_msg.actual = point # 简化处理，实际应发回真实编码器值
            # goal_handle.publish_feedback(feedback_msg)


        # 所有点都执行完毕
        goal_handle.succeed()
        self.get_logger().info('Goal execution succeeded!')
        
        # 返回结果
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


    def publish_joint_states(self, joint_names, positions):
       """
       发布关节状态消息，用于在 RViz 中更新机器人模型。
       """
       msg = JointState()
       msg.header.stamp = self.get_clock().now().to_msg()
       msg.name = joint_names
       msg.position = list(positions) # 确保是list of float
        
       self.joint_state_publisher_.publish(msg)
       self.get_logger().debug(f"Published fake joint states for RViz: {positions}")
    
    # --- 辅助函数 ---
    
    def servo_pulse_to_radians(self, pulse, total_degrees):
        """
        将舵机脉冲值 (0-1000) 转换为弧度值。
        
        Args:
            pulse (int): 舵机脉冲值, 范围 0-1000.
            total_degrees (int): 舵机总行程角度 (例如 180, 270).
        
        Returns:
            float: 对应的弧度值.
        """
        # 将脉冲值从 [0, 1000] 映射到 [0, total_degrees]
        degrees = (pulse / 1000.0) * total_degrees
        
        # 将角度从行程中心（total_degrees / 2）平移到0点
        centered_degrees = degrees - (total_degrees / 2.0)
        
        # 将角度转换为弧度
        return math.radians(centered_degrees)

    def radians_to_servo_pulse(self, rad, total_degrees):
        """
        将弧度值转换为舵机脉冲值 (0-1000)。
        
        Args:
            rad (float): 弧度值.
            total_degrees (int): 舵机总行程角度 (例如 180, 270).
            
        Returns:
            int: 对应的舵机脉冲值 (0-1000).
        """
        # 将弧度转换为角度
        degrees = math.degrees(rad)
        
        # 计算总弧度范围的一半
        half_range_rad = math.radians(total_degrees / 2.0)
        
        # 限制输入弧度在舵机范围内
        rad = max(-half_range_rad, min(half_range_rad, rad))
        
        # 将弧度值从 [-half_range_rad, half_range_rad] 映射到 [0, 1000]
        # 首先，将值平移到 [0, 2 * half_range_rad]
        # 然后，归一化到 [0, 1]
        # 最后，缩放到 [0, 1000]
        pulse = (rad + half_range_rad) / (2 * half_range_rad) * 1000.0
        
        # 限制范围并转换为整数
        return int(max(0, min(1000, pulse)))

def main(args=None):
    rclpy.init(args=args)
    node = FruitArmDriver()

    # 使用多线程执行器，防止Action回调阻塞其他任务
    executor = MultiThreadedExecutor()
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