import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
# 导入Action和相关消息类型
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from robot_control_interfaces.msg import ArmControl
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
        self.control_data_publisher_ = self.create_publisher(ArmControl, 'arm_control_data', 10)

        #-----------------------读取串口舵机角度-------------------
        # 注意：下位机当前不提供舵机角度反馈，使用命令值作为状态反馈
        # 如果下位机未来提供反馈，取消下面的注释：
        # self.subscriber_ = self.create_subscription(SerialData, 'feedback_data', self.feedback_callback, 10)

        #-----------------------timer-------------------
        # 以20Hz频率发布joint_states，用于RViz显示（使用命令值作为反馈）
        self.timer = self.create_timer(0.05, self.timer_callback)


        self.latest_joint_feedback = [0.0, 0.0, 0.0, 0.0]  # 初始化为中位
        self.feedback_lock = threading.Lock()
        
        self.get_logger().warn('⚠️ 下位机未提供舵机反馈，使用命令值作为状态！请确保舵机能准确执行命令。')
        
        self.joint_names = ['j1', 'j2', 'j3', 'j4']
        
        # 立即发布初始状态，让MoveIt知道机器人的起始位置
        self.publish_joint_states(self.joint_names, self.latest_joint_feedback)
        self.get_logger().info(f'已发布初始关节状态: {self.latest_joint_feedback}') 
        
    
    # def feedback_callback(self, msg):
    #     """
    #     处理从串口接收到的舵机反馈数据
        
    #     Args:
    #         msg (SerialData): 包含舵机反馈值的消息
    #     """
    #     # 将接收到的舵机反馈值存储起来
    #     with self.feedback_lock:
    #         self.latest_joint_feedback = [
    #             msg.servo1,
    #             msg.servo2,
    #             msg.servo3,
    #             msg.servo4,
    #         ]
    #     self.get_logger().debug(f"收到舵机反馈: {self.latest_joint_feedback}")
    
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
        
        # 从目标中提取轨迹信息
        trajectory = goal_handle.request.trajectory
        
        # 记录开始执行前的状态，用于检测异常运动
        with self.feedback_lock:
            start_positions = self.latest_joint_feedback.copy()
        self.get_logger().info(f'Starting from positions: {start_positions}')
        
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

            #直接发布原始数据话题
            for i, joint_name in enumerate(trajectory.joint_names):
                servo_name = joint_map.get(joint_name)
                position = point.positions[i]
                
                servo_vals[servo_name] = position
                self.get_logger().info(f"Joint {joint_name} (Position: {position})")
 
                
            # --- 计算延时 ---
            # 计算当前点与上一个点之间的时间差，并延时
            current_point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            sleep_duration = current_point_time - last_point_time
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            last_point_time = current_point_time

            servo1_val = servo_vals.get('servo1', 0.0)  # 默认中位
            servo2_val = servo_vals.get('servo2', 0.0)
            servo3_val = servo_vals.get('servo3', 0.0)
            servo4_val = servo_vals.get('servo4', 0.0)
            
            # 安全检查：检测单步运动幅度是否过大
            with self.feedback_lock:
                prev_positions = self.latest_joint_feedback.copy()
            max_step = max(abs(servo1_val - prev_positions[0]),
                          abs(servo2_val - prev_positions[1]),
                          abs(servo3_val - prev_positions[2]),
                          abs(servo4_val - prev_positions[3]))
            if max_step > 900:  # 超过90度（900脉冲）视为异常
                self.get_logger().warn(f'⚠️ 检测到大幅度运动！最大步进: {max_step:.1f} 脉冲 ({max_step/10:.1f}度)')

            self.get_logger().info(f"Publishing to servos (pulse 0-3600): {servo1_val}, {servo2_val}, {servo3_val}, {servo4_val}")

            # 发布到串口舵机控制话题
            msg = ArmControl()
            msg.servo1 = servo1_val
            msg.servo2 = servo2_val
            msg.servo3 = servo3_val
            msg.servo4 = servo4_val

            self.control_data_publisher_.publish(msg)
            self.get_logger().info(f"Published to control_data: {msg}")
            
            # 更新关节状态反馈（使用命令值作为反馈）
            with self.feedback_lock:
                self.latest_joint_feedback = [servo1_val, servo2_val, servo3_val, servo4_val]
            
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