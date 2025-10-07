import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from robot_control_interfaces.action import MoveRobot
from example_action_rclpy.robot import Robot

class ActionRobotServer(Node):
    """机器人端Action服务"""

    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")

        self.robot_ = Robot()

        self.action_server_ = ActionServer(
            self, MoveRobot, 'move_robot', self.execute_callback
            # ,callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """执行回调函数,若采用默认handle_goal函数则会自动调用"""
        self.get_logger().info('执行移动机器人')
        feedback_msg = MoveRobot.Feedback()
        self.robot_.set_goal(goal_handle.request.distance)

        # rate = self.create_rate(2)
        while rclpy.ok() and not self.robot_.close_goal():
            # move
            self.robot_.move_step()
            # feedback
            feedback_msg.pose = self.robot_.get_current_pose()
            feedback_msg.status = self.robot_.get_status()
            goal_handle.publish_feedback(feedback_msg)
            # cancel check
            if goal_handle.is_cancel_requested:
                result = MoveRobot.Result()
                result.pose = self.robot_.get_current_pose()
                return result
            # rate.sleep() # Rate会造成死锁，单线程执行器时不能使用
            time.sleep(0.5)

        goal_handle.succeed()
        result = MoveRobot.Result()
        result.pose = self.robot_.get_current_pose()
        return result

def main(arg=None):
        
    rclpy.init(args=arg)
    node = ActionRobotServer('action_robot_server')
    rclpy.spin(node)
    node.get_logger().info('Action Robot Server is shutting down.')
    rclpy.shutdown()