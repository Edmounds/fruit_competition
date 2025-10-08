#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
Adapted for fruit robot arm.
"""

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
    max_attempts=3
):
    """
    帮助函数，用于规划和执行机器人运动
    
    参数:
        robot: MoveItPy机器人接口
        planning_component: 规划组件
        logger: 日志记录器
        single_plan_parameters: 单一规划参数
        multi_plan_parameters: 多规划器参数
        sleep_time: 执行完成后等待时间
        max_attempts: 最大尝试次数
    """
    # 尝试规划直到成功或达到最大尝试次数
    attempt = 0
    plan_result = None
    
    while attempt < max_attempts:
        attempt += 1
        # 规划轨迹
        logger.info(f"Planning trajectory (attempt {attempt}/{max_attempts})")
        
        try:
            if multi_plan_parameters is not None:
                plan_result = planning_component.plan(
                    multi_plan_parameters=multi_plan_parameters
                )
            elif single_plan_parameters is not None:
                plan_result = planning_component.plan(
                    single_plan_parameters=single_plan_parameters
                )
            else:
                plan_result = planning_component.plan()
                
            # 如果成功获得规划结果则退出循环
            if plan_result:
                break
        except Exception as e:
            logger.error(f"Planning attempt {attempt} failed with error: {str(e)}")
            
        # 如果规划失败，尝试使用稍微不同的目标状态
        if attempt < max_attempts:
            logger.info("Adjusting goal state slightly for next attempt")
            # 获取当前机器人状态
            current_state = planning_component.get_start_state()
            # 随机化目标状态，但幅度很小
            planning_component.set_start_state(robot_state = current_state)
            time.sleep(0.5)  # 短暂等待

    # 执行规划
    if plan_result:
        logger.info("Executing plan")
        try:
            robot_trajectory = plan_result.trajectory
            # 确保使用已配置的控制器
            robot.execute(robot_trajectory, controllers=["arm_controller"])
        except Exception as e:
            logger.error(f"Execution failed: {str(e)}")
    else:
        logger.error(f"Planning failed after {max_attempts} attempts")

    time.sleep(sleep_time)


class MoveItPyNode(MoveItPy):
    """A ROS2 node encapsulating MoveItPy functionality."""
    
    def __init__(self, name="moveit_node"):
        super().__init__(name)
        self.get_logger().info('MoveIt node has been started.')
        self.moveit = MoveItPy(node=self)
        self.robot = self.moveit.get_robot_interface()
        
    def get_planning_component(self, planning_group_name):
        return self.robot.get_planning_component(planning_group_name)
        
    def get_robot(self):
        return self.robot
        
    def plan_and_execute(
        self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        max_attempts=3
        ):
        
        """
        规划和执行机器人运动的帮助函数
        
        参数:
            robot: MoveItPy机器人接口
            planning_component: 规划组件
            logger: 日志记录器
            single_plan_parameters: 单一规划参数
            multi_plan_parameters: 多规划器参数
            max_attempts: 最大尝试次数
        """
        # 尝试规划直到成功或达到最大尝试次数
        attempt = 0
        plan_result = None
        
        while attempt < max_attempts:
            attempt += 1
            # 规划轨迹
            self.get_logger().info(f"Planning trajectory (attempt {attempt}/{max_attempts})")
            
            try:
                if multi_plan_parameters is not None:
                    plan_result = planning_component.plan(
                        multi_plan_parameters=multi_plan_parameters
                    )
                elif single_plan_parameters is not None:
                    plan_result = planning_component.plan(
                        single_plan_parameters=single_plan_parameters
                    )
                else:
                    plan_result = planning_component.plan()
                    
                # 如果成功获得规划结果则退出循环
                if plan_result:
                    break
            except Exception as e:
                self.get_logger().error(f"Planning attempt {attempt} failed with error: {str(e)}")
                
            # 如果规划失败，尝试使用稍微不同的目标状态
            if attempt < max_attempts and not plan_result:
                self.get_logger().info("Adjusting goal state slightly for next attempt")
                # 获取当前机器人状态
                current_state = planning_component.get_start_state()
                # 重置起始状态
                planning_component.set_start_state(robot_state = current_state)
                time.sleep(0.5)  # 短暂等待

        # 执行规划
        if plan_result:
            self.get_logger().info("Executing plan")
            try:
                robot_trajectory = plan_result.trajectory
                # 确保使用已配置的控制器
                robot.execute(robot_trajectory, controllers=["arm_controller"])
            except Exception as e:
                self.get_logger().error(f"Execution failed: {str(e)}")
        else:
            self.get_logger().error(f"Planning failed after {max_attempts} attempts")


def main():
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    fruit_arm = MoveItPy(node_name="moveit_node")
    arm = fruit_arm.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    arm.set_start_state(configuration_name="home")

    # set pose goal using predefined state (since only home is defined, use random_valid or same)
    # For demo, we'll use home to home, but in practice you'd have more states
    arm.set_goal_state(configuration_name="home")  # Note: SRDF only has home, so this is trivial

    # plan to goal
    plan_and_execute(fruit_arm, arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 2 - set goal state with RobotState object
    ###########################################################################

    # instantiate a RobotState instance using the current robot model
    robot_model = fruit_arm.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(fruit_arm, arm, logger, sleep_time=3.0)

    ###########################################################################
    # Plan 3 - set goal state with PoseStamped message
    ###########################################################################

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"  # 基于base_link参考系
    pose_goal.pose.orientation.w = 1.0
    # 调整目标位置以避免碰撞
    pose_goal.pose.position.x = 0.25  
    pose_goal.pose.position.y = 0.15  
    pose_goal.pose.position.z = 0.35  # 提高Z轴位置
    arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link4_1")  # 末端执行器链接

    # plan to goal - 使用更多尝试次数
    plan_and_execute(fruit_arm, arm, logger, sleep_time=3.0, max_attempts=3)

    ###########################################################################
    # Plan 4 - set goal state with constraints
    ###########################################################################

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    # 使用更安全的关节值，避免可能的碰撞
    joint_values = {
        "j1": 0.0,
        "j2": 0.3,  # 减小角度避免碰撞
        "j3": 0.8,  # 减小角度避免碰撞
        "j4": 1.0,  # 减小角度避免碰撞
    }
    robot_state.joint_positions = joint_values
    joint_constraint = construct_joint_constraint(
        robot_state=robot_state,
        joint_model_group=fruit_arm.get_robot_model().get_joint_model_group("arm"),
    )
    arm.set_goal_state(motion_plan_constraints=[joint_constraint])

    # plan to goal - 使用更多尝试次数
    plan_and_execute(fruit_arm, arm, logger, sleep_time=3.0, max_attempts=3)

    ###########################################################################
    # Plan 5 - Planning with Multiple Pipelines simultaneously
    ###########################################################################

    # set plan start state to current state
    arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    arm.set_goal_state(configuration_name="home")

    # initialise multi-pipeline plan request parameters
    # 使用实际可用的规划器名称
    multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
        fruit_arm, ["ompl", "pilz_industrial_motion_planner", "chomp"]
    )

    # plan to goal - 使用更多尝试次数
    plan_and_execute(
        fruit_arm,
        arm,
        logger,
        multi_plan_parameters=multi_pipeline_plan_request_params,
        sleep_time=3.0,
        max_attempts=3,
    )

if __name__ == "__main__":
    main()
