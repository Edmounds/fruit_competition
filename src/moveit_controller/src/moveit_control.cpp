#include "moveit_control.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

/**
 * @brief 构造函数 - 初始化MoveIt控制器
 *
 * @param node ROS 2 节点共享指针
 */
MoveItController::MoveItController(rclcpp::Node::SharedPtr node)
    : node_(node),
      tol_pos_(0.05), tol_ori_(0.05)
{
    // 声明参数
    declareParameters();

    // 加载参数
    loadParameters();

    // 初始化 MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "manipulator");

    // 获取并打印规划组信息
    std::string planning_frame = move_group_->getPlanningFrame();
    RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", planning_frame.c_str());

    std::string end_effector_link = move_group_->getEndEffectorLink();
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", end_effector_link.c_str());

    // 创建服务服务器
    service_ = node_->create_service<robot_control_interfaces::srv::MoveToPosition>(
        "/move_to_position",
        std::bind(&MoveItController::handleMoveToPosition, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "MoveIt控制服务已启动，等待服务请求...");
    RCLCPP_INFO(node_->get_logger(), "服务名称: /move_to_position");
}

/**
 * @brief 服务回调函数
 *
 * @param request 服务请求（包含目标坐标）
 * @param response 服务响应（包含执行结果）
 */
void MoveItController::handleMoveToPosition(
    const std::shared_ptr<robot_control_interfaces::srv::MoveToPosition::Request> request,
    std::shared_ptr<robot_control_interfaces::srv::MoveToPosition::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "收到移动请求: [%.3f, %.3f, %.3f]",
                request->x, request->y, request->z);

    // 执行规划和运动
    bool success = planAndExecute(request->x, request->y, request->z);

    // 设置响应
    response->success = success;
    if (success)
    {
        response->message = "机械臂成功移动到目标位置";
        RCLCPP_INFO(node_->get_logger(), "执行成功");
    }
    else
    {
        response->message = "机械臂移动失败，规划或执行出错";
        RCLCPP_ERROR(node_->get_logger(), "执行失败");
    }
}

/**
 * @brief 声明ROS参数
 */
void MoveItController::declareParameters()
{
    node_->declare_parameter("Tolerances.position", 0.05);
    node_->declare_parameter("Tolerances.orientation", 0.05);
}

/**
 * @brief 从参数服务器加载参数
 */
void MoveItController::loadParameters()
{
    tol_pos_ = node_->get_parameter("Tolerances.position").as_double();
    tol_ori_ = node_->get_parameter("Tolerances.orientation").as_double();

    RCLCPP_INFO(node_->get_logger(), "容差参数加载完成:");
    RCLCPP_INFO(node_->get_logger(), "  位置容差: %.3f", tol_pos_);
    RCLCPP_INFO(node_->get_logger(), "  姿态容差: %.3f", tol_ori_);
}

/**
 * @brief 将度数转换为弧度
 */
double MoveItController::degreesToRadians(double degrees) const
{
    return degrees * M_PI / 180.0;
}

/**
 * @brief 将弧度转换为度数
 */
double MoveItController::radiansToDegrees(double radians) const
{
    return radians * 180.0 / M_PI;
}

/**
 * @brief 执行运动规划和运动到指定位置
 *
 * @param x 目标X坐标
 * @param y 目标Y坐标
 * @param z 目标Z坐标
 * @return true 规划和执行成功
 * @return false 规划或执行失败
 */
bool MoveItController::planAndExecute(double x, double y, double z)
{
    // 设置目标位置
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    // 设置容差
    move_group_->setGoalPositionTolerance(tol_pos_);
    move_group_->setGoalOrientationTolerance(tol_ori_);

    // 设置位置目标
    move_group_->setPositionTarget(x, y, z);

    RCLCPP_WARN(node_->get_logger(), "开始规划到位置: [%.3f, %.3f, %.3f]", x, y, z);

    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "规划 %s", success ? "成功" : "失败");

    // 执行运动规划
    if (success)
    {
        move_group_->execute(my_plan);
        RCLCPP_INFO(node_->get_logger(), "执行完成");
        return true;
    }

    return false;
}

/**
 * @brief 主函数
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_control");

    try
    {
        // 创建MoveIt控制器对象
        MoveItController controller(node);

        // 持续运行，等待服务请求
        RCLCPP_INFO(node->get_logger(), "节点运行中，按Ctrl+C退出");
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "捕获异常: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}