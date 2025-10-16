#include "moveit_controller/moveit_pose_controller.hpp"

/**
 * @brief 构造函数 - 初始化MoveIt命名位姿控制器
 *
 * @param node ROS 2 节点共享指针
 */
MoveItPoseController::MoveItPoseController(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    // 初始化 MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, "manipulator");

    // 获取并打印规划组信息
    std::string planning_frame = move_group_->getPlanningFrame();
    RCLCPP_INFO(node_->get_logger(), "规划参考坐标系: %s", planning_frame.c_str());

    std::string end_effector_link = move_group_->getEndEffectorLink();
    RCLCPP_INFO(node_->get_logger(), "末端执行器链接: %s", end_effector_link.c_str());

    // 获取并打印所有命名位姿
    std::vector<std::string> named_targets = move_group_->getNamedTargets();
    RCLCPP_INFO(node_->get_logger(), "可用的命名位姿:");
    for (const auto &target : named_targets)
    {
        RCLCPP_INFO(node_->get_logger(), "  - %s", target.c_str());
    }

    // 创建服务服务器
    service_ = node_->create_service<robot_control_interfaces::srv::MoveToPose>(
        "/move_to_pose",
        std::bind(&MoveItPoseController::handleMoveToPose, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "MoveIt命名位姿控制服务已启动");
    RCLCPP_INFO(node_->get_logger(), "服务名称: /move_to_pose");
}

/**
 * @brief 服务回调函数 - 处理移动到命名位姿的请求
 *
 * @param request 服务请求，包含位姿名称
 * @param response 服务响应，包含执行结果
 */
void MoveItPoseController::handleMoveToPose(
    const std::shared_ptr<robot_control_interfaces::srv::MoveToPose::Request> request,
    std::shared_ptr<robot_control_interfaces::srv::MoveToPose::Response> response)
{
    RCLCPP_INFO(node_->get_logger(), "收到移动请求，目标位姿: '%s'",
                request->pose_name.c_str());

    // 检查位姿名称是否为空
    if (request->pose_name.empty())
    {
        response->success = false;
        response->message = "位姿名称不能为空";
        RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
        return;
    }

    // 检查位姿是否存在
    std::vector<std::string> named_targets = move_group_->getNamedTargets();
    bool pose_exists = std::find(named_targets.begin(), named_targets.end(),
                                 request->pose_name) != named_targets.end();

    if (!pose_exists)
    {
        response->success = false;
        response->message = "位姿 '" + request->pose_name + "' 不存在。可用位姿: ";
        for (size_t i = 0; i < named_targets.size(); ++i)
        {
            response->message += named_targets[i];
            if (i < named_targets.size() - 1)
            {
                response->message += ", ";
            }
        }
        RCLCPP_WARN(node_->get_logger(), "%s", response->message.c_str());
        return;
    }

    // 执行移动
    bool success = moveToNamedTarget(request->pose_name);

    // 设置响应
    response->success = success;
    if (success)
    {
        response->message = "机械臂成功移动到位姿 '" + request->pose_name + "'";
        RCLCPP_INFO(node_->get_logger(), "执行成功");
    }
    else
    {
        response->message = "机械臂移动到位姿 '" + request->pose_name + "' 失败";
        RCLCPP_ERROR(node_->get_logger(), "执行失败");
    }
}

/**
 * @brief 执行移动到命名位姿
 *
 * @param pose_name 位姿名称
 * @return true 执行成功
 * @return false 执行失败
 */
bool MoveItPoseController::moveToNamedTarget(const std::string &pose_name)
{
    RCLCPP_INFO(node_->get_logger(), "开始规划到位姿: '%s'", pose_name.c_str());

    // 设置命名目标
    move_group_->setNamedTarget(pose_name);

    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool plan_success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_success)
    {
        RCLCPP_ERROR(node_->get_logger(), "规划失败");
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "规划成功，开始执行...");

    // 执行运动规划
    auto execute_result = move_group_->execute(my_plan);
    bool execute_success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (execute_success)
    {
        RCLCPP_INFO(node_->get_logger(), "执行完成");
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "执行失败");
        return false;
    }
}

/**
 * @brief 主函数
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_pose_controller");

    try
    {
        // 创建MoveIt命名位姿控制器对象
        MoveItPoseController controller(node);

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
