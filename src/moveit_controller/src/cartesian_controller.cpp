#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <robot_control_interfaces/srv/move_to_position.hpp>
#include <vector>
#include <memory>

using namespace std::chrono_literals;

/**
 * @brief 笛卡尔空间控制器节点类
 *
 * 提供 Service 接口用于控制机械臂在笛卡尔空间中移动到指定位置
 */
class CartesianControllerNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    CartesianControllerNode()
        : Node("cartesian_controller",
               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // 创建 MoveGroupInterface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), PLANNING_GROUP);

        // 创建 Service Server
        service_ = this->create_service<robot_control_interfaces::srv::MoveToPosition>(
            "move_to_position",
            std::bind(&CartesianControllerNode::handleMoveRequest,
                      this,
                      std::placeholders::_1,
                      std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "笛卡尔控制器节点已启动，等待服务调用...");
    }

private:
    /**
     * @brief 处理移动请求的回调函数
     *
     * @param request 包含目标位置 (x, y, z) 的请求
     * @param response 包含执行结果的响应
     */
    void handleMoveRequest(
        const std::shared_ptr<robot_control_interfaces::srv::MoveToPosition::Request> request,
        std::shared_ptr<robot_control_interfaces::srv::MoveToPosition::Response> response)
    {
        RCLCPP_INFO(this->get_logger(),
                    "收到移动请求: x=%.3f, y=%.3f, z=%.3f",
                    request->x, request->y, request->z);

        try
        {
            // 1. 获取当前末端执行器的位姿
            geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;

            RCLCPP_INFO(this->get_logger(),
                        "当前位姿: x=%.3f, y=%.3f, z=%.3f",
                        current_pose.position.x,
                        current_pose.position.y,
                        current_pose.position.z);

            // 2. 定义路标点 (Waypoints)
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(current_pose);

            // 创建目标位姿（相对移动）
            geometry_msgs::msg::Pose target_pose = current_pose;
            target_pose.position.x += request->x;
            target_pose.position.y += request->y;
            target_pose.position.z += request->z;
            waypoints.push_back(target_pose);

            RCLCPP_INFO(this->get_logger(),
                        "目标位姿: x=%.3f, y=%.3f, z=%.3f",
                        target_pose.position.x,
                        target_pose.position.y,
                        target_pose.position.z);

            // 3. 计算笛卡尔路径
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0; // 关节空间跳跃阈值
            const double eef_step = 0.01;      // 路径分辨率 (1cm)

            double fraction = move_group_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory);

            RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划完成，成功率: %.2f%%", fraction * 100.0);

            // 4. 检查规划结果并执行
            if (fraction < 0.9) // 至少90%成功率
            {
                response->success = false;
                response->message = "笛卡尔路径规划失败，成功率: " +
                                    std::to_string(fraction * 100.0) + "%";
                RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            }
            else
            {
                // 执行规划好的路径
                moveit::core::MoveItErrorCode error_code = move_group_->execute(trajectory);

                if (error_code == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    response->success = true;
                    response->message = "机械臂移动成功完成";
                    RCLCPP_INFO(this->get_logger(), "机械臂移动成功！");
                }
                else
                {
                    response->success = false;
                    response->message = "机械臂执行轨迹失败，错误码: " +
                                        std::to_string(error_code.val);
                    RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                }
            }
        }
        catch (const std::exception &e)
        {
            response->success = false;
            response->message = std::string("执行过程中发生异常: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // 规划组名称
    static const std::string PLANNING_GROUP;

    // MoveGroupInterface 智能指针
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // Service Server
    rclcpp::Service<robot_control_interfaces::srv::MoveToPosition>::SharedPtr service_;
};

// 静态成员初始化
const std::string CartesianControllerNode::PLANNING_GROUP = "manipulator";

int main(int argc, char *argv[])
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<CartesianControllerNode>();

    // 使用多线程执行器（MoveIt 需要）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "开始运行笛卡尔控制器服务...");
    executor.spin();

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}