#ifndef MOVEIT_POSE_CONTROLLER_HPP
#define MOVEIT_POSE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <robot_control_interfaces/srv/move_to_pose.hpp>

/**
 * @brief MoveIt命名位姿控制器类
 *
 * 通过ROS 2 Service接口，接收位姿名称，控制机械臂移动到MoveIt中预定义的命名位姿
 */
class MoveItPoseController
{
public:
    /**
     * @brief 构造函数
     *
     * @param node ROS 2节点共享指针
     */
    explicit MoveItPoseController(rclcpp::Node::SharedPtr node);

    /**
     * @brief 析构函数
     */
    ~MoveItPoseController() = default;

private:
    /**
     * @brief 服务回调函数 - 处理移动到命名位姿的请求
     *
     * @param request 服务请求，包含位姿名称
     * @param response 服务响应，包含执行结果
     */
    void handleMoveToPose(
        const std::shared_ptr<robot_control_interfaces::srv::MoveToPose::Request> request,
        std::shared_ptr<robot_control_interfaces::srv::MoveToPose::Response> response);

    /**
     * @brief 执行移动到命名位姿
     *
     * @param pose_name 位姿名称
     * @return true 执行成功
     * @return false 执行失败
     */
    bool moveToNamedTarget(const std::string &pose_name);

    // ROS 2节点
    rclcpp::Node::SharedPtr node_;

    // MoveIt MoveGroup接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // 服务服务器
    rclcpp::Service<robot_control_interfaces::srv::MoveToPose>::SharedPtr service_;
};

#endif // MOVEIT_POSE_CONTROLLER_HPP
