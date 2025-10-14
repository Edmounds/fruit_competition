#ifndef MOVEIT_CONTROL_HPP
#define MOVEIT_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <robot_control_interfaces/srv/move_to_position.hpp>
#include <memory>

/**
 * @brief MoveIt机械臂控制器类
 *
 * 提供服务接口控制机械臂移动到指定的三维坐标位置
 * 服务名称: /move_to_position
 */
class MoveItController
{
public:
    /**
     * @brief 构造函数
     *
     * @param node ROS 2 节点共享指针
     */
    explicit MoveItController(rclcpp::Node::SharedPtr node);

    /**
     * @brief 析构函数
     */
    ~MoveItController() = default;

    /**
     * @brief 执行运动规划和运动到指定位置
     *
     * @param x 目标X坐标
     * @param y 目标Y坐标
     * @param z 目标Z坐标
     * @return true 规划和执行成功
     * @return false 规划或执行失败
     */
    bool planAndExecute(double x, double y, double z);

private:
    /**
     * @brief 服务回调函数
     *
     * @param request 服务请求（包含目标坐标）
     * @param response 服务响应（包含执行结果）
     */
    void handleMoveToPosition(
        const std::shared_ptr<robot_control_interfaces::srv::MoveToPosition::Request> request,
        std::shared_ptr<robot_control_interfaces::srv::MoveToPosition::Response> response);

    /**
     * @brief 声明ROS参数
     */
    void declareParameters();

    /**
     * @brief 从参数服务器加载参数
     */
    void loadParameters();

    /**
     * @brief 将度数转换为弧度
     *
     * @param degrees 度数
     * @return double 弧度
     */
    double degreesToRadians(double degrees) const;

    /**
     * @brief 将弧度转换为度数
     *
     * @param radians 弧度
     * @return double 度数
     */
    double radiansToDegrees(double radians) const;

    // ROS 2 节点
    rclcpp::Node::SharedPtr node_;

    // MoveIt接口
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // 服务服务器
    rclcpp::Service<robot_control_interfaces::srv::MoveToPosition>::SharedPtr service_;

    // 容差参数
    double tol_pos_;
    double tol_ori_;
};

#endif // MOVEIT_CONTROL_HPP
