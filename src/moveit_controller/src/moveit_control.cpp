#include <rclcpp/rclcpp.hpp>
// 运动规划和执行功能进行交互的头文件
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <Eigen/Dense>

#include <cmath>
#include "moveit_controller/utils.h"

using std::vector;
using std::string;
using std::cout;
using std::endl;
using rclcpp::Node;
using rclcpp::init;

// 定义一个将度数转换为弧度的函数
double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}


/**
 * @brief 声明节点参数（位置、姿态、容差）
 *
 * @param node rclcpp 节点共享指针
 */
void declare_parameters(Node::SharedPtr node) {
    node->declare_parameter("position.x", 0.000);
    node->declare_parameter("position.y", 0.000);
    node->declare_parameter("position.z", 0.000);

    node->declare_parameter("orientation.roll", 0.0);
    node->declare_parameter("orientation.pitch", 0.0);

    node->declare_parameter("Tolerances.position", 0.05);
    node->declare_parameter("Tolerances.orientation", 0.05);
}

vector<double> get_parameters(Node::SharedPtr node){

    vector<double> parameters;
    parameters.push_back(node->get_parameter("position.x").as_double());
    parameters.push_back(node->get_parameter("position.y").as_double());
    parameters.push_back(node->get_parameter("position.z").as_double());

    //orientation
    parameters.push_back(node->get_parameter("orientation.roll").as_double());
    parameters.push_back(node->get_parameter("orientation.pitch").as_double());
    //tolerances
    parameters.push_back(node->get_parameter("Tolerances.position").as_double());
    parameters.push_back(node->get_parameter("Tolerances.orientation").as_double());


    RCLCPP_WARN(node->get_logger(), "Parameters read from the parameter server:");
    RCLCPP_WARN(node->get_logger(), "PARM:Target position: [%.3f, %.3f, %.3f]", parameters[0], parameters[1], parameters[2]);
    RCLCPP_WARN(node->get_logger(), "PARM:Tolerances: [%.3f, %.3f]", parameters[5], parameters[6]);
    return parameters;
}


int main(int argc, char** argv)
{
    init(argc, argv);
    auto node = Node::make_shared("moveit_control");

    declare_parameters(node);
    
    vector<double> parameters = get_parameters(node);

    

    double pos_x = parameters[0];
    double pos_y = parameters[1];
    double pos_z = parameters[2];
    
    vector<double> current_position = {0.0, -2.0}; 
    vector<double> target_position = {pos_x, pos_y}; 

    double yaw = -Utils::CalculateRelativeAngle(current_position, target_position);

    yaw = 0.0;

    // double yaw = atan2(pos_y, pos_x);
    double roll = degreesToRadians(parameters[3]);
    double pitch = degreesToRadians(parameters[4]);
    double tol_pos = parameters[6];
    double tol_ori = parameters[7];

    

    // 打印读取的参数
    RCLCPP_INFO(node->get_logger(), "Target position: [%.3f, %.3f, %.3f]", pos_x, pos_y, pos_z);
    RCLCPP_WARN(node->get_logger(), "Target orientation: [%.3f, %.3f, %.3f]", radiansToDegrees(roll), radiansToDegrees(pitch), radiansToDegrees(yaw));


    // 创建MoveGroupInterface对象，指定规划组名称
    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");

    // 获取规划组的参考坐标系
    string planning_frame = move_group.getPlanningFrame();
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", planning_frame.c_str());

    // 获取规划组的末端执行器名称
    string end_effector_link = move_group.getEndEffectorLink();
    RCLCPP_INFO(node->get_logger(), "End effector link: %s", end_effector_link.c_str());

    // 设置目标位置
    geometry_msgs::msg::Pose target_pose;

    // 创建一个四元数对象
    tf2::Quaternion quaternion;
    // 根据欧拉角设置四元数
    quaternion.setRPY(roll, pitch, yaw);

    // 将tf2的四元数转换为geometry_msgs的四元数
    target_pose.orientation = tf2::toMsg(quaternion);

    RCLCPP_INFO(node->get_logger(), "new orientation %f %f %f %f", target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);

    // 使用从参数读取的位置值
    target_pose.position.x = pos_x;
    target_pose.position.y = pos_y;
    target_pose.position.z = pos_z;

    RCLCPP_INFO(node->get_logger(), "new position %f %f %f", target_pose.position.x, target_pose.position.y, target_pose.position.z);

    move_group.setGoalPositionTolerance(tol_pos);
    move_group.setGoalOrientationTolerance(tol_ori);
    // move_group.setPoseTarget(target_pose);
    move_group.setPositionTarget(target_pose.position.x, target_pose.position.y, target_pose.position.z);

    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node->get_logger(), "Planning %s", success ? "SUCCEEDED" : "FAILED");

    // 执行运动规划
    if (success) {
        move_group.execute(my_plan);
    }

    rclcpp::shutdown();
    return 0;
}