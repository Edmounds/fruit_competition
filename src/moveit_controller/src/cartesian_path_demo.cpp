#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

// 为了简化代码，我们使用 using namespace
using namespace std::chrono_literals;

int main(int argc, char *argv[])
{

    // 1. 初始化 ROS 2 C++ 客户端库
    rclcpp::init(argc, argv);

    // 创建一个节点
    // 我们需要将 use_sim_time 参数设置为 true，如果我们的环境（如Gazebo）使用的是模拟时间
    auto node = std::make_shared<rclcpp::Node>(
        "cartesian_path_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // 声明参数
    node->declare_parameter("target_x", 0.0);
    node->declare_parameter("target_y", 0.1);
    node->declare_parameter("target_z", -0.2);

    // 从config中读取参数
    double target_x = node->get_parameter("target_x").as_double();
    double target_y = node->get_parameter("target_y").as_double();
    double target_z = node->get_parameter("target_z").as_double();

    // 创建一个日志记录器
    auto const logger = rclcpp::get_logger("cartesian_path_demo");

    // 为了让 MoveGroupInterface 正确初始化，需要在一个执行器中 spin 节点
    // 我们将在一个单独的线程中执行此操作
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    // 2. 初始化 MoveGroupInterface
    // MoveGroupInterface 是与 MoveIt 的 move_group 节点交互的主要接口
    // "panda_arm" 是你在 SRDF 文件中定义的规划组名
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    RCLCPP_INFO(logger, "target_x: %.2f, target_y: %.2f, target_z: %.2f", target_x, target_y, target_z);

    // 等待 MoveGroupInterface 完全初始化
    // 这是一个可选的步骤，但有助于调试
    // rclcpp::sleep_for(2s);

    // 3. 获取当前末端执行器的位姿
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    // 使用日志打印当前位姿
    RCLCPP_INFO(logger, "当前位姿: x=%.2f, y=%.2f, z=%.2f",
                current_pose.position.x,
                current_pose.position.y,
                current_pose.position.z);

    // 4. 定义路标点 (Waypoints)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // 首先添加当前位姿作为路径的起点
    waypoints.push_back(current_pose);

    // 创建一个新的目标位姿
    geometry_msgs::msg::Pose target_pose = current_pose;
    // 在X轴方向上增加 0.2 米 (20厘米)
    target_pose.position.x += target_x;
    target_pose.position.z += target_z;
    target_pose.position.y += target_y;
    waypoints.push_back(target_pose);

    // 5. 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 关节空间跳跃阈值，设为0.0表示禁用
    const double eef_step = 0.01;      // 路径的分辨率，比如0.01表示每隔1cm计算一个点

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    // fraction 表示路径规划的成功率 (0.0到1.0)，1.0表示完全成功
    if (fraction < 1.0)
    {
        RCLCPP_ERROR(logger, "笛卡尔路径规划失败，成功率: %.2f%%", fraction * 100.0);
    }
    else
    {
        RCLCPP_INFO(logger, "笛卡尔路径规划成功！准备执行。");
        // 6. 执行规划好的路径
        move_group.execute(trajectory);
    }

    // 关闭 ROS 2
    rclcpp::shutdown();
    return 0;
}