#include "moveit_servo.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

namespace moveit_controller
{

    MoveItServoNode::MoveItServoNode(const rclcpp::NodeOptions &options)
        : Node("moveit_servo_node", options),
          state_(ServoState::IDLE),
          current_offset_x_(0.0),
          current_offset_y_(0.0),
          has_target_(false)
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt Servo节点正在初始化...");

        // 初始化节点
        initialize();

        RCLCPP_INFO(this->get_logger(), "MoveIt Servo节点初始化完成");
    }

    MoveItServoNode::~MoveItServoNode()
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt Servo节点正在关闭...");
    }

    void MoveItServoNode::initialize()
    {
        // 声明和获取参数
        this->declare_parameter<double>("control_rate", 10.0);
        this->declare_parameter<double>("alignment_threshold", 10.0); // 像素
        this->declare_parameter<double>("move_down_distance", 0.1);   // 米
        this->declare_parameter<double>("pixel_to_meter_x", 0.001);   // 像素到米转换系数
        this->declare_parameter<double>("pixel_to_meter_y", 0.001);   // 像素到米转换系数
        this->declare_parameter<double>("max_xy_velocity", 0.05);     // 米/秒
        this->declare_parameter<std::string>("planning_group", "manipulator");

        control_rate_ = this->get_parameter("control_rate").as_double();
        alignment_threshold_ = this->get_parameter("alignment_threshold").as_double();
        move_down_distance_ = this->get_parameter("move_down_distance").as_double();
        pixel_to_meter_x_ = this->get_parameter("pixel_to_meter_x").as_double();
        pixel_to_meter_y_ = this->get_parameter("pixel_to_meter_y").as_double();
        max_xy_velocity_ = this->get_parameter("max_xy_velocity").as_double();
        planning_group_ = this->get_parameter("planning_group").as_string();

        RCLCPP_INFO(this->get_logger(), "控制频率: %.2f Hz", control_rate_);
        RCLCPP_INFO(this->get_logger(), "对齐阈值: %.2f 像素", alignment_threshold_);
        RCLCPP_INFO(this->get_logger(), "下移距离: %.3f 米", move_down_distance_);
        RCLCPP_INFO(this->get_logger(), "规划组: %s", planning_group_.c_str());

        // 初始化MoveIt接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);

        RCLCPP_INFO(this->get_logger(), "MoveIt规划组初始化完成: %s",
                    move_group_->getName().c_str());

        // 初始化订阅器
        detection_sub_ = this->create_subscription<robot_control_interfaces::msg::DetectionArray>(
            "fruit_detection",
            10,
            std::bind(&MoveItServoNode::detection_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "订阅水果检测话题: fruit_detection");

        // 创建定时器
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&MoveItServoNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "控制定时器已启动");
    }

    void MoveItServoNode::detection_callback(const robot_control_interfaces::msg::DetectionArray::SharedPtr msg)
    {
        // 如果当前正在执行任务，忽略新的检测结果
        if (state_ != ServoState::IDLE)
        {
            return;
        }

        // 检查是否有检测到的水果
        if (msg->fruits.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "未检测到水果");
            has_target_ = false;
            return;
        }

        // 选择第一个水果作为目标（新消息格式没有isgood字段）
        const auto &fruit = msg->fruits[0];
        current_offset_x_ = fruit.offset_x;
        current_offset_y_ = fruit.offset_y;
        has_target_ = true;
        state_ = ServoState::ALIGNING_XY;

        RCLCPP_INFO(this->get_logger(),
                    "检测到目标水果 - 类型: %d, 偏移: (%.2f, %.2f) 像素",
                    fruit.type, current_offset_x_, current_offset_y_);
    }

    void MoveItServoNode::timer_callback()
    {
        switch (state_)
        {
        case ServoState::IDLE:
            // 空闲状态，等待检测结果
            break;

        case ServoState::ALIGNING_XY:
        {
            // 执行XY平面对齐
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "正在对齐XY平面...");

            if (execute_xy_alignment())
            {
                RCLCPP_INFO(this->get_logger(), "XY对齐完成，开始向下移动");
                state_ = ServoState::MOVING_DOWN;
            }
            break;
        }

        case ServoState::MOVING_DOWN:
        {
            // 执行向下移动
            RCLCPP_INFO(this->get_logger(), "正在向下移动...");

            if (execute_move_down())
            {
                RCLCPP_INFO(this->get_logger(), "移动完成！");
                state_ = ServoState::COMPLETED;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "向下移动失败，重置状态");
                reset_state();
            }
            break;
        }

        case ServoState::COMPLETED:
        {
            // 完成状态，重置等待下一次
            RCLCPP_INFO(this->get_logger(), "任务完成，重置状态");
            reset_state();
            break;
        }
        }
    }

    bool MoveItServoNode::execute_xy_alignment()
    {
        // 检查是否已经对齐
        double offset_magnitude = std::sqrt(current_offset_x_ * current_offset_x_ +
                                            current_offset_y_ * current_offset_y_);

        if (offset_magnitude < alignment_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "已对齐 (偏移: %.2f 像素)", offset_magnitude);
            return true;
        }

        // 获取当前位姿
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

        // 计算目标位姿（只改变X和Y，保持Z不变）
        geometry_msgs::msg::Pose target_pose = current_pose.pose;

        // 将像素偏移转换为米，并应用到位姿
        // 注意：这里假设相机坐标系与机械臂坐标系的对应关系
        // offset_x 对应机械臂的 Y 轴，offset_y 对应机械臂的 -X 轴（常见配置）
        double delta_x = -current_offset_y_ * pixel_to_meter_y_; // 像素Y -> 机械臂X（取反）
        double delta_y = current_offset_x_ * pixel_to_meter_x_;  // 像素X -> 机械臂Y

        // 限制单次移动的幅度
        double move_distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        double max_move = max_xy_velocity_ / control_rate_; // 单次最大移动距离

        if (move_distance > max_move)
        {
            double scale = max_move / move_distance;
            delta_x *= scale;
            delta_y *= scale;
        }

        target_pose.position.x += delta_x;
        target_pose.position.y += delta_y;
        // Z坐标保持不变

        RCLCPP_DEBUG(this->get_logger(),
                     "XY移动: delta_x=%.4f, delta_y=%.4f (偏移: %.2f, %.2f 像素)",
                     delta_x, delta_y, current_offset_x_, current_offset_y_);

        // 设置目标位姿并规划
        move_group_->setPoseTarget(target_pose);

        // 执行规划和移动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_->execute(plan);
            RCLCPP_DEBUG(this->get_logger(), "XY移动执行成功");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "XY移动规划失败");
        }

        // 返回false表示还未完全对齐（需要持续调整）
        return false;
    }

    bool MoveItServoNode::execute_move_down()
    {
        // 获取当前位姿
        geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

        // 计算目标位姿（只改变Z）
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.z -= move_down_distance_;

        RCLCPP_INFO(this->get_logger(),
                    "向下移动: 从 Z=%.3f 到 Z=%.3f (距离=%.3f)",
                    current_pose.pose.position.z,
                    target_pose.position.z,
                    move_down_distance_);

        // 设置目标位姿
        move_group_->setPoseTarget(target_pose);

        // 执行规划和移动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            auto result = move_group_->execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "向下移动执行成功");
                return true;
            }
        }

        RCLCPP_ERROR(this->get_logger(), "向下移动失败");
        return false;
    }

    void MoveItServoNode::reset_state()
    {
        state_ = ServoState::IDLE;
        has_target_ = false;
        current_offset_x_ = 0.0;
        current_offset_y_ = 0.0;
        RCLCPP_DEBUG(this->get_logger(), "状态已重置");
    }

} // namespace moveit_controller

/**
 * @brief 主函数
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 退出状态码
 */
int main(int argc, char **argv)
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<moveit_controller::MoveItServoNode>();

    // 使用MultiThreadedExecutor（推荐用于Action Server等场景）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "开始运行MoveIt Servo节点...");

    // 运行节点
    executor.spin();

    // 清理
    rclcpp::shutdown();

    return 0;
}
