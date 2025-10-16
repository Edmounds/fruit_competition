#include "moveit_servo.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

namespace moveit_controller
{

    MoveItServoNode::MoveItServoNode(const rclcpp::NodeOptions &options)
        : Node("moveit_servo_node", options),
          state_(ServoState::IDLE),
          current_offset_x_(0.0),
          current_offset_y_(0.0),
          has_target_(false),
          frame_counter_(0)
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

    //============================================================parameters=================================================
    void MoveItServoNode::initialize()
    {
        // 声明和获取参数
        this->declare_parameter<double>("control_rate", 10.0);
        this->declare_parameter<double>("alignment_threshold", 10.0); // 像素
        this->declare_parameter<double>("move_down_distance", 0.2);   // 米
        this->declare_parameter<double>("pixel_to_meter_x", 0.001);   // 像素到米转换系数
        this->declare_parameter<double>("pixel_to_meter_y", 0.001);   // 像素到米转换系数
        this->declare_parameter<double>("max_xy_velocity", 0.05);     // 米/秒
        this->declare_parameter<double>("max_z_velocity", 0.05);      // 米/秒
        this->declare_parameter<std::string>("planning_group", "manipulator");
        this->declare_parameter<std::string>("ee_frame", "link4"); // 末端执行器坐标系
                                                                   //============================================================parameters=================================================
        control_rate_ = this->get_parameter("control_rate").as_double();
        alignment_threshold_ = this->get_parameter("alignment_threshold").as_double();
        move_down_distance_ = this->get_parameter("move_down_distance").as_double();
        pixel_to_meter_x_ = this->get_parameter("pixel_to_meter_x").as_double();
        pixel_to_meter_y_ = this->get_parameter("pixel_to_meter_y").as_double();
        max_xy_velocity_ = this->get_parameter("max_xy_velocity").as_double();
        max_z_velocity_ = this->get_parameter("max_z_velocity").as_double();
        planning_group_ = this->get_parameter("planning_group").as_string();
        ee_frame_ = this->get_parameter("ee_frame").as_string();

        RCLCPP_INFO(this->get_logger(), "控制频率: %.2f Hz", control_rate_);
        RCLCPP_INFO(this->get_logger(), "对齐阈值: %.2f 像素", alignment_threshold_);
        RCLCPP_INFO(this->get_logger(), "下移距离: %.3f 米", move_down_distance_);
        RCLCPP_INFO(this->get_logger(), "规划组: %s", planning_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "末端执行器坐标系: %s", ee_frame_.c_str());

        // 初始化MoveIt接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);

        RCLCPP_INFO(this->get_logger(), "MoveIt规划组初始化完成: %s",
                    move_group_->getName().c_str());

        // 创建TwistStamped发布器（用于发送速度命令）
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        RCLCPP_INFO(this->get_logger(), "TwistStamped发布器已创建");

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
        // 检查是否有检测到的水果
        if (msg->fruits.empty())
        {
            RCLCPP_DEBUG(this->get_logger(), "未检测到水果");
            has_target_ = false;
            return;
        }

        // 获取第一个水果作为目标
        const auto &fruit = msg->fruits[0];

        // 实时更新offset值，不进行状态转换
        current_offset_x_ = fruit.offset_x;
        current_offset_y_ = fruit.offset_y;

        // 如果还没有目标，设置为有目标并转入SERVO_XY状态
        if (!has_target_)
        {
            has_target_ = true;
            state_ = ServoState::SERVO_XY;
            frame_counter_ = 0;

            RCLCPP_INFO(this->get_logger(),
                        "检测到目标水果 - 类型: %d, 偏移: (%.2f, %.2f) 像素",
                        fruit.type, current_offset_x_, current_offset_y_);
        }
        else
        {
            // 持续更新offset，供servo控制使用
            RCLCPP_DEBUG(this->get_logger(),
                         "更新目标偏移: (%.2f, %.2f) 像素",
                         current_offset_x_, current_offset_y_);
        }
    }

    void MoveItServoNode::timer_callback()
    {
        switch (state_)
        {
        case ServoState::IDLE:
            // 空闲状态，等待检测结果
            break;

        case ServoState::SERVO_XY:
        {
            // 实时伺服XY平面，基于连续更新的offset
            send_servo_command();

            // 检查是否已对齐
            double offset_magnitude = std::sqrt(current_offset_x_ * current_offset_x_ +
                                                current_offset_y_ * current_offset_y_);

            if (offset_magnitude < alignment_threshold_)
            {
                frame_counter_++;
                // 连续对齐5帧后才认为完全对齐
                if (frame_counter_ > 5)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "XY对齐完成 (偏移: %.2f 像素)，开始向下移动",
                                offset_magnitude);
                    state_ = ServoState::SERVO_Z;
                    frame_counter_ = 0;
                }
            }
            else
            {
                frame_counter_ = 0; // 重置计数器
            }

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                  "XY伺服中... 偏移: (%.2f, %.2f) 像素",
                                  current_offset_x_, current_offset_y_);
            break;
        }

        case ServoState::SERVO_Z:
        {
            // 实时伺服Z轴向下移动
            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = this->get_clock()->now();
            twist.header.frame_id = ee_frame_;

            // 发送向下速度命令
            twist.twist.linear.z = -max_z_velocity_; // 负数表示向下
            twist.twist.linear.x = 0.0;
            twist.twist.linear.y = 0.0;
            twist.twist.angular.x = 0.0;
            twist.twist.angular.y = 0.0;
            twist.twist.angular.z = 0.0;

            twist_pub_->publish(twist);

            // 这里应该有某种机制来检测是否到达预期的Z距离
            // 由于没有Z方向的反馈，这里设定一个固定的运动时间
            frame_counter_++;
            if (frame_counter_ > static_cast<int>(control_rate_ * 2.0)) // 运动2秒
            {
                RCLCPP_INFO(this->get_logger(), "Z轴移动完成！");
                state_ = ServoState::COMPLETED;
            }

            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                  "Z轴伺服中... (%.1f%% 完成)",
                                  (frame_counter_ * 100.0 / (control_rate_ * 2.0)));
            break;
        }

        case ServoState::COMPLETED:
        {
            // 停止所有运动
            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = this->get_clock()->now();
            twist.header.frame_id = ee_frame_;
            twist.twist.linear.x = 0.0;
            twist.twist.linear.y = 0.0;
            twist.twist.linear.z = 0.0;
            twist_pub_->publish(twist);

            RCLCPP_INFO(this->get_logger(), "任务完成，重置状态");
            reset_state();
            break;
        }
        }
    }

    /**
     * @brief 发送Servo速度命令
     *
     * 基于当前的像素offset计算并发送TwistStamped速度命令
     */
    void MoveItServoNode::send_servo_command()
    {
        // 计算偏移量大小
        double offset_magnitude = std::sqrt(current_offset_x_ * current_offset_x_ +
                                            current_offset_y_ * current_offset_y_);

        // 如果偏移量很小，停止运动
        if (offset_magnitude < alignment_threshold_ / 2.0)
        {
            geometry_msgs::msg::TwistStamped twist;
            twist.header.stamp = this->get_clock()->now();
            twist.header.frame_id = ee_frame_;
            twist.twist.linear.x = 0.0;
            twist.twist.linear.y = 0.0;
            twist.twist.linear.z = 0.0;
            twist_pub_->publish(twist);
            return;
        }

        // 将像素偏移转换为米
        // offset_x 对应机械臂的 Y 轴，offset_y 对应机械臂的 -X 轴（常见配置）
        double velocity_x = -current_offset_y_ * pixel_to_meter_y_; // 像素Y -> 机械臂X
        double velocity_y = current_offset_x_ * pixel_to_meter_x_;  // 像素X -> 机械臂Y

        // 计算速度的大小
        double velocity_magnitude = std::sqrt(velocity_x * velocity_x + velocity_y * velocity_y);

        // 限制最大速度
        if (velocity_magnitude > max_xy_velocity_)
        {
            double scale = max_xy_velocity_ / velocity_magnitude;
            velocity_x *= scale;
            velocity_y *= scale;
        }

        // 创建并发布TwistStamped消息
        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = this->get_clock()->now();
        twist.header.frame_id = ee_frame_;

        // 设置线速度
        twist.twist.linear.x = velocity_x;
        twist.twist.linear.y = velocity_y;
        twist.twist.linear.z = 0.0;

        // 角速度为0
        twist.twist.angular.x = 0.0;
        twist.twist.angular.y = 0.0;
        twist.twist.angular.z = 0.0;

        twist_pub_->publish(twist);

        RCLCPP_DEBUG(this->get_logger(),
                     "Servo速度命令 - vx=%.4f, vy=%.4f (偏移: %.2f, %.2f 像素)",
                     velocity_x, velocity_y, current_offset_x_, current_offset_y_);
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
