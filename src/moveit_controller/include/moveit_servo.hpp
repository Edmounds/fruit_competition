#ifndef MOVEIT_SERVO_HPP
#define MOVEIT_SERVO_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <robot_control_interfaces/msg/detection_array.hpp>
#include <robot_control_interfaces/msg/fruit_info.hpp>
#include <memory>

namespace moveit_controller
{
    /**
     * @brief 视觉伺服控制状态
     */
    enum class ServoState
    {
        IDLE,     // 空闲，等待检测结果
        SERVO_XY, // 实时伺服XY平面
        SERVO_Z,  // 实时伺服Z轴向下移动
        COMPLETED // 完成
    };

    /**
     * @brief MoveIt Servo控制节点类
     *
     * 该类实现了基于MoveIt的实时机械臂视觉伺服控制功能
     * 订阅水果检测消息，控制机械臂移动到水果位置
     */
    class MoveItServoNode : public rclcpp::Node
    {
    public:
        /**
         * @brief 构造函数
         *
         * @param options 节点选项
         */
        explicit MoveItServoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        /**
         * @brief 析构函数
         */
        ~MoveItServoNode();

    private:
        /**
         * @brief 初始化节点
         *
         * 初始化参数、话题订阅、服务等
         */
        void initialize();

        /**
         * @brief 定时器回调函数
         *
         * 定期执行的控制循环
         */
        void timer_callback();

        /**
         * @brief 水果检测消息回调函数
         *
         * @param msg 检测数组消息
         */
        void detection_callback(const robot_control_interfaces::msg::DetectionArray::SharedPtr msg);

        /**
         * @brief 发送Servo速度命令
         *
         * 基于当前offset计算并发送TwistStamped消息
         */
        void send_servo_command();

        /**
         * @brief 重置控制状态
         */
        void reset_state();

        // MoveIt接口
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

        // Servo相关
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

        // 订阅器
        rclcpp::Subscription<robot_control_interfaces::msg::DetectionArray>::SharedPtr detection_sub_;

        // 定时器
        rclcpp::TimerBase::SharedPtr timer_;

        // 状态变量
        ServoState state_;
        float current_offset_x_; // 当前X轴偏移量（像素）
        float current_offset_y_; // 当前Y轴偏移量（像素）
        bool has_target_;        // 是否有目标
        int frame_counter_;      // 帧计数器，用于触发状态转换

        // 参数
        double alignment_threshold_; // 对齐阈值（像素）
        double move_down_distance_;  // 向下移动距离（米）
        double pixel_to_meter_x_;    // X轴像素到米的转换系数
        double pixel_to_meter_y_;    // Y轴像素到米的转换系数
        double max_xy_velocity_;     // XY方向最大速度（米/秒）
        double max_z_velocity_;      // Z方向最大速度（米/秒）
        double control_rate_;        // 控制频率（Hz）
        std::string planning_group_; // 规划组名称
        std::string ee_frame_;       // 末端执行器坐标系名称
    };

} // namespace moveit_controller

#endif // MOVEIT_SERVO_HPP
