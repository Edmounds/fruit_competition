#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <cmath>

class ImuTfTransformNode : public rclcpp::Node
{
public:
    ImuTfTransformNode() : Node("imu_tf_transform_node")
    {
        // 参数声明和获取
        this->declare_parameter("source_imu_topic", "imu/data");
        this->declare_parameter("target_imu_topic", "imu");
        this->declare_parameter("source_frame", "imu_link");
        this->declare_parameter("target_frame", "imu_transformed");
        this->declare_parameter("base_frame", "base_link");
        
        source_imu_topic_ = this->get_parameter("source_imu_topic").as_string();
        target_imu_topic_ = this->get_parameter("target_imu_topic").as_string();
        source_frame_ = this->get_parameter("source_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();

        // 初始化状态标志
        initial_orientation_set_ = false;
        
        // 创建订阅器和发布器
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            source_imu_topic_, 10,
            std::bind(&ImuTfTransformNode::imu_callback, this, std::placeholders::_1));

        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(target_imu_topic_, 10);

        // 创建TF广播器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "IMU TF Transform Node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", source_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing to: %s", target_imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Source frame: %s", source_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waiting for initial IMU data to set reference orientation...");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 获取当前IMU的四元数
        tf2::Quaternion current_q;
        tf2::fromMsg(msg->orientation, current_q);

        // 如果还没有设置初始参考方向，则记录当前方向作为参考
        if (!initial_orientation_set_) {
            initial_orientation_ = current_q;
            initial_orientation_set_ = true;
            
            // 输出初始角度信息
            tf2::Matrix3x3 m(current_q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            RCLCPP_INFO(this->get_logger(), 
                "Initial orientation set - Roll: %.4f, Pitch: %.4f, Yaw: %.4f (rad)", 
                roll, pitch, yaw);
            RCLCPP_INFO(this->get_logger(), 
                "Initial orientation set - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
                roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
            
            // 第一次接收数据时，差值为零（没有变换）
            tf2::Quaternion diff_q;
            diff_q.setRPY(0.0, 0.0, 0.0);
            
            // 发布初始状态
            publish_transformed_data(msg, diff_q);
            return;
        }

        // 计算当前角度与初始角度的差值
        // 使用四元数逆运算: diff_q = current_q * initial_q^(-1)
        tf2::Quaternion initial_q_inverse = initial_orientation_.inverse();
        tf2::Quaternion diff_q = current_q * initial_q_inverse;

        // 发布变换后的数据
        publish_transformed_data(msg, diff_q);

        // 输出调试信息
        tf2::Matrix3x3 m(current_q);
        double current_roll, current_pitch, current_yaw;
        m.getRPY(current_roll, current_pitch, current_yaw);

        tf2::Matrix3x3 m_diff(diff_q);
        double diff_roll, diff_pitch, diff_yaw;
        m_diff.getRPY(diff_roll, diff_pitch, diff_yaw);

        RCLCPP_DEBUG(this->get_logger(), 
            "Current RPY: [%.4f, %.4f, %.4f] rad, Diff RPY: [%.4f, %.4f, %.4f] rad",
            current_roll, current_pitch, current_yaw,
            diff_roll, diff_pitch, diff_yaw);
        RCLCPP_DEBUG(this->get_logger(), 
            "Current RPY: [%.2f°, %.2f°, %.2f°], Diff RPY: [%.2f°, %.2f°, %.2f°]",
            current_roll * 180.0 / M_PI, current_pitch * 180.0 / M_PI, current_yaw * 180.0 / M_PI,
            diff_roll * 180.0 / M_PI, diff_pitch * 180.0 / M_PI, diff_yaw * 180.0 / M_PI);
    }

    void publish_transformed_data(const sensor_msgs::msg::Imu::SharedPtr msg, const tf2::Quaternion& diff_q)
    {
        // 创建变换后的IMU消息
        auto transformed_imu_msg = std::make_shared<sensor_msgs::msg::Imu>(*msg);
        transformed_imu_msg->header.frame_id = target_frame_;
        transformed_imu_msg->orientation = tf2::toMsg(diff_q);

        // 同样需要变换角速度和线性加速度
        // 获取旋转矩阵来变换向量
        tf2::Matrix3x3 rotation_matrix(diff_q);
        
        // 变换角速度
        tf2::Vector3 angular_vel(msg->angular_velocity.x, 
                                msg->angular_velocity.y, 
                                msg->angular_velocity.z);
        tf2::Vector3 transformed_angular_vel = rotation_matrix * angular_vel;
        
        transformed_imu_msg->angular_velocity.x = transformed_angular_vel.x();
        transformed_imu_msg->angular_velocity.y = transformed_angular_vel.y();
        transformed_imu_msg->angular_velocity.z = transformed_angular_vel.z();

        // 变换线性加速度
        tf2::Vector3 linear_acc(msg->linear_acceleration.x,
                               msg->linear_acceleration.y,
                               msg->linear_acceleration.z);
        tf2::Vector3 transformed_linear_acc = rotation_matrix * linear_acc;
        
        transformed_imu_msg->linear_acceleration.x = transformed_linear_acc.x();
        transformed_imu_msg->linear_acceleration.y = transformed_linear_acc.y();
        transformed_imu_msg->linear_acceleration.z = transformed_linear_acc.z();

        // 发布变换后的IMU数据
        imu_publisher_->publish(*transformed_imu_msg);

        // 发布TF变换
        publish_tf_transform(msg->header.stamp, diff_q);
    }

    void publish_tf_transform(const builtin_interfaces::msg::Time& stamp, const tf2::Quaternion& diff_q)
    {
        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = stamp;
        transformStamped.header.frame_id = base_frame_;
        transformStamped.child_frame_id = target_frame_;

        // 设置变换的旋转部分（角度差值）
        transformStamped.transform.rotation = tf2::toMsg(diff_q);

        // 平移部分保持为零（只是角度变换）
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;

        // 发布TF变换
        tf_broadcaster_->sendTransform(transformStamped);
    }

    // 成员变量
    std::string source_imu_topic_;
    std::string target_imu_topic_;
    std::string source_frame_;
    std::string target_frame_;
    std::string base_frame_;

    tf2::Quaternion initial_orientation_;  // 启动时的初始角度参考方向
    bool initial_orientation_set_;          // 是否已设置初始参考方向

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuTfTransformNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting IMU TF Transform Node...");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in IMU TF Transform Node: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
} 