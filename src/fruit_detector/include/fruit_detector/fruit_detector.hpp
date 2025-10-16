#ifndef FRUIT_DETECTOR_HPP_
#define FRUIT_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include "fruit_detector/openvino_detect.hpp"

// 包含自定义消息头文件
#include "fruit_detector/msg/detection_array.hpp"
#include "fruit_detector/msg/fruit_info.hpp"

namespace fruit_detector
{

  /**
   * @brief 水果检测节点 - 两阶段YOLO检测
   *
   * 第一阶段：使用模型A检测成熟度（ripe/unripe）
   * 第二阶段：对成熟水果使用模型C检测种类（辣椒/南瓜/洋葱/番茄）
   */
  class FruitDetector : public rclcpp::Node
  {
  public:
    explicit FruitDetector(const rclcpp::NodeOptions &options);
    virtual ~FruitDetector() = default;

  private:
    // 类型别名
    using DetectionArrayMsg = msg::DetectionArray;

    // 结构体：用于存储边界框信息
    struct BoundingBox
    {
      int x1, y1, x2, y2;
      float confidence;
      int class_id;
      std::string class_name;

      // 计算中心点
      float centerX() const { return (x1 + x2) / 2.0f; }
      float centerY() const { return (y1 + y2) / 2.0f; }
      float area() const { return (x2 - x1) * (y2 - y1); }
    };

    /**
     * @brief 图像回调函数 - 执行两阶段检测
     *
     * @param msg 输入图像消息
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    /**
     * @brief 检测开关控制回调
     *
     * @param msg Bool消息（true=启用，false=禁用）
     */
    void detectionEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /**
     * @brief 计算两个边界框的IoU（交并比）
     *
     * @param box1 第一个边界框
     * @param box2 第二个边界框
     * @return float IoU值（0-1）
     */
    float calculateIoU(const BoundingBox &box1, const BoundingBox &box2);

    /**
     * @brief 检查检测框是否与成熟区域重叠
     *
     * @param box 待检查的边界框
     * @param ripe_boxes 成熟水果的边界框列表
     * @param iou_threshold IoU阈值（默认0.3）
     * @return bool 是否有足够的重叠
     */
    bool checkOverlap(const BoundingBox &box,
                      const std::vector<BoundingBox> &ripe_boxes,
                      float iou_threshold = 0.3f);

    /**
     * @brief 创建水果检测消息
     *
     * @param classifier_detections 种类检测结果
     * @param ripe_boxes 成熟区域边界框
     * @param img_center_x 图像中心X坐标
     * @param img_center_y 图像中心Y坐标
     * @return DetectionArrayMsg 水果信息数组
     */
    DetectionArrayMsg createFruitInfoMessage(
        const std::vector<BoundingBox> &classifier_detections,
        const std::vector<BoundingBox> &ripe_boxes,
        int img_center_x,
        int img_center_y);

    /**
     * @brief 将检测结果转换为边界框结构
     *
     * @param detections YOLO推理器输出的检测结果
     * @param inference 推理器指针（用于获取类别名称）
     * @return std::vector<BoundingBox> 边界框列表
     */
    std::vector<BoundingBox> convertDetections(
        const std::vector<std::vector<cv::Point2f>> &detections,
        const std::unique_ptr<yolo::Inference> &inference);

    // 参数
    std::string image_topic_;         // 图像话题
    std::string model_path_a_;        // 成熟度检测模型路径
    std::string model_path_c_;        // 种类检测模型路径
    float ripeness_conf_threshold_;   // 成熟度检测置信度阈值
    float classifier_conf_threshold_; // 种类检测置信度阈值
    float iou_threshold_;             // IoU阈值
    int image_center_x_;              // 图像中心X坐标
    int image_center_y_;              // 图像中心Y坐标
    bool detection_enabled_;          // 检测开关

    // 水果类型映射（类别名称 -> 类型编号）
    std::map<std::string, int> fruit_type_map_;

    // OpenVINO推理器 - 两个模型（A: 成熟度, C: 种类）
    std::unique_ptr<yolo::Inference> ripeness_inference_;   // 模型A：成熟度检测
    std::unique_ptr<yolo::Inference> classifier_inference_; // 模型C：种类检测

    // ROS订阅和发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr detection_enable_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher_;
    rclcpp::Publisher<DetectionArrayMsg>::SharedPtr detection_publisher_;

    // FPS计算相关
    std::chrono::time_point<std::chrono::steady_clock> last_frame_time_;
    float current_fps_;
    bool first_frame_;
  };

} // namespace fruit_detector

#endif // FRUIT_DETECTOR_HPP_