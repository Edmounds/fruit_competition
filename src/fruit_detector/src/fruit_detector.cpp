#include "fruit_detector/fruit_detector.hpp"
#include <memory>
#include <algorithm>
#include <openvino/openvino.hpp>

// 包含生成的消息头文件
#include "fruit_detector/msg/fruit_info.hpp"
#include "fruit_detector/msg/detection_array.hpp"

namespace fruit_detector {

FruitDetector::FruitDetector(const rclcpp::NodeOptions &options)
    : Node("fruit_detector", options), 
      current_fps_(0.0f),
      first_frame_(true),
      detection_enabled_(true) {
  
  // 声明参数
  this->declare_parameter("image_topic", "/camera/color/image_raw");
  this->declare_parameter("model_path_a", "");  // 成熟度检测模型
  this->declare_parameter("model_path_c", "");  // 种类检测模型
  this->declare_parameter("ripeness_confidence", 0.6f);
  this->declare_parameter("classifier_confidence", 0.75f);
  this->declare_parameter("iou_threshold", 0.3f);
  this->declare_parameter("image_center_x", 320);
  this->declare_parameter("image_center_y", 240);
  this->declare_parameter("enable_on_start", true);

  // 获取参数
  image_topic_ = this->get_parameter("image_topic").as_string();
  model_path_a_ = this->get_parameter("model_path_a").as_string();
  model_path_c_ = this->get_parameter("model_path_c").as_string();
  ripeness_conf_threshold_ = this->get_parameter("ripeness_confidence").as_double();
  classifier_conf_threshold_ = this->get_parameter("classifier_confidence").as_double();
  iou_threshold_ = this->get_parameter("iou_threshold").as_double();
  image_center_x_ = this->get_parameter("image_center_x").as_int();
  image_center_y_ = this->get_parameter("image_center_y").as_int();
  detection_enabled_ = this->get_parameter("enable_on_start").as_bool();

  RCLCPP_INFO(this->get_logger(), "=== Fruit Detector Node Initialized ===");
  RCLCPP_INFO(this->get_logger(), "Ripeness model (A): %s", model_path_a_.c_str());
  RCLCPP_INFO(this->get_logger(), "Classifier model (C): %s", model_path_c_.c_str());
  RCLCPP_INFO(this->get_logger(), "Ripeness confidence: %.2f", ripeness_conf_threshold_);
  RCLCPP_INFO(this->get_logger(), "Classifier confidence: %.2f", classifier_conf_threshold_);
  RCLCPP_INFO(this->get_logger(), "Detection enabled: %s", detection_enabled_ ? "true" : "false");

  // 初始化水果类型映射（类别名称 -> 类型编号）
  // 类型编号：1=辣椒, 2=南瓜, 3=洋葱, 4=番茄
  fruit_type_map_ = {
    {"lajiao_ripe", 1}, {"lajiao_unripe", 1},
    {"nangua_ripe", 2}, {"nangua_unripe", 2},
    {"onion_ripe", 3}, {"onion_unripe", 3},
    {"tomato_ripe", 4}, {"tomato_unripe", 4}
  };

  // 定义模型A（成熟度检测）的类别：2个类别
  std::vector<std::string> classes_a = {"ripe", "unripe"};
  
  // 定义模型C（种类检测）的类别：8个类别
  std::vector<std::string> classes_c = {
    "lajiao_ripe", "lajiao_unripe", 
    "nangua_ripe", "nangua_unripe",
    "onion_ripe", "onion_unripe", 
    "tomato_ripe", "tomato_unripe"
  };

  // 初始化推理器
  ripeness_inference_ = std::make_unique<yolo::Inference>(
    model_path_a_, classes_a, ripeness_conf_threshold_, iou_threshold_);
  
  classifier_inference_ = std::make_unique<yolo::Inference>(
    model_path_c_, classes_c, classifier_conf_threshold_, iou_threshold_);

  // 创建发布者
  processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "processed_image", 10);
  detection_publisher_ = this->create_publisher<fruit_detector::msg::DetectionArray>(
      "fruit_detection", 10);

  // 创建订阅者
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&FruitDetector::imageCallback, this, std::placeholders::_1));

  detection_enable_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "detection_enable", 10,
      std::bind(&FruitDetector::detectionEnableCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing to: processed_image, fruit_detection");
  RCLCPP_INFO(this->get_logger(), "Control topic: detection_enable (std_msgs/Bool)");
  RCLCPP_INFO(this->get_logger(), "=== Node Ready ===");
}

// 检测开关控制回调
void FruitDetector::detectionEnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  detection_enabled_ = msg->data;
  std::string status = detection_enabled_ ? "enabled" : "disabled";
  RCLCPP_INFO(this->get_logger(), "Detection %s", status.c_str());
}

// 图像回调函数 - 两阶段检测
void FruitDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // 计算FPS
    auto now = std::chrono::steady_clock::now();
    if (!first_frame_) {
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_frame_time_).count();
      if (elapsed > 0) {
        current_fps_ = 1000.0f / static_cast<float>(elapsed);
      }
    } else {
      first_frame_ = false;
    }
    last_frame_time_ = now;
    
    // 转换ROS图像消息为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cv_ptr->image;

    // 获取图像尺寸
    int img_width = frame.cols;
    int img_height = frame.rows;
    
    // 创建可视化图像（复制原图）
    cv::Mat annotated_frame = frame.clone();
    
    // 绘制中心十字线
    cv::line(annotated_frame, cv::Point(0, image_center_y_), 
             cv::Point(img_width, image_center_y_), cv::Scalar(0, 0, 255), 1);
    cv::line(annotated_frame, cv::Point(image_center_x_, 0), 
             cv::Point(image_center_x_, img_height), cv::Scalar(0, 0, 255), 1);
    
    // 如果检测未启用，只发布原图
    if (!detection_enabled_) {
      cv::putText(annotated_frame, "Detection DISABLED", 
                  cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                  0.8, cv::Scalar(0, 0, 255), 2);
      
      // 发布图像
      cv_bridge::CvImage out_img;
      out_img.header = msg->header;
      out_img.encoding = sensor_msgs::image_encodings::BGR8;
      out_img.image = annotated_frame;
      processed_image_publisher_->publish(*out_img.toImageMsg());
      return;
    }
    
    // ===== 第一阶段：成熟度检测 =====
    ripeness_inference_->RunInference(frame);
    auto ripeness_detections = ripeness_inference_->GetContours();
    
    // 转换为边界框结构
    std::vector<BoundingBox> ripe_boxes;
    auto ripeness_boxes = convertDetections(ripeness_detections, ripeness_inference_);
    
    // 筛选出成熟的水果
    for (const auto& box : ripeness_boxes) {
      if (box.class_name == "ripe") {
        ripe_boxes.push_back(box);
        
        // 在图像上绘制成熟度检测框（绿色）
        cv::rectangle(annotated_frame, 
                     cv::Point(box.x1, box.y1), 
                     cv::Point(box.x2, box.y2), 
                     cv::Scalar(0, 255, 0), 2);
        
        std::string label = "ripe " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";
        cv::putText(annotated_frame, label, 
                   cv::Point(box.x1, box.y1 - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      } else {
        // 未成熟的用灰色标注（可选）
        cv::rectangle(annotated_frame, 
                     cv::Point(box.x1, box.y1), 
                     cv::Point(box.x2, box.y2), 
                     cv::Scalar(128, 128, 128), 1);
        
        std::string label = "unripe " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";
        cv::putText(annotated_frame, label, 
                   cv::Point(box.x1, box.y1 - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 1);
      }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Stage 1: Found %zu ripe fruits", ripe_boxes.size());
    
    // ===== 第二阶段：对成熟水果进行种类检测 =====
    std::vector<BoundingBox> valid_classifier_boxes;
    
    if (!ripe_boxes.empty()) {
      // 对整图进行种类检测
      classifier_inference_->RunInference(frame);
      auto classifier_detections = classifier_inference_->GetContours();
      auto classifier_boxes = convertDetections(classifier_detections, classifier_inference_);
      
      // 只处理成熟的类别（过滤掉_unripe）并检查与成熟区域的重叠
      for (const auto& box : classifier_boxes) {
        // 只处理_ripe类别
        if (box.class_name.find("_ripe") == std::string::npos) {
          continue;
        }
        
        // 检查是否与成熟区域重叠
        if (!checkOverlap(box, ripe_boxes, iou_threshold_)) {
          continue;
        }
        
        valid_classifier_boxes.push_back(box);
        
        // 在图像上绘制种类检测框（蓝色）
        cv::rectangle(annotated_frame, 
                     cv::Point(box.x1, box.y1), 
                     cv::Point(box.x2, box.y2), 
                     cv::Scalar(255, 0, 0), 2);
        
        // 绘制中心点
        int center_x = static_cast<int>(box.centerX());
        int center_y = static_cast<int>(box.centerY());
        cv::drawMarker(annotated_frame, cv::Point(center_x, center_y),
                      cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);
        
        // 计算偏移量
        float offset_x = box.centerX() - image_center_x_;
        float offset_y = box.centerY() - image_center_y_;
        
        // 显示水果名称和偏移量
        std::string display_name = box.class_name;
        // 去掉_ripe后缀
        size_t pos = display_name.find("_ripe");
        if (pos != std::string::npos) {
          display_name = display_name.substr(0, pos);
        }
        
        std::string info_text = display_name + ": (" + 
                               std::to_string(static_cast<int>(offset_x)) + ", " +
                               std::to_string(static_cast<int>(offset_y)) + ")";
        cv::putText(annotated_frame, info_text, 
                   cv::Point(box.x1, box.y1 - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Stage 2: Detected %s (type=%d) conf=%.2f, offset=(%.1f, %.1f)",
                    display_name.c_str(), 
                    fruit_type_map_[box.class_name],
                    box.confidence, offset_x, offset_y);
      }
    }
    
    // 创建并发布检测结果
    auto detection_array = createFruitInfoMessage(
      valid_classifier_boxes, ripe_boxes, image_center_x_, image_center_y_);
    detection_publisher_->publish(detection_array);
    
    // 在图像左上角显示检测信息
    int y_offset = 30;
    if (!valid_classifier_boxes.empty()) {
      std::string count_text = "Detected: " + std::to_string(valid_classifier_boxes.size()) + " ripe fruit(s)";
      cv::putText(annotated_frame, count_text, 
                 cv::Point(10, y_offset), 
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      RCLCPP_DEBUG(this->get_logger(), "Published %zu ripe fruit(s)", valid_classifier_boxes.size());
    } else {
      cv::putText(annotated_frame, "No ripe fruits detected", 
                 cv::Point(10, y_offset), 
                 cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
      RCLCPP_DEBUG(this->get_logger(), "No ripe fruits detected in stage 2");
    }
    
    // 显示FPS
    y_offset += 30;
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(current_fps_));
    cv::putText(annotated_frame, fps_text, 
               cv::Point(10, y_offset), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    
    // 发布可视化图像
    cv_bridge::CvImage out_img;
    out_img.header = msg->header;
    out_img.encoding = sensor_msgs::image_encodings::BGR8;
    out_img.image = annotated_frame;
    processed_image_publisher_->publish(*out_img.toImageMsg());

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "CV Bridge异常: %s", e.what());
    // 发布错误状态但不终止节点
    auto error_array = fruit_detector::msg::DetectionArray();
    detection_publisher_->publish(error_array);
  } catch (const ov::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "OpenVINO推理异常: %s", e.what());
    // 发布空检测结果，暂时禁用检测
    auto error_array = fruit_detector::msg::DetectionArray();
    detection_publisher_->publish(error_array);
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(this->get_logger(), "运行时错误: %s", e.what());
    auto error_array = fruit_detector::msg::DetectionArray();
    detection_publisher_->publish(error_array);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "未知异常: %s", e.what());
    auto error_array = fruit_detector::msg::DetectionArray();
    detection_publisher_->publish(error_array);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "捕获到未知类型异常！");
    auto error_array = fruit_detector::msg::DetectionArray();
    detection_publisher_->publish(error_array);
  }
}

// 计算IoU
float FruitDetector::calculateIoU(const BoundingBox& box1, const BoundingBox& box2) {
  // 计算交集
  int inter_x1 = std::max(box1.x1, box2.x1);
  int inter_y1 = std::max(box1.y1, box2.y1);
  int inter_x2 = std::min(box1.x2, box2.x2);
  int inter_y2 = std::min(box1.y2, box2.y2);
  
  if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1) {
    return 0.0f;  // 无交集
  }
  
  float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
  float box1_area = box1.area();
  float box2_area = box2.area();
  float union_area = box1_area + box2_area - inter_area;
  
  return inter_area / union_area;
}

// 检查是否与成熟区域重叠
bool FruitDetector::checkOverlap(const BoundingBox& box, 
                                const std::vector<BoundingBox>& ripe_boxes,
                                float iou_threshold) {
  for (const auto& ripe_box : ripe_boxes) {
    float iou = calculateIoU(box, ripe_box);
    if (iou >= iou_threshold) {
      return true;
    }
  }
  return false;
}

// 转换检测结果为边界框结构
std::vector<FruitDetector::BoundingBox> FruitDetector::convertDetections(
    const std::vector<std::vector<cv::Point2f>>& detections,
    const std::unique_ptr<yolo::Inference>& inference) {
  std::vector<BoundingBox> boxes;
  
  for (const auto& detection : detections) {
    if (detection.empty()) {
      continue;
    }
    
    BoundingBox box;
    // 提取类别和置信度 (检测器存储在contour的第一个点)
    box.class_id = static_cast<int>(detection[0].x);
    box.confidence = detection[0].y;
    
    // 获取类别名称
    auto class_names = inference->GetClassNames();
    if (box.class_id >= 0 && box.class_id < static_cast<int>(class_names.size())) {
      box.class_name = class_names[box.class_id];
    }
    
    // 边界框坐标（假设detection[1]是左上角，detection[2]是右下角）
    box.x1 = static_cast<int>(detection[1].x);
    box.y1 = static_cast<int>(detection[1].y);
    box.x2 = static_cast<int>(detection[2].x);
    box.y2 = static_cast<int>(detection[2].y);
    
    boxes.push_back(box);
  }
  
  return boxes;
}

// 创建水果信息消息
FruitDetector::DetectionArrayMsg FruitDetector::createFruitInfoMessage(
    const std::vector<BoundingBox>& classifier_detections,
    const std::vector<BoundingBox>& ripe_boxes,
    int img_center_x,
    int img_center_y) {
  DetectionArrayMsg detection_array;
  
  for (const auto& box : classifier_detections) {
    fruit_detector::msg::FruitInfo fruit_info;
    
    // 获取水果类型编号
    auto it = fruit_type_map_.find(box.class_name);
    if (it != fruit_type_map_.end()) {
      fruit_info.type = it->second;
    } else {
      fruit_info.type = 0;  // 未知类型
    }
    
    // 计算偏移量（相对于图像中心）
    fruit_info.offset_x = box.centerX() - img_center_x;
    fruit_info.offset_y = box.centerY() - img_center_y;
    fruit_info.isgood = true;  // 默认为好水果
    
    detection_array.fruits.push_back(fruit_info);
  }
  
  return detection_array;
}

} // namespace fruit_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fruit_detector::FruitDetector)
