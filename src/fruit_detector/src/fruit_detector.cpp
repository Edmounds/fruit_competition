#include "fruit_detector/fruit_detector.hpp"
#include <memory>
#include <algorithm>
#include <mutex>
#include <openvino/openvino.hpp>

// 包含生成的消息头文件
#include "robot_control_interfaces/msg/fruit_info.hpp"

namespace fruit_detector
{

  FruitDetector::FruitDetector(const rclcpp::NodeOptions &options)
      : Node("fruit_detector", options),
        current_fps_(0.0f),
        detection_enabled_(false), // 默认禁用，通过Service启用
        first_frame_(true),
        model_mode_(0), // 默认模式0
        latest_is_ripe_(false),
        latest_fruit_type_(0),
        latest_offset_x_(0.0f),
        latest_offset_y_(0.0f)
  {

    // 声明参数
    this->declare_parameter("image_topic", "/camera/color/image_raw");
    this->declare_parameter("model_path_a", ""); // 成熟度检测模型
    this->declare_parameter("model_path_c", ""); // 种类检测模型
    this->declare_parameter("ripeness_confidence", 0.65f);
    this->declare_parameter("classifier_confidence", 0.75f);
    this->declare_parameter("iou_threshold", 0.3f);
    this->declare_parameter("image_center_x", 320);
    this->declare_parameter("image_center_y", 240);

    // 获取参数
    image_topic_ = this->get_parameter("image_topic").as_string();
    model_path_a_ = this->get_parameter("model_path_a").as_string();
    model_path_c_ = this->get_parameter("model_path_c").as_string();
    ripeness_conf_threshold_ = this->get_parameter("ripeness_confidence").as_double();
    classifier_conf_threshold_ = this->get_parameter("classifier_confidence").as_double();
    iou_threshold_ = this->get_parameter("iou_threshold").as_double();
    image_center_x_ = this->get_parameter("image_center_x").as_int();
    image_center_y_ = this->get_parameter("image_center_y").as_int();

    RCLCPP_INFO(this->get_logger(), "=== Fruit Detector Node Initialized ===");
    RCLCPP_INFO(this->get_logger(), "Ripeness model (A): %s", model_path_a_.c_str());
    RCLCPP_INFO(this->get_logger(), "Classifier model (C): %s", model_path_c_.c_str());
    RCLCPP_INFO(this->get_logger(), "Ripeness confidence: %.2f", ripeness_conf_threshold_);
    RCLCPP_INFO(this->get_logger(), "Classifier confidence: %.2f", classifier_conf_threshold_);

    // 定义模型A（成熟度检测）的类别：2个类别
    std::vector<std::string> classes_a = {"ripe", "unripe"};

    // 定义模型C（种类检测）的类别：8个类别
    std::vector<std::string> classes_c = {
        "lajiao_ripe", "lajiao_unripe",
        "nangua_ripe", "nangua_unripe",
        "onion_ripe", "onion_unripe",
        "tomato_ripe", "tomato_unripe"};

    // 【关键修复】初始化水果类型映射
    // 水果类型：1=辣椒(lajiao), 2=南瓜(nangua), 3=洋葱(onion), 4=番茄(tomato)
    fruit_type_map_ = {
        {"lajiao_ripe", 1},
        {"lajiao_unripe", 1},
        {"nangua_ripe", 2},
        {"nangua_unripe", 2},
        {"onion_ripe", 3},
        {"onion_unripe", 3},
        {"tomato_ripe", 4},
        {"tomato_unripe", 4}};

    // 不再在构造函数中加载模型，改为按需加载以节省资源
    // 模型将在首次使用时通过 ensureModelLoaded() 加载
    RCLCPP_INFO(this->get_logger(), "Models will be loaded on demand to save resources");
    RCLCPP_INFO(this->get_logger(), "Fruit type mapping initialized (lajiao=1, nangua=2, onion=3, tomato=4)");

    // 创建发布者
    processed_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "processed_image", 10);
    detection_publisher_ = this->create_publisher<robot_control_interfaces::msg::FruitInfo>(
        "fruit_detection", 10);

    // 创建订阅者
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic_, 10,
        std::bind(&FruitDetector::imageCallback, this, std::placeholders::_1));

    // 创建Service服务器
    detection_service_ = this->create_service<robot_control_interfaces::srv::FruitDetectionControl>(
        "fruit_detection_control",
        std::bind(&FruitDetector::handleDetectionService, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: processed_image, fruit_detection");
    RCLCPP_INFO(this->get_logger(), "Service: fruit_detection_control (FruitDetectionControl)");
    RCLCPP_INFO(this->get_logger(), "=== Node Ready ===");
  }

  // Service回调 - 控制检测并返回最新结果
  void FruitDetector::handleDetectionService(
      const std::shared_ptr<robot_control_interfaces::srv::FruitDetectionControl::Request> request,
      std::shared_ptr<robot_control_interfaces::srv::FruitDetectionControl::Response> response)
  {
    uint8_t new_mode = request->model_mode;

    RCLCPP_INFO(this->get_logger(), "Service called: enable=%s, model_mode=%d",
                request->enable ? "true" : "false", new_mode);

    if (!request->enable)
    {
      // 禁用检测
      detection_enabled_ = false;
      response->success = true;
      response->message = "Detection disabled";
      RCLCPP_INFO(this->get_logger(), "Detection disabled");
      return;
    }

    // 如果检测已经启用，且请求的是相同模式，直接返回成功
    if (detection_enabled_ && new_mode == model_mode_)
    {
      RCLCPP_INFO(this->get_logger(), "Detection already enabled with mode %d", model_mode_);
      response->success = true;
      response->message = "Detection already enabled with same mode";
      return;
    }

    // 首次启用或模式切换，需要更新状态
    detection_enabled_ = true;

    // 模式切换时，加载所需模型并卸载不需要的模型
    if (new_mode != model_mode_)
    {
      RCLCPP_INFO(this->get_logger(), "Switching model mode from %d to %d", model_mode_, new_mode);
      model_mode_ = new_mode;

      try
      {
        // 先卸载不需要的模型
        unloadUnusedModels(model_mode_);

        // 再加载需要的模型
        ensureModelLoaded(model_mode_);
      }
      catch (const std::exception &e)
      {
        response->success = false;
        response->message = std::string("Failed to load model: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }
    }
    else
    {
      // 模式未改变，只需确保模型已加载
      try
      {
        ensureModelLoaded(model_mode_);
      }
      catch (const std::exception &e)
      {
        response->success = false;
        response->message = std::string("Failed to load model: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        return;
      }
    }

    // 启用检测成功
    response->success = true;
    response->message = "Detection enabled successfully";
    RCLCPP_INFO(this->get_logger(), "Detection enabled with mode %d", model_mode_);
  }

  // 图像回调函数 - 两阶段检测
  void FruitDetector::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // 计算FPS
      auto now = std::chrono::steady_clock::now();
      if (!first_frame_)
      {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_frame_time_)
                           .count();
        if (elapsed > 0)
        {
          current_fps_ = 1000.0f / static_cast<float>(elapsed);
        }
      }
      else
      {
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
      if (!detection_enabled_)
      {
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

      // 根据模式执行不同的检测逻辑
      if (model_mode_ == 0)
      {
        // ===== 模式0：仅成熟度检测 =====
        processRipenessOnly(frame, annotated_frame, msg);
      }
      else if (model_mode_ == 1)
      {
        // ===== 模式1：仅种类检测 =====
        processClassifierOnly(frame, annotated_frame, msg);
      }
      else if (model_mode_ == 2)
      {
        // ===== 模式2：两阶段检测 =====
        processTwoStage(frame, annotated_frame, msg);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Unknown model_mode: %d", model_mode_);
        cv::putText(annotated_frame, "Invalid Mode",
                    cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, cv::Scalar(0, 0, 255), 2);
      }

      // 显示FPS
      int y_offset = 60;
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
    }
    catch (const std::exception &e)
    {
      // 统一异常处理
      RCLCPP_ERROR(this->get_logger(), "检测异常: %s", e.what());

      // 发布错误状态并更新缓存
      {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_is_ripe_ = false;
        latest_fruit_type_ = 0;
        latest_offset_x_ = 0.0f;
        latest_offset_y_ = 0.0f;
      }

      robot_control_interfaces::msg::FruitInfo error_fruit;
      error_fruit.fruit_type = 0;
      error_fruit.is_ripe = false;
      error_fruit.offset_x = 0.0f;
      error_fruit.offset_y = 0.0f;
      detection_publisher_->publish(error_fruit);
    }
  }

  // 模式0：仅成熟度检测
  void FruitDetector::processRipenessOnly(
      cv::Mat &frame,
      cv::Mat &annotated_frame,
      const sensor_msgs::msg::Image::SharedPtr &msg)
  {
    if (!ripeness_inference_)
    {
      RCLCPP_ERROR(this->get_logger(), "Ripeness model not loaded!");
      return;
    }

    // ===== 第一阶段：成熟度检测 =====
    ripeness_inference_->RunInference(frame);
    auto ripeness_detections = ripeness_inference_->GetContours();
    auto ripeness_boxes = convertDetections(ripeness_detections, ripeness_inference_);

    bool has_ripe = false;
    bool has_unripe = false;

    // 绘制检测框并统计
    for (const auto &box : ripeness_boxes)
    {
      if (box.class_name == "ripe")
      {
        has_ripe = true;

        // 在图像上绘制成熟度检测框（绿色）
        cv::rectangle(annotated_frame,
                      cv::Point(box.x1, box.y1),
                      cv::Point(box.x2, box.y2),
                      cv::Scalar(0, 255, 0), 2);

        std::string label = "RIPE " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";
        cv::putText(annotated_frame, label,
                    cv::Point(box.x1, box.y1 - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      }
      else
      {
        has_unripe = true;

        // 未成熟的用灰色标注
        cv::rectangle(annotated_frame,
                      cv::Point(box.x1, box.y1),
                      cv::Point(box.x2, box.y2),
                      cv::Scalar(128, 128, 128), 2);

        std::string label = "UNRIPE " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";
        cv::putText(annotated_frame, label,
                    cv::Point(box.x1, box.y1 - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(128, 128, 128), 2);
      }
    }

    // 更新缓存结果
    {
      std::lock_guard<std::mutex> lock(detection_mutex_);
      latest_is_ripe_ = has_ripe;
      latest_fruit_type_ = 0; // 模式0不检测种类
      latest_offset_x_ = 0.0f;
      latest_offset_y_ = 0.0f;
    }

    // 发布检测结果
    robot_control_interfaces::msg::FruitInfo fruit_info;
    fruit_info.header = msg->header; // 保留时间戳
    fruit_info.is_ripe = has_ripe;
    fruit_info.fruit_type = 0; // 模式0不返回具体种类
    fruit_info.offset_x = 0.0f;
    fruit_info.offset_y = 0.0f;
    detection_publisher_->publish(fruit_info);

    // 显示检测信息
    int y_offset = 30;
    std::string mode_text = "Mode 0: Ripeness Only";
    cv::putText(annotated_frame, mode_text,
                cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

    if (has_ripe)
    {
      cv::putText(annotated_frame, "Status: RIPE fruit detected",
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      RCLCPP_INFO(this->get_logger(), "Ripeness detection: RIPE fruit found");
    }
    else if (has_unripe)
    {
      cv::putText(annotated_frame, "Status: Only UNRIPE fruits",
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(128, 128, 128), 2);
      RCLCPP_INFO(this->get_logger(), "Ripeness detection: Only UNRIPE fruits");
    }
    else
    {
      cv::putText(annotated_frame, "Status: No fruits detected",
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
    }
  }

  // 模式1：仅种类检测
  void FruitDetector::processClassifierOnly(
      cv::Mat &frame,
      cv::Mat &annotated_frame,
      const sensor_msgs::msg::Image::SharedPtr &msg)
  {
    if (!classifier_inference_)
    {
      RCLCPP_ERROR(this->get_logger(), "Classifier model not loaded!");
      return;
    }

    classifier_inference_->RunInference(frame);
    auto classifier_detections = classifier_inference_->GetContours();
    auto classifier_boxes = convertDetections(classifier_detections, classifier_inference_);

    std::vector<BoundingBox> valid_boxes;
    // 只处理_ripe类别（假设主控已确认成熟，只需识别种类）
    for (const auto &box : classifier_boxes)
    {
      if (box.class_name.find("_ripe") == std::string::npos)
      {
        continue;
      }

      valid_boxes.push_back(box);

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
      size_t pos = display_name.find("_ripe");
      if (pos != std::string::npos)
      {
        display_name = display_name.substr(0, pos);
      }

      std::string info_text = display_name + ": (" +
                              std::to_string(static_cast<int>(offset_x)) + ", " +
                              std::to_string(static_cast<int>(offset_y)) + ")";
      cv::putText(annotated_frame, info_text,
                  cv::Point(box.x1, box.y1 - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
    }

    // 更新缓存并发布结果
    if (!valid_boxes.empty())
    {
      {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        const auto &first_box = valid_boxes[0];
        latest_is_ripe_ = true; // 模式1假设都是成熟的
        latest_fruit_type_ = fruit_type_map_[first_box.class_name];
        latest_offset_x_ = first_box.centerX() - image_center_x_;
        latest_offset_y_ = first_box.centerY() - image_center_y_;
      }

      for (const auto &box : valid_boxes)
      {
        auto fruit_info = createFruitInfoMessage(box, image_center_x_, image_center_y_, msg->header);
        detection_publisher_->publish(fruit_info);
      }
    }
    else
    {
      {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_is_ripe_ = true;
        latest_fruit_type_ = 0;
        latest_offset_x_ = 0.0f;
        latest_offset_y_ = 0.0f;
      }

      robot_control_interfaces::msg::FruitInfo empty_fruit;
      empty_fruit.header = msg->header;
      empty_fruit.is_ripe = true; // 模式1假设在已确认成熟的情况下运行
      empty_fruit.fruit_type = 0;
      empty_fruit.offset_x = 0.0f;
      empty_fruit.offset_y = 0.0f;
      detection_publisher_->publish(empty_fruit);
    }

    // 显示检测信息
    int y_offset = 30;
    std::string mode_text = "Mode 1: Classifier Only";
    cv::putText(annotated_frame, mode_text,
                cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

    if (!valid_boxes.empty())
    {
      std::string count_text = "Detected: " + std::to_string(valid_boxes.size()) + " fruit(s)";
      cv::putText(annotated_frame, count_text,
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      RCLCPP_INFO(this->get_logger(), "Classifier: Detected %zu fruit(s)", valid_boxes.size());
    }
    else
    {
      cv::putText(annotated_frame, "No fruits detected",
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
    }
  }

  // 模式2：两阶段检测
  void FruitDetector::processTwoStage(
      cv::Mat &frame,
      cv::Mat &annotated_frame,
      const sensor_msgs::msg::Image::SharedPtr &msg)
  {
    if (!ripeness_inference_ || !classifier_inference_)
    {
      RCLCPP_ERROR(this->get_logger(), "Models not fully loaded for two-stage detection!");
      return;
    }

    // 第一阶段：成熟度检测
    ripeness_inference_->RunInference(frame);
    auto ripeness_detections = ripeness_inference_->GetContours();
    auto ripeness_boxes = convertDetections(ripeness_detections, ripeness_inference_);

    std::vector<BoundingBox> ripe_boxes;
    for (const auto &box : ripeness_boxes)
    {
      if (box.class_name == "ripe")
      {
        ripe_boxes.push_back(box);
        cv::rectangle(annotated_frame,
                      cv::Point(box.x1, box.y1),
                      cv::Point(box.x2, box.y2),
                      cv::Scalar(0, 255, 0), 2);
        std::string label = "ripe " + std::to_string(static_cast<int>(box.confidence * 100)) + "%";
        cv::putText(annotated_frame, label,
                    cv::Point(box.x1, box.y1 - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
      }
      else
      {
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

    // 第二阶段：对成熟水果进行种类检测
    std::vector<BoundingBox> valid_classifier_boxes;
    if (!ripe_boxes.empty())
    {
      classifier_inference_->RunInference(frame);
      auto classifier_detections = classifier_inference_->GetContours();
      auto classifier_boxes = convertDetections(classifier_detections, classifier_inference_);

      for (const auto &box : classifier_boxes)
      {
        if (box.class_name.find("_ripe") == std::string::npos)
        {
          continue;
        }

        if (!checkOverlap(box, ripe_boxes, iou_threshold_))
        {
          continue;
        }

        valid_classifier_boxes.push_back(box);

        cv::rectangle(annotated_frame,
                      cv::Point(box.x1, box.y1),
                      cv::Point(box.x2, box.y2),
                      cv::Scalar(255, 0, 0), 2);

        int center_x = static_cast<int>(box.centerX());
        int center_y = static_cast<int>(box.centerY());
        cv::drawMarker(annotated_frame, cv::Point(center_x, center_y),
                       cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 20, 2);

        float offset_x = box.centerX() - image_center_x_;
        float offset_y = box.centerY() - image_center_y_;

        std::string display_name = box.class_name;
        size_t pos = display_name.find("_ripe");
        if (pos != std::string::npos)
        {
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

    // 更新缓存并发布结果
    if (!valid_classifier_boxes.empty())
    {
      {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        const auto &first_box = valid_classifier_boxes[0];
        latest_is_ripe_ = true;
        latest_fruit_type_ = fruit_type_map_[first_box.class_name];
        latest_offset_x_ = first_box.centerX() - image_center_x_;
        latest_offset_y_ = first_box.centerY() - image_center_y_;
      }

      for (const auto &box : valid_classifier_boxes)
      {
        auto fruit_info = createFruitInfoMessage(box, image_center_x_, image_center_y_, msg->header);
        detection_publisher_->publish(fruit_info);
      }
    }
    else
    {
      {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_is_ripe_ = false;
        latest_fruit_type_ = 0;
        latest_offset_x_ = 0.0f;
        latest_offset_y_ = 0.0f;
      }

      robot_control_interfaces::msg::FruitInfo empty_fruit;
      empty_fruit.header = msg->header;
      empty_fruit.is_ripe = false; // 两阶段模式下，未检测到则认为不成熟
      empty_fruit.fruit_type = 0;
      empty_fruit.offset_x = 0.0f;
      empty_fruit.offset_y = 0.0f;
      detection_publisher_->publish(empty_fruit);
    }

    // 显示检测信息
    int y_offset = 30;
    std::string mode_text = "Mode 2: Two-Stage Detection";
    cv::putText(annotated_frame, mode_text,
                cv::Point(10, y_offset),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

    if (!valid_classifier_boxes.empty())
    {
      std::string count_text = "Detected: " + std::to_string(valid_classifier_boxes.size()) + " ripe fruit(s)";
      cv::putText(annotated_frame, count_text,
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      RCLCPP_DEBUG(this->get_logger(), "Published %zu ripe fruit(s)", valid_classifier_boxes.size());
    }
    else
    {
      cv::putText(annotated_frame, "No ripe fruits detected",
                  cv::Point(10, y_offset + 30),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 165, 255), 2);
      RCLCPP_DEBUG(this->get_logger(), "No ripe fruits detected in stage 2");
    }
  } // 计算IoU
  float FruitDetector::calculateIoU(const BoundingBox &box1, const BoundingBox &box2)
  {
    // 计算交集
    int inter_x1 = std::max(box1.x1, box2.x1);
    int inter_y1 = std::max(box1.y1, box2.y1);
    int inter_x2 = std::min(box1.x2, box2.x2);
    int inter_y2 = std::min(box1.y2, box2.y2);

    if (inter_x2 <= inter_x1 || inter_y2 <= inter_y1)
    {
      return 0.0f; // 无交集
    }

    float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    float box1_area = box1.area();
    float box2_area = box2.area();
    float union_area = box1_area + box2_area - inter_area;

    return inter_area / union_area;
  }

  // 检查是否与成熟区域重叠
  bool FruitDetector::checkOverlap(const BoundingBox &box,
                                   const std::vector<BoundingBox> &ripe_boxes,
                                   float iou_threshold)
  {
    for (const auto &ripe_box : ripe_boxes)
    {
      float iou = calculateIoU(box, ripe_box);
      if (iou >= iou_threshold)
      {
        return true;
      }
    }
    return false;
  }

  // 转换检测结果为边界框结构
  std::vector<FruitDetector::BoundingBox> FruitDetector::convertDetections(
      const std::vector<std::vector<cv::Point2f>> &detections,
      const std::unique_ptr<yolo::Inference> &inference)
  {
    std::vector<BoundingBox> boxes;

    for (const auto &detection : detections)
    {
      if (detection.empty())
      {
        continue;
      }

      BoundingBox box;
      // 提取类别和置信度 (检测器存储在contour的第一个点)
      box.class_id = static_cast<int>(detection[0].x);
      box.confidence = detection[0].y;

      // 获取类别名称
      auto class_names = inference->GetClassNames();
      if (box.class_id >= 0 && box.class_id < static_cast<int>(class_names.size()))
      {
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

  // 创建单个水果信息消息
  FruitDetector::FruitInfoMsg FruitDetector::createFruitInfoMessage(
      const BoundingBox &box,
      int img_center_x,
      int img_center_y,
      const std_msgs::msg::Header &header)
  {
    robot_control_interfaces::msg::FruitInfo fruit_info;
    fruit_info.header = header;

    // 判断是否成熟（根据类别名称）
    fruit_info.is_ripe = (box.class_name.find("_ripe") != std::string::npos);

    // 获取水果类型编号
    auto it = fruit_type_map_.find(box.class_name);
    if (it != fruit_type_map_.end())
    {
      fruit_info.fruit_type = it->second;
    }
    else
    {
      fruit_info.fruit_type = 0; // 未知类型
    }

    // 计算偏移量（相对于图像中心）
    fruit_info.offset_x = box.centerX() - img_center_x;
    fruit_info.offset_y = box.centerY() - img_center_y;

    return fruit_info;
  }

  // 确保指定模型已加载
  void FruitDetector::ensureModelLoaded(uint8_t mode)
  {
    if (mode == 0)
    {
      // 加载成熟度检测模型
      if (!ripeness_inference_)
      {
        RCLCPP_INFO(this->get_logger(), "Loading ripeness detection model (A)...");
        std::vector<std::string> classes_a = {"ripe", "unripe"};
        ripeness_inference_ = std::make_unique<yolo::Inference>(
            model_path_a_, classes_a, ripeness_conf_threshold_, iou_threshold_);
        RCLCPP_INFO(this->get_logger(), "Ripeness model loaded successfully");
      }
    }
    else if (mode == 1)
    {
      // 加载种类检测模型
      if (!classifier_inference_)
      {
        RCLCPP_INFO(this->get_logger(), "Loading classifier detection model (C)...");
        std::vector<std::string> classes_c = {
            "lajiao_ripe", "lajiao_unripe",
            "nangua_ripe", "nangua_unripe",
            "onion_ripe", "onion_unripe",
            "tomato_ripe", "tomato_unripe"};
        classifier_inference_ = std::make_unique<yolo::Inference>(
            model_path_c_, classes_c, classifier_conf_threshold_, iou_threshold_);
        RCLCPP_INFO(this->get_logger(), "Classifier model loaded successfully");
      }
    }
    else if (mode == 2)
    {
      // 两阶段模式需要加载两个模型
      ensureModelLoaded(0);
      ensureModelLoaded(1);
    }
  }

  // 卸载未使用的模型以释放资源
  void FruitDetector::unloadUnusedModels(uint8_t keep_mode)
  {
    if (keep_mode == 0)
    {
      // 只保留成熟度模型，卸载种类模型
      if (classifier_inference_)
      {
        RCLCPP_INFO(this->get_logger(), "Unloading classifier model to save resources");
        classifier_inference_.reset();
      }
    }
    else if (keep_mode == 1)
    {
      // 只保留种类模型，卸载成熟度模型
      if (ripeness_inference_)
      {
        RCLCPP_INFO(this->get_logger(), "Unloading ripeness model to save resources");
        ripeness_inference_.reset();
      }
    }
    // keep_mode == 2 时不卸载任何模型
  }

} // namespace fruit_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fruit_detector::FruitDetector)
