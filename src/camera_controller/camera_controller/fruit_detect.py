import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from ultralytics import YOLO # pyright: ignore
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from robot_control_interfaces.msg import FruitInfo, DetectionArray
from robot_control_interfaces.srv import ControlDetection


# 模型路径
models_dir = get_package_share_directory('camera_controller') + '/models/model/'
ripeness_model_path = models_dir + 'RipeLevel.onnx'
classifier_model_path = models_dir + 'FruitClassifier.onnx'

# 加载两个模型
ripeness_model = YOLO(ripeness_model_path)
classifier_model = YOLO(classifier_model_path)


class FruitDetect(Node):
    """水果检测节点
    
    使用两阶段YOLO检测：
    1. 成熟度检测（RipeLevel模型）
    2. 种类检测（FruitClassifier模型，仅对成熟水果）
    支持通过服务动态控制检测的开关
    """
    
    def __init__(self):
        super().__init__('fruit_detect')
        
        # 声明置信度阈值参数
        self.declare_parameter('ripeness_confidence', 0.6)  # 成熟度检测阈值
        self.declare_parameter('classifier_confidence', 0.75)  # 种类检测阈值
        self.ripeness_conf = self.get_parameter('ripeness_confidence').value
        self.classifier_conf = self.get_parameter('classifier_confidence').value
        
        # 声明图像中心坐标参数（用于计算偏移量）
        self.declare_parameter('image_center_x', 320)  # 默认640x480图像的中心
        self.declare_parameter('image_center_y', 240)
        
        # 声明默认检测状态
        self.declare_parameter('enable_on_start', True)
        self.detection_enabled = self.get_parameter('enable_on_start').value
        
        # 保存模型引用
        self.ripeness_model = ripeness_model
        self.classifier_model = classifier_model
        
        # 水果类型映射（FruitClassifier输出类别ID到水果类型）
        # 类型编号：1=辣椒, 2=南瓜, 3=洋葱, 4=番茄
        self.fruit_type_map = {
            'lajiao_ripe': 1,
            'lajiao_unripe': 1,
            'nangua_ripe': 2,
            'nangua_unripe': 2,
            'onion_ripe': 3,
            'onion_unripe': 3,
            'tomato_ripe': 4,
            'tomato_unripe': 4,
        }
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        
        # 发布检测结果
        self.detection_publisher_ = self.create_publisher(DetectionArray, 'fruit_detection', 10)
        self.process_image_publisher_ = self.create_publisher(Image, 'processed_image', 10)
        
        # 创建服务：控制检测开关
        self.control_service = self.create_service(
            ControlDetection,
            'control_detection',
            self.control_detection_callback)
        
        self.br = CvBridge()
        
        self.get_logger().info('=== Fruit Detect Node Started ===')
        self.get_logger().info(f'Ripeness model: {ripeness_model_path}')
        self.get_logger().info(f'Classifier model: {classifier_model_path}')
        self.get_logger().info(f'Ripeness confidence: {self.ripeness_conf}')
        self.get_logger().info(f'Classifier confidence: {self.classifier_conf}')
        self.get_logger().info(f'Detection enabled: {self.detection_enabled}')
        self.get_logger().info('Service "control_detection" ready')
    
    def control_detection_callback(self, request, response):
        """服务回调：控制检测开关
        
        Args:
            request: ControlDetection.Request，包含enable字段
            response: ControlDetection.Response
            
        Returns:
            response: 包含操作结果
        """
        self.detection_enabled = request.enable
        status = "enabled" if self.detection_enabled else "disabled"
        response.success = True
        response.message = f"Detection {status}"
        self.get_logger().info(f'Detection {status} by service call')
        return response
        

        
    def listener_callback(self, data):
        """图像回调函数 - 两阶段检测
        
        第一阶段：使用RipeLevel模型检测成熟度
        第二阶段：对成熟区域使用FruitClassifier模型检测种类
        
        Args:
            data: 图像消息
        """
        current_frame = self.br.imgmsg_to_cv2(data)
        height, width, _ = current_frame.shape
        
        # 获取图像中心坐标
        center_x = self.get_parameter('image_center_x').value
        center_y = self.get_parameter('image_center_y').value
        
        # 创建可视化图像（复制原图）
        annotated_frame = current_frame.copy()
        
        # 绘制中心十字线
        cv2.line(annotated_frame, (0, center_y), (width, center_y), (0, 0, 255), 1)
        cv2.line(annotated_frame, (center_x, 0), (center_x, height), (0, 0, 255), 1)
        
        # 创建DetectionArray消息
        detection_array = DetectionArray()
        info_lines = []
        
        # 如果检测未启用，只发布原图
        if not self.detection_enabled:
            status_text = "Detection DISABLED"
            cv2.putText(annotated_frame, status_text, 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            self.process_image_publisher_.publish(self.br.cv2_to_imgmsg(annotated_frame, 'rgb8'))
            return
        
        # ===== 第一阶段：成熟度检测 =====
        ripeness_results = self.ripeness_model(
            current_frame, 
            conf=self.ripeness_conf, 
            verbose=False
        )[0]
        
        ripe_boxes = []  # 存储成熟水果的边界框
        
        if ripeness_results.boxes:
            for box in ripeness_results.boxes:
                cls_id = int(box.cls[0])
                conf = box.conf[0].item()
                class_name = self.ripeness_model.names[cls_id]
                
                # 只处理成熟的水果
                if class_name == 'ripe':
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    ripe_boxes.append((x1, y1, x2, y2, conf))
                    
                    # 在图像上绘制成熟度检测框（绿色）
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_frame, f'ripe {conf:.2f}', 
                               (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    # 未成熟的用灰色标注（可选）
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (128, 128, 128), 1)
                    cv2.putText(annotated_frame, f'unripe {conf:.2f}', 
                               (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
            
            self.get_logger().debug(f'Stage 1: Found {len(ripe_boxes)} ripe fruits')
        
        # ===== 第二阶段：对成熟水果进行种类检测 =====
        if ripe_boxes:
            # 对整图进行种类检测（也可以只检测ripe_boxes区域以进一步节省资源）
            classifier_results = self.classifier_model(
                current_frame,
                conf=self.classifier_conf,
                verbose=False
            )[0]
            
            if classifier_results.boxes:
                for box in classifier_results.boxes:
                    cls_id = int(box.cls[0])
                    conf = box.conf[0].item()
                    class_name = self.classifier_model.names[cls_id]
                    
                    # 只处理成熟的类别（过滤掉_unripe）
                    if not class_name.endswith('_ripe'):
                        continue
                    
                    # 计算边界框中心
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    box_center_x = (x1 + x2) // 2
                    box_center_y = (y1 + y2) // 2
                    
                    # 检查这个检测框是否与任何成熟区域重叠
                    is_in_ripe_region = self._check_overlap(
                        (x1, y1, x2, y2), ripe_boxes
                    )
                    
                    if not is_in_ripe_region:
                        continue  # 不在成熟区域内，跳过
                    
                    # 计算相对于图像中心的偏移量
                    offset_x = float(box_center_x - center_x)
                    offset_y = float(box_center_y - center_y)
                    
                    # 获取水果类型编号
                    fruit_type = self.fruit_type_map.get(class_name, 0)
                    
                    # 创建FruitInfo消息
                    fruit_info = FruitInfo()
                    fruit_info.type = fruit_type
                    fruit_info.offset_x = offset_x
                    fruit_info.offset_y = offset_y
                    
                    detection_array.fruits.append(fruit_info)
                    
                    # 可视化标注 - 绘制种类检测框（蓝色）
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    cv2.drawMarker(annotated_frame, (box_center_x, box_center_y), 
                                  (255, 0, 0), markerType=cv2.MARKER_CROSS, 
                                  markerSize=20, thickness=2)
                    
                    # 显示水果名称（去掉_ripe后缀）
                    display_name = class_name.replace('_ripe', '')
                    info_lines.append(f'{display_name}: ({offset_x:.0f}, {offset_y:.0f})')
                    
                    self.get_logger().debug(
                        f'Stage 2: Detected {display_name} (type={fruit_type}) '
                        f'conf={conf:.2f}, offset=({offset_x:.1f}, {offset_y:.1f})'
                    )
        
        # 在图像左上角显示检测信息
        y_offset = 30
        if info_lines:
            for line in info_lines:
                cv2.putText(annotated_frame, line, 
                           (10, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                y_offset += 25
            
            # 发布检测结果
            self.detection_publisher_.publish(detection_array)
            self.get_logger().debug(f'Published {len(detection_array.fruits)} ripe fruit(s)')
        else:
            cv2.putText(annotated_frame, "No ripe fruits detected", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            self.get_logger().debug('No ripe fruits detected in stage 2')
        
        # 发布可视化图像
        self.process_image_publisher_.publish(self.br.cv2_to_imgmsg(annotated_frame, 'rgb8'))
    
    def _check_overlap(self, box, ripe_boxes, iou_threshold=0.3):
        """检查检测框是否与成熟区域重叠
        
        Args:
            box: (x1, y1, x2, y2) 待检查的框
            ripe_boxes: 成熟区域框列表
            iou_threshold: IoU阈值
            
        Returns:
            bool: 是否有足够的重叠
        """
        x1, y1, x2, y2 = box
        box_area = (x2 - x1) * (y2 - y1)
        
        for ripe_box in ripe_boxes:
            rx1, ry1, rx2, ry2, _ = ripe_box
            
            # 计算交集
            inter_x1 = max(x1, rx1)
            inter_y1 = max(y1, ry1)
            inter_x2 = min(x2, rx2)
            inter_y2 = min(y2, ry2)
            
            if inter_x2 > inter_x1 and inter_y2 > inter_y1:
                inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
                ripe_area = (rx2 - rx1) * (ry2 - ry1)
                
                # 计算IoU
                iou = inter_area / (box_area + ripe_area - inter_area)
                
                if iou >= iou_threshold:
                    return True
        
        return False
            

            
def main(args=None):
    rclpy.init(args=args)

    fruit_detect_node = FruitDetect()

    try:
        rclpy.spin(fruit_detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        fruit_detect_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()