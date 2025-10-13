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
from camera_controller.get_depth import DepthGetter
#不使用openvino进行加速

models_path = get_package_share_directory('camera_controller') + '/models/' + 'best_openvino_model/'
ov_model = YOLO(models_path)

class FruitDetect(Node):
    """水果检测节点
    
    使用YOLO模型进行水果检测，发布检测结果和目标位置
    """
    
    def __init__(self):
        super().__init__('fruit_detect')
        
        # 声明置信度阈值参数，默认值0.5
        self.declare_parameter('confidence_threshold', 0.5)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'fruit_detection', 10)
        self.br = CvBridge()
        
        self.process_image_publisher_ = self.create_publisher(Image, 'processed_image', 10)
        
        # 发布检测到的水果目标2D坐标
        self.target_point_publisher_ = self.create_publisher(Point, 'fruit_target_2d', 10)
        
        self.get_logger().info(f'Fruit Detect Node started with confidence threshold: {self.confidence_threshold}')
        
        self.fruit_3d_subscriber = self.create_subscription(
            Point,
            'fruit_target_3d',
            self.fruit_3d_callback,
            10)
        
        # self.target_offset_publisher_ = self.create_publisher(Point, 'fruit_target_offset', 10)
        
        self.target_position = [0.0, 0.0, 0.0] # x, y, z
        
    def fruit_3d_callback(self, msg):
        """3D目标点回调函数
        
        接收3D目标点并存储
        
        Args:
            msg (Point): 3D目标点消息
        """
        self.target_position[0] = msg.x
        self.target_position[1] = msg.y
        self.target_position[2] = msg.z
        # self.get_logger().info(f'Received 3D target position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')

        
    def listener_callback(self, data):
        """图像回调函数
        
        处理接收到的图像，进行YOLO检测并发布结果
        
        Args:
            data: 图像消息
        """
        current_frame = self.br.imgmsg_to_cv2(data)
        # 使用置信度阈值进行YOLO推理
        results = ov_model(current_frame, conf=self.confidence_threshold, verbose=False)[0]
        
        annotated_frame = results.plot()
        
        
        height, width, _ = annotated_frame.shape

        # self.get_logger().info(f'Image dimensions: width={width}, height={height}')
        center_x, center_y = width // 2, height // 2
        
        # cv2.drawMarker(annotated_frame, (center_x, center_y), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
        
        #draw a red line across the center
        cv2.line(annotated_frame, (0, center_y), (width, center_y), (0, 0, 255), 1)
        cv2.line(annotated_frame, (center_x, 0), (center_x, height), (0, 0, 255), 1)

        
        detection_info = String()
        if results.boxes:
            for box in results.boxes:
                cls_id = int(box.cls[0])
                conf = box.conf[0].item()
                class_name = ov_model.names[cls_id]
                detection_info.data += f"{class_name}:{conf:.2f} "
                
                # ===========================================================draw marker at the center of the bounding box====================
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                box_center_x = (x1 + x2) // 2
                box_center_y = (y1 + y2) // 2
                cv2.drawMarker(annotated_frame, (box_center_x, box_center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                
                #write target position at the top-left corner of the image
                cv2.putText(annotated_frame, f'({box_center_x},{box_center_y})', (box_center_x + 10, box_center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                #calculate the offset from the image center
                # offset_x = box_center_x - center_x
                # offset_y = box_center_y - center_y

                # self.target_offset_publisher_.publish(Point(x=float(offset_x), y=float(offset_y), z=0.0))

                self.get_logger().info(f'Detected {class_name} with confidence {conf:.2f} at ({box_center_x}, {box_center_y})')
                
                self.target_point_publisher_.publish(Point(x=float(box_center_x), y=float(box_center_y), z=0.0))

            self.publisher_.publish(detection_info)
            self.get_logger().info(f'Published: {detection_info.data.strip()}')
        else:
            self.get_logger().info('No fruits detected.')
        
        # Publish the annotated image
        self.process_image_publisher_.publish(self.br.cv2_to_imgmsg(annotated_frame, 'rgb8'))
            
    
            
            
            
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