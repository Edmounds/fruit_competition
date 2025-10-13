import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from ultralytics import YOLO # pyright: ignore
from ament_index_python.packages import get_package_share_directory

#不使用openvino进行加速

models_path = get_package_share_directory('camera_controller') + '/models/' + 'best_openvino_model/'
ov_model = YOLO(models_path)

class FruitDetect(Node):
    def __init__(self):
        super().__init__('fruit_detect')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'fruit_detection', 10)
        self.br = CvBridge()
        self.get_logger().info('Fruit Detect Node has been started.')
        
        self.process_image_publisher_ = self.create_publisher(Image, 'processed_image', 10)
        
        
        #这里不应该是string,这里应该自定义消息类型
        self.target_point_publisher_ = self.create_publisher(String, 'target_point', 10)

        
        
    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        results = ov_model(current_frame)[0]
        
        annotated_frame = results.plot()
        
        
        height, width, _ = annotated_frame.shape

        self.get_logger().info(f'Image dimensions: width={width}, height={height}')
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
                
                # draw marker at the center of the bounding box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                box_center_x = (x1 + x2) // 2
                box_center_y = (y1 + y2) // 2
                cv2.drawMarker(annotated_frame, (box_center_x, box_center_y), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                
                # Publish the processed image
                self.process_image_publisher_.publish(self.br.cv2_to_imgmsg(annotated_frame, 'rgb8'))

                self.get_logger().info(f'Detected {class_name} with confidence {conf:.2f} at ({box_center_x}, {box_center_y})')

            self.publisher_.publish(detection_info)
            self.get_logger().info(f'Published: {detection_info.data.strip()}')
        else:
            self.get_logger().info('No fruits detected.')
            
    
            
            
            
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