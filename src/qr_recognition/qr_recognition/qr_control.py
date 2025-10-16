from pyzbar.pyzbar import decode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_control_interfaces.srv import ControlDetection
from cv_bridge import CvBridge
import cv2
import numpy as np

class QRRecognitionNode(Node):
    """二维码识别节点
    
    功能：
        - 订阅相机图像话题进行二维码检测
        - 通过Service控制二维码扫描的开启/关闭
    """
    
    def __init__(self):
        super().__init__('qr_recognition_node')
        self.get_logger().info('QR Recognition Node has been started.')
        
        
        # 扫码开关标志
        self.scanning_enabled = False
        
        
        # 当前二维码类型: 'B'=水果列表, 'C'=数字序列或单个水果名
        self.qr_type = 'C'  # 默认为C类型
        
        
        # CV Bridge用于ROS图像消息和OpenCV图像格式转换
        self.bridge = CvBridge()
        
        
        # Initialize camera and other necessary components here
        self.subscription = self.create_subscription(
            Image,
            'camera/color/image_raw',
            self.process_frame,
            10)
        
        # Initialize service server - 控制二维码扫描开关
        self.control_service = self.create_service(
            ControlDetection,
            'qr_scan_control',
            self.control_scan_callback
        )
        
        self.get_logger().info('QR scan control service created: /qr_scan_control')
        
    def control_scan_callback(self, request, response):
        """Service回调函数：控制二维码扫描开关
        
        Args:
            request: ControlDetection.Request，包含enable和qr_type字段
            response: ControlDetection.Response，包含success和message字段
            
        Returns:
            response: 操作结果
        """
        self.scanning_enabled = request.enable
        
        # 设置二维码类型（如果提供）
        if request.qr_type in ['B', 'C']:
            self.qr_type = request.qr_type
        elif request.qr_type:
            response.success = False
            response.message = f'无效的二维码类型: {request.qr_type}，必须是 B 或 C'
            self.get_logger().warn(response.message)
            return response
        
        if request.enable:
            response.success = True
            response.message = f'QR scanning enabled (Type: {self.qr_type})'
            self.get_logger().info(f'二维码扫描已开启 (类型: {self.qr_type})')
        else:
            response.success = True
            response.message = 'QR scanning disabled'
            self.get_logger().info('二维码扫描已关闭')
        
        return response
        
    def process_frame(self, msg):
        """图像处理回调函数：检测二维码
        
        Args:
            msg: ROS 2 Image消息
        """
        # 只有在扫描开关开启时才处理图像
        if not self.scanning_enabled:
            return
        
        try:
            # 将ROS Image消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 转换为灰度图
            gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 解码二维码
            barcodes = decode(gray_img)

            # 遍历解码结果
            for barcode in barcodes:
                # 提取数据
                barcode_data = barcode.data.decode('utf-8')
                barcode_type = barcode.type
                
                self.get_logger().info(f"检测到二维码 [{barcode_type}]: {barcode_data}")
                
                # 处理二维码数据
                self.process_qr_data(barcode_data)
                
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {str(e)}')
    
    def process_qr_data(self, qr_data):
        """处理二维码数据
        
        Args:
            qr_data: 二维码解码后的字符串数据
            
        根据self.qr_type选择不同的处理方式：
            - B类型: 水果名称列表，每行一个 (如"苹果\n梨子\n梨子\n苹果")
            - C类型: 数字序列 (如"4,3,1,10,8,9,2,11") 或单个水果名称
        """
        if self.qr_type == 'B':
            # B类型：水果列表，每行一个
            self.process_type_b_qr(qr_data)
        else:
            # C类型：数字序列或单个水果名
            self.process_type_c_qr(qr_data)
    
    def process_type_b_qr(self, qr_data):
        """处理B类型二维码：水果名称列表
        
        Args:
            qr_data: 多行水果名称，如 "苹果\n梨子\n梨子\n苹果\n苹果\n梨子\n苹果\n苹果"
        """
        try:
            # 按行分割，去除空行和空格
            fruits = [line.strip() for line in qr_data.split('\n') if line.strip()]
            
            if fruits:
                self.get_logger().info(f'识别到B类型二维码 - 水果列表 (共{len(fruits)}个):')
                for i, fruit in enumerate(fruits, 1):
                    self.get_logger().info(f'  {i}. {fruit}')
                
                # 这里可以添加对水果列表的处理逻辑
                # 例如：发布话题、调用Service、存储到变量等
            else:
                self.get_logger().warn('B类型二维码为空')
                
        except Exception as e:
            self.get_logger().error(f'解析B类型二维码失败: {str(e)}')
    
    def process_type_c_qr(self, qr_data):
        """处理C类型二维码：数字序列或单个水果名称
        
        Args:
            qr_data: 数字序列 (如"4,3,1,10,8,9,2,11") 或单个水果名称
        """
        # 判断是水果名称还是数字序列
        if ',' in qr_data:
            # 数字序列
            try:
                numbers = [int(x.strip()) for x in qr_data.split(',')]
                self.get_logger().info(f'识别到C类型二维码 - 数字序列: {numbers}')
                # 这里可以添加对数字序列的处理逻辑
            except ValueError:
                self.get_logger().warn(f'无法解析数字序列: {qr_data}')
        else:
            # 单个水果名称
            fruit_name = qr_data.strip()
            self.get_logger().info(f'识别到C类型二维码 - 水果: {fruit_name}')
            # 这里可以添加对水果名称的处理逻辑


def main(args=None):
    """节点主入口函数"""
    rclpy.init(args=args)
    
    qr_recognition_node = QRRecognitionNode()
    
    try:
        rclpy.spin(qr_recognition_node)
    except KeyboardInterrupt:
        pass
    finally:
        qr_recognition_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()