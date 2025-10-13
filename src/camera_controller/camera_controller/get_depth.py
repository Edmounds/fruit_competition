import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np


class DepthGetter(Node):
    """
    深度获取节点
    
    订阅深度图像和2D目标点，计算并发布3D坐标
    """
    def __init__(self):
        super().__init__('depth_getter')
        
        # 配置QoS以匹配相机发布者
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 在这里初始化订阅者和发布者
        # 例如，订阅深度图像话题
        self.depth_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile)
        
        self.fruit_2d_subscriber = self.create_subscription(
            Point,
            'fruit_target_2d',
            self.fruit_2d_callback,
            10)
        
        self.fruit_3d_publisher_ = self.create_publisher(Point, 'fruit_target_3d', 10)
        
        # CV Bridge用于处理ROS图像消息
        self.bridge = CvBridge()
        
        # 缓存最新的深度图像
        self.latest_depth_image = None
        
        # 2D目标坐标（像素坐标）
        self.x = 0
        self.y = 0
        
        # 标记是否有有效的2D目标
        self.has_valid_target = False
        
        # 标记是否已经记录过深度图信息（只记录一次）
        self.depth_info_logged = False
        
        self.get_logger().info('Depth Getter Node has been started.')
        self.get_logger().info('Subscribed to /camera/depth/image_raw with RELIABLE QoS')
        self.get_logger().info('Waiting for depth image (OB_FORMAT_Y16 -> 16UC1)...')


    def depth_callback(self, msg: Image):
        """
        深度图像回调函数
        
        Args:
            msg: 深度图像消息
        """
        # 第一次接收到深度图像时记录信息
        if not self.depth_info_logged:
            self.get_logger().info(
                f'Received first depth image: '
                f'size={msg.width}x{msg.height}, '
                f'encoding={msg.encoding}, '
                f'step={msg.step}, '
                f'is_bigendian={msg.is_bigendian}')
            self.depth_info_logged = True
        
        # 缓存最新的深度图像
        self.latest_depth_image = msg
        
        # 只有在有有效2D目标时才处理
        if not self.has_valid_target:
            return
        
        # 从深度图中对应的xy地方取深度值
        depth_value = self.get_depth_value(msg, self.x, self.y)
        
        if depth_value is None:
            self.get_logger().warn(f'Invalid depth at ({self.x}, {self.y})')
            return
        
        # self.get_logger().info(f'Depth at ({self.x}, {self.y}): {depth_value:.3f} meters')
        
        # 发布3D点
        point_3d = Point()
        point_3d.x = float(self.x)
        point_3d.y = float(self.y)
        point_3d.z = depth_value
        self.fruit_3d_publisher_.publish(point_3d)
        # self.get_logger().info(f'Published 3D point: ({point_3d.x}, {point_3d.y}, {point_3d.z:.3f})')

    def fruit_2d_callback(self, msg: Point):
        """
        2D水果目标点回调函数
        
        Args:
            msg: 2D目标点消息（像素坐标）
        """
        self.x = int(msg.x)
        self.y = int(msg.y)
        self.has_valid_target = True
        
        # self.get_logger().info(f'Received 2D point: ({self.x}, {self.y})')
        
        # 如果已经有缓存的深度图像，立即处理
        if self.latest_depth_image is not None:
            depth_value = self.get_depth_value(self.latest_depth_image, self.x, self.y)
            
            if depth_value is not None:
                # self.get_logger().info(f'Depth at ({self.x}, {self.y}): {depth_value:.3f} meters')
                
                # 发布3D点
                point_3d = Point()
                point_3d.x = float(self.x)
                point_3d.y = float(self.y)
                point_3d.z = depth_value
                self.fruit_3d_publisher_.publish(point_3d)
                # self.get_logger().info(f'Published 3D point: ({point_3d.x}, {point_3d.y}, {point_3d.z:.3f})')
            else:
                self.get_logger().warn(f'Invalid depth at ({self.x}, {self.y})')
    
    def get_depth_value(self, depth_image: Image, x: int, y: int):
        """
        从深度图像中获取指定像素点的深度值（以米为单位）
        
        适用于 Orbbec 相机的 OB_FORMAT_Y16 格式（16位灰度深度图）
        
        Args:
            depth_image: ROS 深度图像消息
            x: 像素点的 x 坐标
            y: 像素点的 y 坐标
        
        Returns:
            深度值（以米为单位），如果无效则返回 None
        """
        try:
            # 确保坐标在图像范围内
            if x < 0 or x >= depth_image.width or y < 0 or y >= depth_image.height:
                self.get_logger().error(
                    f'Coordinates ({x}, {y}) are out of bounds for depth image '
                    f'size ({depth_image.width}x{depth_image.height})')
                return None
            
            # OB_FORMAT_Y16 对应 ROS 的 16UC1 编码
            # 使用 cv_bridge 转换图像为 numpy 数组
            try:
                # 保持原始编码，不进行转换
                depth_array = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
                
                # 获取指定位置的深度值（单位：毫米）
                depth_value_mm = depth_array[y, x]
                
                # 检查深度值是否有效
                # 0 表示无效深度，通常 > 10000 也可能是异常值
                if depth_value_mm == 0:
                    self.get_logger().debug(f'Zero depth value at ({x}, {y})')
                    return None
                
                if depth_value_mm > 10000:  # 大于10米可能是噪声
                    self.get_logger().warn(f'Depth value too large at ({x}, {y}): {depth_value_mm} mm')
                    return None
                
                # 转换为米
                depth_m = float(depth_value_mm) / 1000.0
                
                self.get_logger().debug(
                    f'Raw depth: {depth_value_mm} mm, converted: {depth_m:.3f} m, '
                    f'encoding: {depth_image.encoding}')
                
                return depth_m
                
            except Exception as e:
                self.get_logger().error(f'Error converting depth image with cv_bridge: {str(e)}')
                self.get_logger().error(f'Image encoding: {depth_image.encoding}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'Error in get_depth_value: {str(e)}')
            return None
    
    
def main(args=None):
    rclpy.init(args=args)
    depth_getter_node = DepthGetter()
    
    try:
        rclpy.spin(depth_getter_node)
    except KeyboardInterrupt:
        pass
    finally:
        depth_getter_node.destroy_node()
        rclpy.shutdown()