# openvino_ros_interfaces/openvino_ros_interfaces/image_classifier_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import openvino.runtime as ov
from ament_index_python.packages import get_package_share_directory
import os

class FruitClassifier(Node):
    def __init__(self):
        super().__init__('fruit_classifier')

        # 声明参数，使得模型路径可以在启动时被修改
        self.declare_parameter('model_path', 'models/fruit.onnx')
        self.declare_parameter('labels_path', 'models/labels.txt')
        self.declare_parameter('device', 'CPU')

        # 获取参数值
        pkg_share_dir = get_package_share_directory('openvino_ros_interfaces')
        model_path = os.path.join(pkg_share_dir, self.get_parameter('model_path').get_parameter_value().string_value)
        labels_path = os.path.join(pkg_share_dir, self.get_parameter('labels_path').get_parameter_value().string_value)
        device = self.get_parameter('device').get_parameter_value().string_value
        
        # 初始化CV Bridge
        self.bridge = CvBridge()

        # 初始化OpenVINO
        self.get_logger().info(f"正在加载模型: {model_path} 到设备: {device}")
        self.core = ov.Core()
        self.model = self.core.read_model(model_path)
        self.compiled_model = self.core.compile_model(self.model, device)
        self.infer_request = self.compiled_model.create_infer_request()
        
        # 获取模型的输入输出层信息
        self.input_layer = self.compiled_model.input(0)
        self.output_layer = self.compiled_model.output(0)
        self.N, self.C, self.H, self.W = self.input_layer.shape
        self.get_logger().info(f"模型输入尺寸: N={self.N}, C={self.C}, H={self.H}, W={self.W}")


        # 加载标签
        with open(labels_path, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]
        self.get_logger().info(f"成功加载 {len(self.labels)} 个标签。")

        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # 订阅的话题
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/classification_result', 10) # 发布的话题
        
        self.get_logger().info("节点初始化完成，等待图像消息...")

    def image_callback(self, msg):
        try:
            # 1. 将ROS Image消息转换为OpenCV图像 (BGR格式)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge 转换失败: {e}")
            return

        # 2. 图像预处理
        #    - 调整尺寸到模型输入大小
        #    - 转换颜色通道 BGR -> RGB
        #    - 归一化
        #    - 调整维度 HWC -> CHW -> NCHW
        resized_image = cv2.resize(cv_image, (self.W, self.H))
        rgb_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB)
        
        # 归一化 (根据ImageNet预训练模型的常用值)
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        normalized_image = (rgb_image / 255.0 - mean) / std
        
        # HWC -> CHW
        transposed_image = normalized_image.transpose(2, 0, 1)
        # CHW -> NCHW (增加一个batch维度)
        input_tensor = np.expand_dims(transposed_image, axis=0).astype(np.float32)

        # 3. 执行推理
        try:
            self.infer_request.infer({self.input_layer.any_name: input_tensor})
            results = self.infer_request.get_output_tensor(self.output_layer.index).data
        except Exception as e:
            self.get_logger().error(f"推理失败: {e}")
            return
            
        # 4. 后处理
        #    - 获取最可能的类别索引
        #    - 查找对应的标签
        results = np.squeeze(results) # 从(1, 1000) -> (1000,)
        prediction_index = np.argmax(results)
        confidence = results[prediction_index]
        predicted_label = self.labels[prediction_index]

        # 5. 发布结果
        result_msg = String()
        result_msg.data = f'类别: {predicted_label}, 置信度: {confidence:.2f}'
        self.publisher.publish(result_msg)
        
        # (可选) 在窗口中显示并打印结果
        self.get_logger().info(result_msg.data)
        cv2.putText(cv_image, result_msg.data, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Detection Result", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FruitClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()