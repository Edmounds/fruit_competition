import rclpy
from rclpy.node import Node
from robot_control_interfaces.msg import SerialData


class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        self.get_logger().info('Manual Control Node has been started.')

        # 在这里添加你的手动控制逻辑，例如订阅话题、发布命令等


        