import rclpy
from rclpy.node import Node

class SpeakerController(Node):
    def __init__(self):
        super().__init__('speaker_controller')
        self.get_logger().info("Speaker Controller Node has been started.")

    def play_sound(self, content):
        self.get_logger().info(f"Playing sound: {content}")


def main(args=None):
    rclpy.init(args=args)
    node = SpeakerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()