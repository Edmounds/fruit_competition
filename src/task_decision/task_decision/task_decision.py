import rclpy
from rclpy.node import Node

class TaskDecision(Node):
    def __init__(self):
        super().__init__('task_decision')
        self.get_logger().info("Task Decision Node has been started.")



def main(args=None):
    rclpy.init(args=args)
    node = TaskDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()