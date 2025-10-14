import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robot_control_interfaces.msg import SerialData
from serial_pkg.serial_controller import SerialServer


def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = SerialServer(port='/dev/serial_ch340', baudrate=115200, type='sender')
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info('串口发送节点已启动，按 Ctrl+C 终止')
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 只清理节点资源，不调用 rclpy.shutdown()
        # launch 系统会统一管理 ROS 2 上下文的关闭
        node.destroy_node()

        
if __name__ == '__main__':
    main()