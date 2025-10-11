import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import threading

from serial_pkg.serial_controller import SerialServer
from robot_control_interfaces.msg import SerialData

class SerialReceiverThread(threading.Thread):
    """
    专用线程类，用于处理串口接收数据，避免阻塞ROS2主循环
    """
    def __init__(self, node):
        threading.Thread.__init__(self)
        self.node = node
        self.daemon = True  # 设置为守护线程，随主线程退出
        self.running = True

    def run(self):
        """线程主循环"""
        while rclpy.ok() and self.running:
            try:
                # 调用节点的接收方法
                self.node.receive_data()
                # 适当延时，避免CPU占用过高
                time.sleep(0.01)
            except Exception as e:
                self.node.get_logger().error(f'串口接收线程出错: {e}')
                # 出错时增加延时，避免快速循环
                time.sleep(0.1)

    def stop(self):
        """停止线程"""
        self.running = False


def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = SerialServer(port='/dev/ttyUSB0', baudrate=115200, type='receiver')
    
    # 创建并启动串口接收线程
    serial_thread = SerialReceiverThread(node)
    serial_thread.start()
    
    # 使用多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # 创建一个线程来运行 executor.spin()，避免阻塞主线程
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        node.get_logger().info('串口接收节点已启动，按 Ctrl+C 终止')
        # 主线程在这里等待，直到接收到 KeyboardInterrupt
        serial_thread.join()
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，开始关闭节点...')
    finally:
        # 停止线程
        serial_thread.stop()
        # 等待线程真正结束
        if serial_thread.is_alive():
            serial_thread.join(timeout=1.0)
        
        # 关闭执行器
        executor.shutdown()
        
        # 清理节点
        node.destroy_node()
        rclpy.shutdown()
        
        # 确保执行器线程也已结束
        if executor_thread.is_alive():
            executor_thread.join()