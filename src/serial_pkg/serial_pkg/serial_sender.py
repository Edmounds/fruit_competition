import rclpy
from rclpy.node import Node
import serial
import struct

from robot_control_interfaces.msg import SerialData
from serial_pkg.serial_controller import SerialNode

class SerialSender(Node):
    def __init__(self):
        super().__init__('serial_sender')
        self.serial_node = SerialNode(port='/dev/ttyUSB0', baudrate=115200)
        self.get_logger().info(f'Serial port {self.serial_node.ser.port} opened with baudrate {self.serial_node.ser.baudrate}')



def main(args=None):
    rclpy.init(args=args)
    sender = SerialSender()
    
    # 示例发送 (根据文档样例)
    sender.pack_and_send(0.2, 0.0, 375, 500, 500, 500, 1)
    
    rclpy.spin_once(sender, timeout_sec=0.1)
    sender.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
