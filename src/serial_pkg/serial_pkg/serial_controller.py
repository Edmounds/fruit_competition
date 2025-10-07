import rclpy
from rclpy.node import Node
import serial
import struct

from robot_control_interface.msg import SerialData


class SerialServer(Node):
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        super().__init__('serial_server')
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f'Serial port {port} opened with baudrate {baudrate}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            SerialData,
            'serial_data',
            self.pack_and_send_data,
            10
        )

    def pack_and_send_data(self, msg):
        header = 0xAA
        func_code = 0x02
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        servo1 = msg.servo1
        servo2 = msg.servo2
        servo3 = msg.servo3
        servo4 = msg.servo4
        servo5 = msg.servo5
        data_len = 0x12
        footer = 0x55

        try:
            payload = struct.pack('<ffHHHHH', linear_x, angular_z, servo1, servo2, servo3, servo4, servo5)
            checksum_data = bytes([func_code, data_len]) + payload
            checksum = sum(checksum_data) & 0xFF
            frame = bytes([header]) + checksum_data + bytes([checksum, footer])
            self.ser.write(frame)
        except Exception as e:
            self.get_logger().error(f"Failed to pack and send data: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialServer(port='/dev/ttyUSB0', baudrate=115200)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()