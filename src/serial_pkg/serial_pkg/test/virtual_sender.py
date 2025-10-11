import struct
import serial
import time
import rclpy
from rclpy.node import Node
from robot_control_interfaces.msg import SerialData
import math

        # 帧格式定义
FRAME_HEADER = 0xAA
FRAME_FOOTER = 0x55
SEND_FUNC_CODE = 0x02


class Virtual_serial_sender(Node):
    def __init__(self):
        super().__init__('virtual_serial_sender')
        self.subscription = self.create_subscription(
            SerialData,
            'control_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def listener_callback(self, msg):
        global linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6
        linear_x = msg.linear_x
        angular_z = msg.angular_z
        servo1 = msg.servo1
        servo2 = msg.servo2
        servo3 = msg.servo3
        servo4 = msg.servo4
        servo5 = msg.servo5
        servo6 = msg.servo6

        ser = self.open_serial_port('/dev/ttyUSB0', 115200, 0.1)

        self.get_logger().info(f"Received data - linear_x: {linear_x}, angular_z: {angular_z}, servo1: {servo1}, servo2: {servo2}, servo3: {servo3}, servo4: {servo4}, servo5: {servo5}, servo6: {servo6}")
        self._prepare_send_frame(linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6, ser)
        
        
    def CalculateChecksum(self, data):

        checksum = sum(data) & 0xFF
        return checksum

        
    def _prepare_send_frame(self,linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6, ser):
        

        tx_buffer = bytearray()
        tx_buffer.append(FRAME_HEADER)

        # 功能码
        tx_buffer.append(SEND_FUNC_CODE)

        tx_buffer.append(0x14)
            
        # 打包数据（小端模式）：linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6
        tx_buffer.extend(struct.pack('<ff', linear_x, angular_z))
        tx_buffer.extend(struct.pack('<HHHHHH', servo1, servo2, servo3, servo4, servo5, servo6))

        print(f"Data length: {len(tx_buffer)+3} bytes")  # +3 for checksum and footer
        print(f"speed_x: {linear_x}, angular_z: {angular_z}")
        print(f"servo1: {servo1}, servo2: {servo2}, servo3: {servo3}, servo4: {servo4}, servo5: {servo5}, servo6: {servo6}")
            
        # 计算校验和：从功能码开始的所有数据 (功能码 + 数据长度 + payload = 22字节)
        data_to_checksum = tx_buffer[1:]
        checksum = self.CalculateChecksum(data_to_checksum)
        tx_buffer.append(checksum)
            
        # 帧尾
        tx_buffer.append(FRAME_FOOTER)
        
        self.get_logger().info(f"Prepared frame: {tx_buffer.hex()}")
        self.send_data(ser, tx_buffer)

    
        return tx_buffer
       
    def open_serial_port(self,port='/dev/ttyUSB0', baudrate=115200, timeout=0.1):
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"串口 {port} 已打开，波特率 {baudrate}")
            return ser
        except serial.SerialException as e:
            print(f"无法打开串口 {port}: {e}")
            return None
        
        
    def send_data(self, ser, data):
        if ser and ser.is_open:
            ser.write(data)
            print(f"发送数据: {data.hex()}")
        else:
            print("串口未打开，无法发送数据")

    # def timer_callback(self):
    #     pass 
        


def main():
    rclpy.init()
    node = Virtual_serial_sender()
    node.get_logger().info("虚拟串口发送节点已启动")
    rclpy.spin(node)
 
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':  
    main()