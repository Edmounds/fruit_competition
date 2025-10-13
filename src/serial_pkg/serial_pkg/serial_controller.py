import rclpy
from rclpy.node import Node
import serial
import struct
from typing import Optional
from robot_control_interfaces.msg import SerialData

    
class SerialServer(Node):
    def __init__(self, port='/dev/serial_ch340', baudrate=115200, type='sender'):

        if type == 'sender':
            node_name = 'serial_sender'

        if type == 'receiver':
            node_name = 'serial_receiver'
        else:
            node_name = 'serial_sender'

        super().__init__(node_name=node_name)


        # 存储串口参数，用于可能的重连
        self.port = port
        self.baudrate = baudrate
        self.type = type
        self.ser: Optional[serial.Serial] = None
        
        # 重连参数
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 1.0  # 重连延迟，单位：秒
        self.current_reconnect_attempt = 0
        
        # 连接串口
        self._connect_serial()
        
        # 帧格式定义
        self.FRAME_HEADER = 0xAA
        self.FRAME_FOOTER = 0x55
        self.FEEDBACK_FUNC_CODE = 0x03
        self.SEND_FUNC_CODE = 0x02
        self.MIN_FRAME_LENGTH = 25  # 最小完整帧长度
        self.EXPECTED_PAYLOAD_SIZE = 0x14  # 20 字节数据段
        self.MAX_BUFFER_SIZE = 1024

        
        # 缓冲区
        self.buffer = bytearray()
        
        # 超时设置（单位：秒）
        self.read_timeout = 0.5
        self.last_receive_time = self.get_clock().now()

        if type == 'sender':
            # 订阅控制数据话题
            self.arm_subscription = self.create_subscription(
                SerialData,
                'control_data',
                self.send_data,
                10
            )
            self.arm_subscription  # 防止未使用警告

        elif type == 'receiver':
            # 发布反馈数据话题
            self.feedback_publisher = self.create_publisher(
                SerialData,
                'feedback_data',
                10
            )
            # 创建定时器，定期检查串口数据（10ms周期）
            self.receive_timer = self.create_timer(0.01, self.receive_data)

        # 初始化缓冲区大小限制
        self.MAX_BUFFER_SIZE = 1024  # 最大缓冲区大小

    def receive_data(self) -> None:
        """
        接收串口数据并解析（ROS回调函数接口）
        
        接收到的数据会被添加到缓冲区并由 _process_buffer 方法处理
        """
        self._receive_frame()

    def _receive_frame(self) -> bool:
        """
        执行实际的串口数据接收和解析
        
        Returns:
            bool: 是否成功接收数据
        """
        # 检查连接状态
        if not self._check_connection():
            return False
        
        # 类型断言：_check_connection 返回 True 时，self.ser 必定不为 None
        assert self.ser is not None
            
        try:
            # 检查是否有数据可读
            if self.ser.in_waiting > 0:
                # 读取所有可用数据
                data = self.ser.read(self.ser.in_waiting)
                self.get_logger().info(f'接收原始数据: {data.hex()}')
                self.buffer.extend(data)
                
                # 更新最后接收时间
                self.last_receive_time = self.get_clock().now()
                
                # 检查缓冲区大小
                if len(self.buffer) > self.MAX_BUFFER_SIZE:
                    self.get_logger().warn(f'缓冲区溢出，清空: {len(self.buffer)} 字节')
                    self.buffer.clear()
                
                # 处理缓冲区中的数据
                self._process_buffer()
                return True
            else:
                # 检查是否超时（长时间没有收到数据）
                current_time = self.get_clock().now()
                time_diff = current_time - self.last_receive_time
                time_diff_sec = time_diff.nanoseconds / 1e9
                
                if time_diff_sec > self.read_timeout and self.type == 'receiver':
                    self.get_logger().warn(f'接收超时: {time_diff_sec:.1f}秒未收到数据')
                    
                return False
                
        except serial.SerialException as e:
            self.get_logger().error(f'串口通信错误: {e}')
            # 尝试重连
            return self._try_reconnect()
        except Exception as e:
            self.get_logger().error(f'接收数据时出错: {e}')
            return False
            


    
    def _process_buffer(self):
        """
        处理缓冲区中的数据，寻找完整的数据帧并解析
        
        此方法可处理不同功能码的数据帧，支持发送和接收操作
        """
        # 首先检查缓冲区是否有足够的数据构成一个最小的合法帧
        while len(self.buffer) >= self.MIN_FRAME_LENGTH:
            # 查找帧头
            if self.buffer[0] != self.FRAME_HEADER:
                # 帧头不匹配，丢弃第一个字节
                self.buffer.pop(0)
                continue
            
            # 检查功能码是否合法
            func_code = self.buffer[1]
            # self.get_logger().info(f'功能码: {func_code}')
            if func_code not in [self.FEEDBACK_FUNC_CODE, self.SEND_FUNC_CODE]:
                # 功能码不匹配，丢弃第一个字节
                self.buffer.pop(0)
                self.get_logger().error(f'功能码错误: 预期={self.FEEDBACK_FUNC_CODE}或{self.SEND_FUNC_CODE}, 实际={func_code}')
                continue
            # 获取数据长度
            data_len = self.buffer[2]
            # self.get_logger().info(f'数据长度: {data_len}')
            
            # 计算完整帧长度: 帧头(1) + 功能码(1) + 数据长度(1) + 数据(N) + 校验和(1) + 帧尾(1)
            frame_len = 1 + 1 + 1 + data_len + 1 + 1
            
            # 检查缓冲区是否有足够的数据构成一个完整帧
            if len(self.buffer) < frame_len:
                self.get_logger().error(f'缓冲区数据不足，等待更多数据: 当前长度={len(self.buffer)}, 需要长度={frame_len}')
                # 数据不完整，等待更多数据
                break
                
            # 检查帧尾
            if self.buffer[frame_len - 1] != self.FRAME_FOOTER:
                self.get_logger().error(f'帧尾错误: 预期={self.FRAME_FOOTER}, 实际={self.buffer[frame_len - 1]}')
                # 帧尾不匹配，丢弃第一个字节
                self.buffer.pop(0)
                continue
                
            # 验证校验和
            checksum_calc = self._calculate_checksum(self.buffer[1:frame_len - 2])
            self.get_logger().info(f'校验和: 计算值={checksum_calc}, 接收值={self.buffer[frame_len - 2]}')
            if checksum_calc != self.buffer[frame_len - 2]:
                self.get_logger().warn(f'校验和错误: 计算值={checksum_calc}, 接收值={self.buffer[frame_len - 2]}')
                self.buffer.pop(0)
                continue
            
            try:
                # 根据功能码处理数据帧
                if func_code == self.FEEDBACK_FUNC_CODE:
                    self._process_received_frame(self.buffer[:frame_len])
                # 对于其他功能码，可以在这里添加处理逻辑

                # 清除已处理的帧
                del self.buffer[:frame_len]
            except Exception as e:
                self.get_logger().error(f'解析数据帧时出错: {e}')
                self.buffer.pop(0)
                
    def _process_received_frame(self, frame):
        """
        处理接收到的完整数据帧（更新以支持正确规范）
        
        Args:
            frame: 完整的数据帧，包含帧头、功能码、数据等
        """
        # 确保是反馈功能码
        if frame[1] != self.FEEDBACK_FUNC_CODE:
            return
        
        data_len = frame[2]
        if data_len != self.EXPECTED_PAYLOAD_SIZE:
            self.get_logger().warn(f'反馈帧数据长度异常: 预期=20(0x14), 实际={data_len}(0x{data_len:02x})')
            return

        # 帧数据切片范围：从索引 3 开始，长度为 20 字节 (payload部分: 3 到 23)
        payload = frame[3:23]

        try:
            # 按照正确规范的顺序解包：linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6
            unpacked_data = struct.unpack('<ffHHHHHH', payload)
            
            # 创建 SerialData 消息
            msg = SerialData()
            
            # 按照解包顺序赋值给消息字段
            msg.linear_x = unpacked_data[0]
            msg.angular_z = unpacked_data[1]
            msg.servo1 = int(unpacked_data[2])  
            msg.servo2 = int(unpacked_data[3]) 
            msg.servo3 = int(unpacked_data[4]) 
            msg.servo4 = int(unpacked_data[5])  
            msg.servo5 = int(unpacked_data[6]) 
            msg.servo6 = int(unpacked_data[7]) 
            
            # 发布消息
            self.feedback_publisher.publish(msg)
            
            # 调试输出
            self.get_logger().info(
                f'收到反馈: linear_x={msg.linear_x:.2f}, angular_z={msg.angular_z:.2f}, '
                f'舵机反馈: [s1:{msg.servo1}, s2:{msg.servo2}, s3:{msg.servo3}, '
                f's4:{msg.servo4}, s5:{msg.servo5}], 开关门:{msg.servo6}'
            )
        except struct.error as e:
            self.get_logger().error(f'解包数据帧时出错: {e}, payload_len={len(payload)}')
        except Exception as e:
            self.get_logger().error(f'处理反馈帧时发生未知错误: {e}')

    
    #==================================================================================发送部分开始==================================================================================

    def _send_frame(self, serial_data: SerialData) -> bool:
        """
        执行实际的串口数据发送
        
        Args:
            serial_data: 包含线速度、角速度和舵机控制值的SerialData消息
            
        Returns:
            bool: 是否成功发送数据
        """
        # 检查连接状态
        if not self._check_connection():
            return False
        
        # 类型断言：_check_connection 返回 True 时，self.ser 必定不为 None
        assert self.ser is not None
            
        try:
            # 从消息中提取数据
            linear_x = serial_data.linear_x
            angular_z = serial_data.angular_z
            servo1 = int(serial_data.servo1)
            servo2 = int(serial_data.servo2)
            servo3 = int(serial_data.servo3)
            servo4 = int(serial_data.servo4)
            servo5 = int(serial_data.servo5)
            servo6 = int(serial_data.servo6)
            
            # 构建并发送帧
            frame = self._prepare_send_frame(linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6)
            
            bytes_written = self.ser.write(frame)
            
            if bytes_written != len(frame):
                self.get_logger().warn(f'数据发送不完整：预期发送 {len(frame)} 字节，实际发送 {bytes_written} 字节')
                return False
                
            self.get_logger().info(f'发送数据: linear_x={linear_x}, angular_z={angular_z}, '
                               f'舵机=[{servo1}, {servo2}, {servo3}, {servo4}, {servo5}], 开关门={servo6}')
            return True
            
        except serial.SerialTimeoutException:
            self.get_logger().error('发送数据超时')
            return False
        except serial.SerialException as e:
            self.get_logger().error(f'串口错误: {e}')
            # 尝试重连
            return self._try_reconnect()
        except Exception as e:
            self.get_logger().error(f"打包并发送数据时出错: {e}")
            return False
            
    def _prepare_send_frame(self, linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6):
        """
        准备发送帧 (正确规范)
        
        Args:
            linear_x: 线速度
            angular_z: 角速度
            servo1-servo5: 舵机控制值
            servo6: 开关门控制值 (0/1)
            
        Returns:
            构建好的完整数据帧
        """
        # 帧头
        tx_buffer = bytearray()
        tx_buffer.append(self.FRAME_HEADER)
        
        # 功能码
        tx_buffer.append(self.SEND_FUNC_CODE)
        
        tx_buffer.append(0x14)
        
        # 打包数据（小端模式）：linear_x, angular_z, servo1, servo2, servo3, servo4, servo5, servo6
        tx_buffer.extend(struct.pack('<ff', linear_x, angular_z))
        tx_buffer.extend(struct.pack('<HHHHHH', servo1, servo2, servo3, servo4, servo5, servo6))
        
        # 计算校验和：从功能码开始的所有数据 (功能码 + 数据长度 + payload = 22字节)
        data_to_checksum = tx_buffer[1:]
        checksum = sum(data_to_checksum) & 0xFF
        tx_buffer.append(checksum)
        
        # 帧尾
        tx_buffer.append(self.FRAME_FOOTER)
        
        return tx_buffer
    
    def send_data(self, serial_data: SerialData) -> None:
        """
        打包并发送控制数据到下位机（ROS回调函数接口）
        
        Args:
            serial_data: 包含线速度、角速度和舵机控制值的SerialData消息
        """
        self._send_frame(serial_data)
        
        
        
    #==================================================================================发送部分结束==================================================================================
    
    
    
    
    #==================================================================================辅助函数==================================================================================
    def _calculate_checksum(self, data):
        """
        计算校验和
        
        Args:
            data: 待计算校验和的字节数据
            
        Returns:
            计算得到的校验和（1字节）
        """
        sum_value = sum(data)
        return sum_value & 0xFF
    
    def _publish_feedback(self, received_data):
        """
        发布反馈数据到ROS话题
        
        Args:
            received_data: 包含线速度、角速度和舵机反馈值的数组
                [linear_x, angular_z, servo1, servo2, servo3, servo4, servo5]
        """
        # 创建SerialData消息
        msg = SerialData()
        
        # 设置数据
        msg.linear_x = received_data[0]
        msg.angular_z = received_data[1]
        msg.servo1 = int(received_data[2])
        msg.servo2 = int(received_data[3])
        msg.servo3 = int(received_data[4])
        msg.servo4 = int(received_data[5])
        msg.servo5 = int(received_data[6])
        
        # 发布消息
        self.feedback_publisher.publish(msg)

    def _connect_serial(self):
        """
        连接串口，支持首次连接和重连
        
        Returns:
            bool: 连接是否成功
        """
        if self.ser is not None and self.ser.is_open:
            return True
            
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f'串口 {self.port} 已打开，波特率 {self.baudrate}')
            self.current_reconnect_attempt = 0
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'无法打开串口 {self.port}: {e}')
            return False
    
    def _try_reconnect(self):
        """
        尝试重新连接串口
        
        Returns:
            bool: 重连是否成功
        """
        if self.current_reconnect_attempt >= self.max_reconnect_attempts:
            self.get_logger().error(f'达到最大重连尝试次数 ({self.max_reconnect_attempts})，放弃重连')
            return False
            
        self.current_reconnect_attempt += 1
        self.get_logger().info(f'尝试重连 {self.current_reconnect_attempt}/{self.max_reconnect_attempts}...')
        
        # 确保先关闭旧连接
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
                self.get_logger().info('已关闭旧串口连接')
            except Exception as e:
                self.get_logger().warn(f'关闭串口时出错: {e}')
            
        # 尝试重连
        return self._connect_serial()
    
    def _check_connection(self):
        """
        检查串口连接状态，如果断开则尝试重连
        
        Returns:
            bool: 连接是否正常
        """
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('串口连接已断开，尝试重连...')
            return self._try_reconnect()
        return True

    def destroy_node(self):
        """
        销毁节点时关闭串口
        """
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


#==================================================================================辅助函数结束==================================================================================
