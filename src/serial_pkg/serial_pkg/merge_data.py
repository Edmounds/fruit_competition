import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from robot_control_interfaces.msg import SerialData
from robot_control_interfaces.msg import ArmControl
import math

class DataMergerNode(Node):
    def __init__(self):
        super().__init__('data_merger_node')

        # --- 参数定义 ---
        # 定义舵机名称到其在数据帧中顺序的映射关系
        # 请根据你的 URDF/xacro 文件中的 joint 名称进行修改！
        self.declare_parameter('joint_map.servo1', 'j1') # 云台
        self.declare_parameter('joint_map.servo2', 'j2')
        self.declare_parameter('joint_map.servo3', 'j3')
        self.declare_parameter('joint_map.servo4', 'j4')
        # servo5 should be an integer (uint16). Declare default as int 0.
        self.declare_parameter('servo5', 0)
        self.declare_parameter('servo6', 0)  

        self.joint_map = {
            'servo1': self.get_parameter('joint_map.servo1').get_parameter_value().string_value,
            'servo2': self.get_parameter('joint_map.servo2').get_parameter_value().string_value,
            'servo3': self.get_parameter('joint_map.servo3').get_parameter_value().string_value,
            'servo4': self.get_parameter('joint_map.servo4').get_parameter_value().string_value,
        }
        self.get_logger().info(f"Joint mapping: {self.joint_map}")


        # --- 内部数据缓存 ---
        self.latest_twist = Twist()
        self.latest_joint_states = {} # 使用字典来存储关节名和位置，方便查找

        # --- 订阅者 ---
        self.twist_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel', # rqt_robot_steering 默认发布到这个话题
            self.twist_callback,
            10)
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states', # joint_state_publisher_gui 发布到这个话题
            self.joint_state_callback,
            10)

        self.control_data_publisher = self.create_publisher(SerialData, 'control_data', 10)

        
        # --- 定时器 ---
        # 10Hz 发布一次合并后的数据
        # 读取 servo5 参数  的定时器
        self.timer1 = self.create_timer(0.1, self.get_servo_data)
        self.timer2 = self.create_timer(0.1, self.pack_and_publish_data) 

    def twist_callback(self, msg):
        # 缓存从 /cmd_vel 收到的最新数据
        self.latest_twist = msg

    def joint_state_callback(self, msg):
        # 缓存从 /joint_states 收到的最新数据
        # 将其转换为 关节名->位置 的字典，方便查找
        for i, name in enumerate(msg.name):
            self.latest_joint_states[name] = msg.position[i]

    def get_servo_data(self):
        # servo5 declared as integer; read integer_value
        servo5_data = self.get_parameter('servo5').get_parameter_value().integer_value  # uint16
        servo6_data = self.get_parameter('servo6').get_parameter_value().integer_value  # uint16
        self.get_logger().info(f"Servo 5 Data: {servo5_data}")
        self.get_logger().info(f"Servo 6 Data: {servo6_data}")

    def rad_to_servo_val(self, rad_val, servo_type):
       
       # rad to 0-1000
       # 机械臂零点是0-1000中的500,而在urdf中舵机所在的500表现为0度
       # 240舵机: (-120 to 120)度 -> 0-1000
        if servo_type == '240':
            deg_val = math.degrees(rad_val)
            servo_val = int((deg_val + 120) / 240 * 1000)

        elif servo_type == '270':
            deg_val = math.degrees(rad_val)
            servo_val = int((deg_val + 135) / 270 * 1000)

        else:
            self.get_logger().error(f"Unknown servo type: {servo_type}")
            servo_val = 500  # 默认值
                
        return servo_val
        

    def pack_and_publish_data(self):
        # --- 1. 数据提取和转换 ---
        
        # 机器人速度
        linear_x = float(self.latest_twist.linear.x)
        angular_z = float(self.latest_twist.angular.z)

        servo1_val = self.rad_to_servo_val(self.latest_joint_states.get(self.joint_map['servo1'], 0.0), '270')  # 云台舵机是240型
        servo2_val = self.rad_to_servo_val(self.latest_joint_states.get(self.joint_map['servo2'], 0.0), '240')
        servo3_val = self.rad_to_servo_val(self.latest_joint_states.get(self.joint_map['servo3'], 0.0), '240')
        servo4_val = self.rad_to_servo_val(self.latest_joint_states.get(self.joint_map['servo4'], 0.0), '240')
        # read servo5 as integer parameter
        try:
            servo5_val = int(self.get_parameter('servo5').get_parameter_value().integer_value)  # uint16
            servo6_val = int(self.get_parameter('servo6').get_parameter_value().integer_value)  # uint16
        except Exception:
            # fallback to 0 if parameter unexpectedly not integer
            self.get_logger().warning('servo5 or servo6 parameter not integer, using 0')
            servo5_val = 0  # uint16
            servo6_val = 0  # uint16

        msg = SerialData()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.data = [linear_x, angular_z, servo1_val, servo2_val, servo3_val, servo4_val, servo5_val, servo6_val]
        msg.servo1 = servo1_val
        msg.servo2 = servo2_val
        msg.servo3 = servo3_val
        msg.servo4 = servo4_val
        msg.servo5 = servo5_val
        msg.servo6 = servo6_val
        msg.linear_x = linear_x
        msg.angular_z = angular_z

        self.control_data_publisher.publish(msg)
        self.get_logger().info(f"Published SerialData: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = DataMergerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()