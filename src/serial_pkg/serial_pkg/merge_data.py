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
            ArmControl,
            '/arm_control_data', # fruit_arm_driver 发布到这个话题
            self.joint_state_callback,
            10)

        # self.joint_state_subscriber = self.create_subscription(
        #     JointState,
        #     '/joint_states', # joint_state_publisher_gui 发布到这个话题
        #     self.joint_state_callback,
        #     10)        


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
        """
        接收从 fruit_arm_driver 发来的 ArmControl 消息（弧度值）
        
        Args:
            msg (ArmControl): 包含4个舵机的弧度值
        """
        # 直接存储弧度值，注意 ArmControl 的字段是 servo1-4
        self.latest_joint_states = {
            self.joint_map['servo1']: msg.servo1,  # j1
            self.joint_map['servo2']: msg.servo2,  # j2
            self.joint_map['servo3']: msg.servo3,  # j3
            self.joint_map['servo4']: msg.servo4,  # j4
        }
        self.get_logger().debug(f"收到机械臂角度(弧度): {self.latest_joint_states}")


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
        # 机器人速度
        linear_x = float(self.latest_twist.linear.x)
        angular_z = float(self.latest_twist.angular.z)

        # 从字典中获取弧度值，然后转换为舵机脉冲值
        # 注意：latest_joint_states 的 key 是 'j1', 'j2', 'j3', 'j4'
        servo1_val = self.rad_to_servo_val(
            self.latest_joint_states.get('j1', 0.0), '270'
        )
        servo2_val = self.rad_to_servo_val(
            self.latest_joint_states.get('j2', 0.0), '240'
        )
        servo3_val = self.rad_to_servo_val(
            self.latest_joint_states.get('j3', 0.0), '240'
        )
        servo4_val = self.rad_to_servo_val(
            self.latest_joint_states.get('j4', 0.0), '240'
        )
        
        # servo5 和 servo6 从参数读取
        try:
            servo5_val = int(self.get_parameter('servo5').get_parameter_value().integer_value)
            servo6_val = int(self.get_parameter('servo6').get_parameter_value().integer_value)
        except Exception:
            self.get_logger().warning('servo5 or servo6 parameter not integer, using 0')
            servo5_val = 0
            servo6_val = 0

        # 创建 SerialData 消息（注意这里需要 uint16）
        msg = SerialData()
        msg.servo1 = servo1_val  # int → uint16 自动转换
        msg.servo2 = servo2_val
        msg.servo3 = servo3_val
        msg.servo4 = servo4_val
        msg.servo5 = servo5_val
        msg.servo6 = servo6_val
        msg.linear_x = linear_x  # float → float32 自动转换
        msg.angular_z = angular_z

        self.control_data_publisher.publish(msg)
        self.get_logger().debug(f"Published SerialData: linear_x={linear_x:.2f}, angular_z={angular_z:.2f}, servos=[{servo1_val}, {servo2_val}, {servo3_val}, {servo4_val}, {servo5_val}, {servo6_val}]")

def main(args=None):
    rclpy.init(args=args)
    node = DataMergerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()