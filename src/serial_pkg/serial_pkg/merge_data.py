import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from robot_control_interfaces.msg import SerialData
import math
from std_srvs.srv import Trigger
from rclpy.parameter import Parameter

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
        
        # 订阅机械臂关节状态（JointState 标准消息）
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/arm_control_data',  # fruit_arm_driver 发布到这个话题
            self.joint_state_callback,
            10)        


        self.control_data_publisher = self.create_publisher(SerialData, 'control_data', 10)
        # --- 定时器 ---
        # 10Hz 发布一次合并后的数据
        # 读取 servo5 参数  的定时器
        self.timer1 = self.create_timer(0.1, self.get_servo_data)
        self.timer2 = self.create_timer(0.1, self.pack_and_publish_data) 
        
        self.last_servo5 = None
        self.last_servo6 = None

        # --- 服务 ---
        self.toggle_servo5_service = self.create_service(
            Trigger,
            'toggle_servo5',
            self.toggle_servo5_callback
        )
        self.toggle_servo6_service = self.create_service(
            Trigger,
            'toggle_servo6',
            self.toggle_servo6_callback
        )

    def twist_callback(self, msg):
        # 缓存从 /cmd_vel 收到的最新数据
        self.latest_twist = msg

    def joint_state_callback(self, msg):
        """
        接收从 fruit_arm_driver 发来的 JointState 消息（弧度值）
        
        Args:
            msg (JointState): 标准 JointState 消息，包含 name 和 position 数组
        """
        # 解析 JointState 消息：遍历 name 和 position 数组
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.latest_joint_states[joint_name] = msg.position[i]
        
        self.get_logger().debug(f"收到机械臂角度(弧度): {self.latest_joint_states}")


    def toggle_servo5_callback(self, request, response):
        """
        Toggle servo5 开关
        
        Args:
            request (Trigger.Request): 标准Trigger请求
            response (Trigger.Response): 标准Trigger响应
            
        Returns:
            Trigger.Response: 成功响应
        """
        current_val = self.get_parameter('servo5').get_parameter_value().integer_value
        new_val = 1 if current_val == 0 else 0
        self.set_parameters([Parameter('servo5', Parameter.Type.INTEGER, new_val)])
        response.success = True
        response.message = f"Servo5 toggled to {new_val}"
        self.get_logger().info(f"Servo5 toggled to {new_val}")
        return response

    def toggle_servo6_callback(self, request, response):
        """
        Toggle servo6 开关
        
        Args:
            request (Trigger.Request): 标准Trigger请求
            response (Trigger.Response): 标准Trigger响应
            
        Returns:
            Trigger.Response: 成功响应
        """
        current_val = self.get_parameter('servo6').get_parameter_value().integer_value
        new_val = 1 if current_val == 0 else 0
        self.set_parameters([Parameter('servo6', Parameter.Type.INTEGER, new_val)])
        response.success = True
        response.message = f"Servo6 toggled to {new_val}"
        self.get_logger().info(f"Servo6 toggled to {new_val}")
        return response

    def get_servo_data(self):
        # servo5 declared as integer; read integer_value
        servo5_data = self.get_parameter('servo5').get_parameter_value().integer_value  # uint16
        servo6_data = self.get_parameter('servo6').get_parameter_value().integer_value  # uint16
        
        #设置只有在被改变之后发info日志
        if servo5_data != self.last_servo5 or servo6_data != self.last_servo6:
            self.last_servo5 = servo5_data
            self.last_servo6 = servo6_data
            self.get_logger().info(f"Servo 5 Data: {servo5_data}")
            self.get_logger().info(f"Servo 6 Data: {servo6_data}")
        
        # self.get_logger().info(f"Servo 5 Data: {servo5_data}")
        # self.get_logger().info(f"Servo 6 Data: {servo6_data}")

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