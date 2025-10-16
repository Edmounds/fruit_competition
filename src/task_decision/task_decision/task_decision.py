import rclpy
from rclpy.node import Node
from robot_control_interfaces.srv import ControlDetection
import time
import os
import subprocess
# 导入正确的消息和服务类型
from robot_control_interfaces.srv import FruitDetectionControl, MoveToPosition, MoveToPose
from robot_control_interfaces.msg import FruitInfo

# 导入标准服务类型
from std_srvs.srv import Trigger

# 导入新增的模块
from task_decision.waypoint_manager import WaypointManager
from task_decision.goal_tracker import GoalTracker
from task_decision.motor_controller import Motor


class TaskDecision(Node):
    def __init__(self):
        # ... (你原来的 __init__ 代码)
        super().__init__('task_decision')
        self.get_logger().info("Task Decision Node has been started.")
        
        # ===== 路点导航模块初始化 =====
        # 获取配置文件路径
        config_dir = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config'
        )
        waypoints_yaml = os.path.join(config_dir, 'waypoints.yaml')
        
        # 初始化路点管理器
        try:
            self.waypoint_manager = WaypointManager(waypoints_yaml)
            self.get_logger().info("✓ 路点管理器已初始化")
            # 打印路点配置摘要
            summary = self.waypoint_manager.print_summary()
            for line in summary.split('\n'):
                if line.strip():
                    self.get_logger().info(line)
        except Exception as e:
            self.get_logger().error(f"✗ 路点管理器初始化失败: {e}")
            self.waypoint_manager = None
        
        # 初始化目标追踪器
        self.goal_tracker = GoalTracker(
            self,
            distance_tolerance=0.1,  # 距离容限：0.1m
            angle_tolerance=0.1      # 角度容限：0.1rad (≈5.7°)
        )
        
        # 注册到达目标的回调
        self.goal_tracker.on_reached(self._on_goal_reached)
        self.goal_tracker.on_near_target(self._on_near_target)
        
        self.get_logger().info("✓ 目标追踪器已初始化")
        
        # ===== 服务客户端初始化 =====
        # 创建二维码扫描控制服务客户端
        self.qr_scan_client = self.create_client(
            ControlDetection,
            'qr_scan_control'
        )
        
        # 【新增】创建水果检测服务客户端
        self.fruit_detection_client = self.create_client(
            FruitDetectionControl,
            'fruit_detection_control'
        )
        
        # 【新增】创建笛卡尔控制器服务客户端
        self.cartesian_controller_client = self.create_client(
            MoveToPosition,
            'move_to_position'
        )
        
        # 【新增】创建命名位姿控制服务客户端
        self.move_to_pose_client = self.create_client(
            MoveToPose,
            'move_to_pose'
        )
        
        # 【新增】创建爪子控制服务客户端
        self.toggle_servo5_client = self.create_client(
            Trigger,
            'toggle_servo5'
        )
        
        # 【新增】订阅水果检测结果话题
        self.fruit_detection_subscriber = self.create_subscription(
            FruitInfo,
            'fruit_detection',
            self.fruit_detection_callback,
            10
        )
        
        # 【新增】初始化电机控制器
        # 电机控制器负责发布 /cmd_vel 话题，控制移动底盘速度
        self.motor = Motor(self)
        
        # 用于存储最新的检测结果
        self.latest_fruit_info = None
        self.get_logger().info("Subscribed to fruit_detection topic")
    
    def fruit_detection_callback(self, msg: FruitInfo):
        """实时接收水果检测结果的回调函数
        
        Args:
            msg: FruitInfo消息，包含is_ripe, fruit_type, offset_x, offset_y
        """
        self.latest_fruit_info = msg
        self.get_logger().debug(
            f'Received fruit detection: is_ripe={msg.is_ripe}, type={msg.fruit_type}, '
            f'offset=({msg.offset_x:.1f}, {msg.offset_y:.1f})'
        )
    # 调用水果检测服务的方法
    def call_fruit_detection_service(self, enable, model_mode=0):
        """调用水果检测服务
        
        Args:
            enable: bool, True=开启检测, False=关闭检测
            model_mode: int, 模型模式 (0=仅成熟度, 1=仅种类, 2=两阶段)
            
        Returns:
            response: FruitDetectionControl.Response 或 None
        """
        # 等待服务可用
        if not self.fruit_detection_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Fruit detection service not available!')
            return None
        
        # 创建请求
        request = FruitDetectionControl.Request()
        request.enable = enable
        request.model_mode = model_mode
        
        # 发送请求
        self.get_logger().info(f'Calling fruit detection service: enable={enable}, mode={model_mode}')
        future = self.fruit_detection_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Fruit detection response: success={response.success}, message={response.message}')
            return response
        else:
            self.get_logger().error('Fruit detection service call failed!')
            return None
        
    def call_qr_scan_service(self, enable, qr_type=''):
        """调用二维码扫描控制服务
        
        Args:
            enable: bool, True=开启扫描, False=关闭扫描
            qr_type: str, 'B'=水果列表, 'C'=数字序列或单个水果名
            
        Returns:
            response: ControlDetection.Response 或 None
        """
        # 等待服务可用
        if not self.qr_scan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('QR scan service not available!')
            return None
        
        # 创建请求
        request = ControlDetection.Request()
        request.enable = enable
        request.qr_type = qr_type
        
        # 发送请求
        self.get_logger().info(f'Calling QR scan service: enable={enable}, type={qr_type}')
        future = self.qr_scan_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Service response: success={response.success}, message={response.message}')
            return response
        else:
            self.get_logger().error('Service call failed!')
            return None
    
    def move_to_named_pose(self, pose_name: str, timeout_sec: float = 30.0):
        """移动机械臂到命名位姿
        
        调用MoveIt的命名位姿服务，将机械臂移动到预定义的位姿
        
        Args:
            pose_name (str): 位姿名称 (如: home, harvest, scan, detect_left, detect_right, grab_left, grab_right)
            timeout_sec (float): 服务调用超时时间 (秒)
            
        Returns:
            bool: 是否执行成功
        """
        # 等待服务可用
        self.get_logger().info(f'等待命名位姿控制服务 /move_to_pose ...')
        if not self.move_to_pose_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('✗ 命名位姿控制服务不可用!')
            self.get_logger().info('请先启动服务节点: ros2 run moveit_controller moveit_pose_controller')
            return False
        
        # 创建请求
        request = MoveToPose.Request()
        request.pose_name = pose_name
        
        # 发送请求
        self.get_logger().info(f'→ 发送请求: 移动到位姿 "{pose_name}" (这可能需要几秒钟...)')
        future = self.move_to_pose_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ 成功: {response.message}')
                return True
            else:
                self.get_logger().warn(f'✗ 失败: {response.message}')
                return False
        else:
            self.get_logger().error('✗ 命名位姿控制服务调用超时!')
            return False
    
    def move_arm_forward(self, distance: float, timeout_sec: float = 10.0):
        """控制机械臂往前移动指定距离
        
        使用subprocess调用moveit_controller的cartesian_controller命令行工具
        
        Args:
            distance (float): 往前移动的距离 (米，正值表示往前)
            timeout_sec (float): 命令执行超时时间 (秒)
            
        Returns:
            bool: 是否执行成功
        """
        try:
            # ROS 2 工作空间路径
            workspace_setup = '/home/rc1/fruit_ws/install/setup.bash'
            ws_moveit = '/home/rc1/ws_moveit/install/setup.bash'
            
            # 构造完整命令：先source环境，再执行ros2命令
            # 使用bash -c执行组合命令
            ros2_cmd = f'ros2 run moveit_controller cartesian_controller 0.0 {distance} 0.0'
            full_cmd = f'source {workspace_setup} && {ros2_cmd}'
            full_cmd = f'source {ws_moveit} && ' + full_cmd  # 先source第二个工作空间
            
            self.get_logger().info(f'机械臂往前移动: {distance:.3f}m')
            self.get_logger().debug(f'执行命令: {ros2_cmd}')
            
            # 使用bash -c执行组合命令
            result = subprocess.run(
                ['bash', '-c', full_cmd],
                timeout=timeout_sec,
                capture_output=True,
                text=True
            )
            
            # 检查返回码
            if result.returncode == 0:
                self.get_logger().info(f'✓ 移动成功')
                # 可选：打印标准输出的最后几行
                if result.stdout:
                    for line in result.stdout.strip().split('\n')[-3:]:
                        if line.strip():
                            self.get_logger().debug(f'  {line}')
                return True
            else:
                self.get_logger().warn(f'✗ 移动失败 (返回码: {result.returncode})')
                if result.stderr:
                    self.get_logger().error(f'错误信息: {result.stderr.strip()}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'✗ 命令执行超时 ({timeout_sec}秒)')
            return False
        except Exception as e:
            self.get_logger().error(f'✗ 执行命令失败: {e}')
            return False
    
    def toggle_gripper(self, timeout_sec: float = 5.0) -> bool:
        """控制机械臂末端爪子的开合（通过servo5）
        
        调用 /toggle_servo5 服务，切换爪子状态（开→闭 或 闭→开）
        每次调用都会在当前状态基础上进行切换
        
        Args:
            timeout_sec (float): 服务调用超时时间 (秒)，默认5.0秒
            
        Returns:
            bool: 是否执行成功
            
        Example:
            >>> # 切换爪子状态（打开→关闭 或 关闭→打开）
            >>> success = node.toggle_gripper()
            >>> if success:
            ...     print("爪子状态已切换")
        """
        # 等待服务可用
        if not self.toggle_servo5_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('✗ 爪子控制服务 /toggle_servo5 不可用!')
            self.get_logger().info('请确认串口通信节点已启动: ros2 run serial_pkg merge_data')
            return False
        
        # 创建请求（Trigger服务不需要参数）
        request = Trigger.Request()
        
        # 发送请求
        self.get_logger().info('→ 切换爪子状态...')
        future = self.toggle_servo5_client.call_async(request)
        
        # 等待响应
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ 成功: {response.message}')
                return True
            else:
                self.get_logger().warn(f'✗ 失败: {response.message}')
                return False
        else:
            self.get_logger().error('✗ 爪子控制服务调用超时!')
            return False
    
    def open_gripper(self, timeout_sec: float = 5.0) -> bool:
        """打开爪子（便捷方法）
        
        注意：由于toggle_servo5是切换服务，此方法会调用一次切换。
        如果需要确保爪子状态，建议在初始化时记录状态或使用参数服务查询当前状态。
        
        Args:
            timeout_sec (float): 超时时间（秒）
            
        Returns:
            bool: 是否执行成功
        """
        self.get_logger().info('🖐️ 打开爪子')
        return self.toggle_gripper(timeout_sec)
    
    def close_gripper(self, timeout_sec: float = 5.0) -> bool:
        """关闭爪子（便捷方法）
        
        注意：由于toggle_servo5是切换服务，此方法会调用一次切换。
        如果需要确保爪子状态，建议在初始化时记录状态或使用参数服务查询当前状态。
        
        Args:
            timeout_sec (float): 超时时间（秒）
            
        Returns:
            bool: 是否执行成功
        """
        self.get_logger().info('✊ 关闭爪子')
        return self.toggle_gripper(timeout_sec)
    
    
    # ====================================== 路点导航相关方法 ======================================
    
    def navigate_to_waypoint(self, target_name: str, timeout_sec: float = 120.0) -> bool:
        """
        导航到指定的路点并等待到达
        
        该方法会调用导航栈将机器人导航到指定路点，并通过goal_tracker
        判断是否成功到达。支持超时机制。
        
        Args:
            target_name (str): 目标名称 (如"A1", "B_qr_scanner", "start_location"等)
            timeout_sec (float): 导航超时时间，单位秒 (默认120秒)
            
        Returns:
            bool: True表示成功到达，False表示导航失败或超时
            
        Example:
            >>> # 导航到A1采摘点
            >>> success = node.navigate_to_waypoint("A1", timeout_sec=60.0)
            >>> if success:
            ...     print("已到达A1，开始采摘")
            ... else:
            ...     print("导航失败")
        """
        if not self.waypoint_manager:
            self.get_logger().error("路点管理器未初始化")
            return False
        
        # 获取目标路点
        waypoint = self.waypoint_manager.get_waypoint(target_name)
        if waypoint is None:
            self.get_logger().error(f"路点不存在: {target_name}")
            return False
        
        self.get_logger().info(f"开始导航到 [{target_name}]")
        
        # 设置目标
        self.goal_tracker.set_target_pose(waypoint.pose, target_name)
        
        # 【TODO】调用实际的导航服务 (move_base, nav2等)
        # 这里需要根据你的实际导航栈进行调整
        # 示例代码：
        # nav_result = self._call_navigation_service(
        #     x=waypoint.pose.x,
        #     y=waypoint.pose.y,
        #     theta=waypoint.pose.theta
        # )
        
        # 等待到达
        import time
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.goal_tracker.is_reached():
                self.get_logger().info(f"✓ 成功到达 [{target_name}]")
                return True
            
            # 检查是否接近（用于日志输出）
            if self.goal_tracker.is_near_target(distance_threshold=0.3):
                distance, angle = self.goal_tracker.get_error()
                self.get_logger().debug(
                    f"接近目标 [{target_name}]: 距离={distance:.3f}m, 角度误差={angle:.3f}rad"
                )
            
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().error(f"✗ 导航超时: [{target_name}] (超时时间: {timeout_sec}秒)")
        return False
    
    def navigate_to_zone(self, zone_name: str, target_index: int = 0, timeout_sec: float = 60.0) -> bool:
        """
        导航到指定区域的特定目标点
        
        Args:
            zone_name (str): 区域名称 (如"A", "B", "C")
            target_index (int): 该区域内的目标索引 (0表示第一个目标)
            timeout_sec (float): 超时时间，单位秒
            
        Returns:
            bool: 是否成功到达
            
        Example:
            >>> # 导航到B区的第一个树
            >>> success = node.navigate_to_zone("B", target_index=0, timeout_sec=60.0)
        """
        if not self.waypoint_manager:
            self.get_logger().error("路点管理器未初始化")
            return False
        
        # 获取该区域的所有路点
        waypoints = self.waypoint_manager.get_zone_waypoints(zone_name)
        
        if not waypoints:
            self.get_logger().error(f"区域不存在或无路点: {zone_name}")
            return False
        
        if target_index >= len(waypoints):
            self.get_logger().error(f"目标索引越界: {zone_name}[{target_index}], 仅有{len(waypoints)}个目标")
            return False
        
        target_waypoint = waypoints[target_index]
        return self.navigate_to_waypoint(target_waypoint.target_name, timeout_sec)
    
    def navigate_to_start(self, timeout_sec: float = 60.0) -> bool:
        """
        导航到起始位置
        
        Args:
            timeout_sec (float): 超时时间，单位秒
            
        Returns:
            bool: 是否成功到达
        """
        if not self.waypoint_manager:
            self.get_logger().error("路点管理器未初始化")
            return False
        
        start_wp = self.waypoint_manager.get_start_location()
        if start_wp is None:
            self.get_logger().error("起始位置未定义")
            return False
        
        return self.navigate_to_waypoint(start_wp.target_name, timeout_sec)
    
    def navigate_to_collection_point(self, timeout_sec: float = 60.0) -> bool:
        """
        导航到收集点（放置采摘物的位置）
        
        Args:
            timeout_sec (float): 超时时间，单位秒
            
        Returns:
            bool: 是否成功到达
        """
        if not self.waypoint_manager:
            self.get_logger().error("路点管理器未初始化")
            return False
        
        collection_wp = self.waypoint_manager.get_collected_location()
        if collection_wp is None:
            self.get_logger().error("收集位置未定义")
            return False
        
        return self.navigate_to_waypoint(collection_wp.target_name, timeout_sec)
    
    def list_available_waypoints(self) -> None:
        """
        打印所有可用的路点（用于调试）
        """
        if not self.waypoint_manager:
            self.get_logger().error("路点管理器未初始化")
            return
        
        summary = self.waypoint_manager.print_summary()
        for line in summary.split('\n'):
            if line.strip():
                self.get_logger().info(line)
    
    def get_navigation_status(self) -> str:
        """
        获取导航状态信息
        
        Returns:
            str: 状态信息
        """
        return self.goal_tracker.print_status()
    
    def _on_goal_reached(self, target_name: str) -> None:
        """
        目标到达回调（内部方法）
        
        当机器人首次到达目标点时调用
        """
        self.get_logger().info(f"🎯 [回调] 已到达目标: {target_name}")
        # 可在此添加额外的处理逻辑，如停止移动、启动采摘等
    
    def _on_near_target(self, target_name: str, distance: float, angle_error: float) -> None:
        """
        接近目标回调（内部方法）
        
        当机器人接近但未到达目标时调用（持续调用）
        """
        self.get_logger().debug(
            f"📍 [回调] 接近目标 [{target_name}]: 距离={distance:.3f}m, 角度误差={angle_error:.3f}rad"
        )
    
    def execute_fruit_detection_pipeline(self, detection_timeout: float = 5.0):
        """执行水果检测流程（成熟度检测→种类识别）
        
        该方法实现两阶段水果检测流程：
        1. 阶段1：成熟度检测（模式0）- 识别水果是否成熟
        2. 阶段2：种类识别（模式1）- 如果检测到成熟水果，进行种类分类
        
        Args:
            detection_timeout (float): 每个检测阶段的最大等待时间（秒），默认5.0秒
            
        Returns:
            dict: 检测结果字典，包含以下字段：
                - 'success' (bool): 整个流程是否成功
                - 'ripeness_detected' (bool): 是否检测到成熟水果
                - 'fruit_type' (int): 水果类型编号（1-4）或0表示未检测到
                - 'fruit_name' (str): 水果名称或"未知"
                - 'offset_x' (float): X轴偏移
                - 'offset_y' (float): Y轴偏移
                - 'message' (str): 过程描述信息
        """
        result = {
            'success': False,
            'ripeness_detected': False,
            'fruit_type': 0,
            'fruit_name': '未知',
            'offset_x': 0.0,
            'offset_y': 0.0,
            'message': ''
        }
        
        self.get_logger().info("Starting fruit detection process...")
        
        # ===== 步骤1：模式0 - 成熟度检测 =====
        self.get_logger().info("\n[Step 1] Mode 0: Ripeness detection only...")
        self.get_logger().info("Waiting for fruit detector publisher...")
        
        # 等待发布者就绪
        while self.get_publishers_info_by_topic('fruit_detection') == []:
            time.sleep(0.1)
        self.get_logger().info("Publisher found!")
        
        # 清空之前的检测结果
        self.latest_fruit_info = None
        
        # 调用服务启动成熟度检测
        response = self.call_fruit_detection_service(enable=True, model_mode=0)
        if not response or not response.success:
            self.get_logger().error('Failed to enable ripeness detection')
            result['message'] = '成熟度检测服务启动失败'
            return result
        else:
            self.get_logger().info('成熟度检测服务已启动，等待检测结果...')
        
        # 等待检测服务处理图像并通过话题发布结果
        self.get_logger().info("等待成熟度检测中...")
        start_time = time.time()
        
        # 使用spin循环等待话题数据
        while (time.time() - start_time) < detection_timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_fruit_info is not None:
                break
        
        # 判断是否检测到成熟水果
        ripeness_detected = False
        self.get_logger().warn(f'Latest fruit info: {self.latest_fruit_info}')
        
        if self.latest_fruit_info is not None and self.latest_fruit_info.is_ripe:
            self.get_logger().info(f'✓ 检测到成熟水果！is_ripe={self.latest_fruit_info.is_ripe}')
            ripeness_detected = True
            result['ripeness_detected'] = True
        else:
            self.get_logger().info('✗ 未检测到成熟水果')
            result['message'] = '未检测到成熟水果，流程结束'
            return result

        # ===== 步骤2：如果检测到成熟，立即切换到模式1进行种类识别 =====
        if ripeness_detected:
            self.get_logger().info("\n[Step 2] Switching to Mode 1: Fruit type classification...")
            
            # 清空之前的检测结果
            self.latest_fruit_info = None
            
            # 调用服务切换到种类识别模式
            response = self.call_fruit_detection_service(enable=True, model_mode=1)
            if not response or not response.success:
                self.get_logger().error('Failed to switch to fruit type classification')
                result['message'] = '种类识别服务启动失败'
                return result
            else:
                self.get_logger().info('种类识别服务已启动，等待分类结果...')
            
            # 等待种类识别服务处理图像
            self.get_logger().info("等待水果种类识别中...")
            start_time = time.time()
            
            # 使用spin循环等待话题数据
            classification_received = False  # 新增一个标志位
            while (time.time() - start_time) < detection_timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # 核心修改：不仅要检查消息是否存在，还要检查消息内容是否是我们想要的
                if self.latest_fruit_info is not None and self.latest_fruit_info.fruit_type > 0:
                    classification_received = True
                    break  # 只有收到有效的分类结果才退出循环
                
            # 根据最新的话题数据进行决策
            # 判断条件改为检查标志位
            if classification_received:
                # 定义水果类型映射
                fruit_names = {1: "辣椒", 2: "南瓜", 3: "洋葱", 4: "番茄"}
                fruit_name = fruit_names.get(self.latest_fruit_info.fruit_type, "未知")
                
                self.get_logger().info(
                    f'✓ 成熟水果类型识别成功! 类型={fruit_name}(编号{self.latest_fruit_info.fruit_type}), '
                    f'偏移=({self.latest_fruit_info.offset_x:.1f}, {self.latest_fruit_info.offset_y:.1f})'
                )
                
                # 填充结果
                result['success'] = True
                result['fruit_type'] = self.latest_fruit_info.fruit_type
                result['fruit_name'] = fruit_name
                result['offset_x'] = self.latest_fruit_info.offset_x
                result['offset_y'] = self.latest_fruit_info.offset_y
                result['message'] = f'检测成功: {fruit_name}'
                
                self.get_logger().info('→ 执行抓取程序...')
                # 【新增】调用机械臂往前移动进行抓取（0.15m作为示例）
                self.move_arm_forward(distance=0.15, timeout_sec=10.0)
            else:
                self.get_logger().info('✗ 种类识别失败 (超时或结果无效)')
                self.get_logger().info('→ 可能需要调整相机角度或重新检测')
                result['message'] = '种类识别失败或超时'
        else:
            self.get_logger().info('→ 回归中位，准备执行下一步操作')
            # 此处可以添加机械臂回归中位的逻辑
        
        # 步骤3: 任务结束后，禁用检测以节省资源
        self.get_logger().info("\n[Step 3] Disabling fruit detection...")
        # self.call_fruit_detection_service(enable=False)
        #在运行之后手动关闭
        
        self.get_logger().info("Fruit detection process completed!")
        
        return result
    
    








def main(args=None):
    rclpy.init(args=args)
    node = TaskDecision()
    
    try:
        # # ======================================= 测试二维码扫描服务 START ==========================
        # node.get_logger().info("="*50)
        # node.get_logger().info("Testing QR code scan service...")
        # node.get_logger().info("="*50)
        
        # # 测试1: 启用B类型二维码扫描（水果列表）
        # node.get_logger().info("\n[Test 1] Enabling B type QR code scan (fruit list)...")
        # response = node.call_qr_scan_service(enable=True, qr_type='B')
        # time.sleep(10)  # 等待2秒观察扫描结果

        # # 测试2: 切换到C类型二维码扫描（数字序列或单个水果）
        # node.get_logger().info("\n[Test 2] Switching to C type QR code scan (number sequence or single fruit)...")
        # response = node.call_qr_scan_service(enable=True, qr_type='C')
        # time.sleep(2)  # 等待2秒观察扫描结果


        # # 测试3: 关闭扫描
        # node.get_logger().info("\n[Test 3] Disabling QR code scan...")
        # response = node.call_qr_scan_service(enable=False, qr_type='')
        
        # node.get_logger().info("\n" + "="*50)
        # node.get_logger().info("QR code scan service test completed!")
        # node.get_logger().info("="*50)
        
        # # 继续运行节点（可以持续接收二维码扫描结果）
        # node.get_logger().info("\nNode is running... Press Ctrl+C to exit.")


#====================================================测试扫描二维码 END ========================================

#最好讲这种常用的函数\包封装成一个类,然后直接调用就行了
#还设计成熟是否的判断之类的,还有语音播报,要注意
# 需要写成类的东西[]
#
#
#
#这里应该是一个循环小节点,就是流程是.先进行移动,移动到了第一个目标点之后,开始操控机械臂移动到指定位置之后,然后进行水果检测,根据检测结果执行不同的机械臂操作.




#流程

## 播报启动
## 循环小节点
    # 读取目标点,然后motor驱动过去,通过pose_data来判断有没有到,到了就motor停下.
    # 到了之后,机械臂移动到指定位置pose,先进行水果成熟度检测,根据检测结果执行不同的机械臂操作
    # 抓完左边的水果之后,重复上一次操作抓右边的水果
    # 抓完之后,机械臂回到中位
    
## 循环完A区之后,  去扫B区二维码
## 然后原路返回从A区过去C区扫二维码
## 然后去C区采摘
     #C区采摘逻辑 -简要就是循环摘,碰到要抓的水果就执行检测->抓取的逻辑,没有就不抓
     
## 然后区收货区播报收获情况

#======================================move

# 获取A区的第一个采摘点
        node.get_logger().info("\nNavigating to A zone, first tree...")



#=====================================arm to pose=================================
        # 示例1: 移动到 home 位姿
        time.sleep(6)
        # node.get_logger().info("\n========== 测试1: 移动到 home 位姿 ==========")
        flag = node.move_to_named_pose('detect_left', timeout_sec=30.0)
        if flag:
            node.get_logger().info("机械臂已回到 detect_left 位姿")
            flag = node.move_to_named_pose('grab_left', timeout_sec=30.0)
            node.toggle_gripper()
            if flag:
                node.get_logger().info("机械臂已回到 grab_left 位姿")
                node.move_arm_forward(distance=0.10, timeout_sec=10.0)
                time.sleep(0.5)
                node.toggle_gripper()
                node.move_to_named_pose('harvest', timeout_sec=30.0)
                flag = node.toggle_gripper()
                if flag:
                    node.get_logger().info("机械臂已回到 harvest 位姿")
                    node.move_to_named_pose('home', timeout_sec=30.0)
                    node.toggle_gripper()
                else:
                    node.get_logger().error("机械臂夹爪关闭失败")
            else:
                node.get_logger().error("移动到 grab_left 位姿失败")
        else:
            node.get_logger().error("移动到 detect_left 位姿失败")
        
        
        time.sleep(1)
        
        # 示例2: 移动到其他命名位姿（根据你的配置文件）
   
        # node.move_to_named_pose('scan', timeout_sec=30.0)
        # node.move_to_named_pose('detect_left', timeout_sec=30.0)
        # node.move_to_named_pose('grab_left', timeout_sec=30.0)




    #=========================================fruit detection start================================

        # # 调用封装的水果检测流程函数
        # detection_result = node.execute_fruit_detection_pipeline(detection_timeout=5.0)
        
        # # 根据检测结果进行后续处理
        # node.get_logger().info(f"\n检测结果摘要: {detection_result['message']}")
        # if detection_result['success']:
        #     node.get_logger().info(
        #         f"  - 水果类型: {detection_result['fruit_name']} (编号{detection_result['fruit_type']})\n"
        #         f"  - 位置偏移: X={detection_result['offset_x']:.1f}, Y={detection_result['offset_y']:.1f}"
        #     )
        
        
        # time.sleep(1)  # 等待2秒观察结果
        
        # node.call_fruit_detection_service(enable=False)
#=====================================fruit detection end=================================


#====================================control the motor to get close the fruit=================================
       # 暂时不写闭环了


#========================================end========================================
        
        
#=====================================ARM Control start================================
        # 测试爪子控制
 
        
        
        
             
        
#=====================================ARM Control end================================
        

        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()