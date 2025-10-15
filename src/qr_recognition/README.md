# QR Recognition Package

二维码识别功能包，用于水果采摘机器人的二维码扫描识别。

## 功能特性

- 实时相机图像订阅与处理
- 二维码/条形码检测与解码
- Service控制扫描开关
- 支持识别水果名称和数字序列

## 节点说明

### qr_recognition_node

**订阅话题**：
- `/camera/color/image_raw` (sensor_msgs/Image) - 相机图像

**提供服务**：
- `/qr_scan_control` (robot_control_interfaces/ControlDetection) - 控制二维码扫描开关

## 使用方法

### 启动节点

```bash
source install/setup.bash
ros2 run qr_recognition qr_recognition_node
```

### 控制扫描开关

开启扫描：
```bash
ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection "{enable: true}"
```

关闭扫描：
```bash
ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection "{enable: false}"
```

## 支持的二维码数据格式

### 水果名称
- 洋葱
- 南瓜
- 西红柿
- 辣椒

### 数字序列
格式示例：`4,3,1,10,8,9,2,11`

## 依赖项

- rclpy
- sensor_msgs
- cv_bridge
- robot_control_interfaces
- opencv-python (cv2)
- pyzbar

### 安装Python依赖

```bash
pip install opencv-python pyzbar
```

## 代码结构

```
qr_recognition/
├── qr_recognition/
│   ├── __init__.py
│   ├── qr_control.py         # 主节点实现
│   └── test_qr_control.py    # 测试文件
├── package.xml
├── setup.py
└── README.md
```

## 示例输出

```
[INFO] [qr_recognition_node]: QR Recognition Node has been started.
[INFO] [qr_recognition_node]: QR scan control service created: /qr_scan_control
[INFO] [qr_recognition_node]: 二维码扫描已开启
[INFO] [qr_recognition_node]: 检测到二维码 [QRCODE]: 洋葱
[INFO] [qr_recognition_node]: 识别到水果: 洋葱
[INFO] [qr_recognition_node]: 检测到二维码 [QRCODE]: 4,3,1,10,8,9,2,11
[INFO] [qr_recognition_node]: 识别到数字序列: [4, 3, 1, 10, 8, 9, 2, 11]
```

## 注意事项

1. 确保相机节点已启动并发布图像到正确的话题
2. 默认情况下扫描功能是关闭的，需要通过service开启
3. 图像处理仅在扫描开关开启时进行，以节省计算资源
4. 支持同时识别多个二维码
