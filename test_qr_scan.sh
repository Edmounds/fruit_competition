#!/bin/bash
# QR二维码扫描测试脚本
# 使用方法：
#   终端1: 运行二维码识别节点
#   终端2: 运行此测试脚本

cd /home/rc1/fruit_ws
source install/setup.bash

echo "=================================="
echo "QR Code Scan Service Test"
echo "=================================="
echo ""

echo "请确保以下节点已运行："
echo "  1. 二维码识别节点: ros2 run qr_recognition qr_control"
echo "  2. 相机节点（如果需要实际扫描）"
echo ""
echo "按Enter开始测试..."
read

echo ""
echo "[测试1] 启用B类型二维码扫描（水果列表）"
echo "命令: ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection \"{enable: true, qr_type: 'B'}\""
ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection "{enable: true, qr_type: 'B'}"
echo ""
echo "等待5秒...（此时可以扫描B类型二维码）"
sleep 5

echo ""
echo "[测试2] 切换到C类型二维码扫描（数字序列或单个水果）"
echo "命令: ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection \"{enable: true, qr_type: 'C'}\""
ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection "{enable: true, qr_type: 'C'}"
echo ""
echo "等待5秒...（此时可以扫描C类型二维码）"
sleep 5

echo ""
echo "[测试3] 关闭二维码扫描"
echo "命令: ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection \"{enable: false, qr_type: ''}\""
ros2 service call /qr_scan_control robot_control_interfaces/srv/ControlDetection "{enable: false, qr_type: ''}"

echo ""
echo "=================================="
echo "测试完成！"
echo "=================================="
