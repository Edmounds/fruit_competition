#!/bin/bash

# 脚本功能: 为机械臂项目 (fruit_arm) 创建并应用 udev 规则
# Udev 规则文件的目标路径
RULES_FILE="/etc/udev/rules.d/99-fruit-arm.rules"

echo "--- 开始创建 Udev 规则 ---"

# 使用 sudo tee 和 heredoc (<<EOF) 将多行文本写入到需要root权限的文件
sudo tee "$RULES_FILE" > /dev/null <<EOF
# Rule for LSLIDAR (CH9102 chip with unique serial)
SUBSYSTEM=="tty", ATTRS{serial}=="58EB005821", MODE="0666", SYMLINK+="lslidar"

# Rule for DM-IMU (using its unique serial number)
SUBSYSTEM=="tty", ATTRS{serial}=="2025021200", MODE="0666", SYMLINK+="dm_imu"

# Rule for the other CH340 device (without a serial number)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="serial_ch340"
EOF

# 检查上一个命令是否成功执行
if [ $? -eq 0 ]; then
    echo "成功创建规则文件: $RULES_FILE"
else
    echo "错误: 创建规则文件失败！"
    exit 1
fi

echo "--- 重新加载 Udev 规则使其生效 ---"
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "--- 操作完成 ---"
echo "设备现在应该可以通过以下链接访问:"
echo "  - Lidar: /dev/lslidar"
echo "  - IMU:   /dev/dm_imu"
echo "  - Other: /dev/serial_ch340"