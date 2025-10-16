#!/bin/bash
# 测试 joint_states 话题和 MoveIt 连接性

echo "=========================================="
echo "测试 1: 检查 joint_states 话题"
echo "=========================================="
source /home/rc1/fruit_ws/install/setup.bash
ros2 topic echo /joint_states --once

echo ""
echo "=========================================="
echo "测试 2: 检查话题发布频率"
echo "=========================================="
timeout 3 ros2 topic hz /joint_states

echo ""
echo "=========================================="
echo "测试 3: 检查 MoveIt 是否接收到数据"
echo "=========================================="
ros2 topic info /joint_states --verbose

echo ""
echo "=========================================="
echo "测试 4: 检查时间戳是否正确"
echo "=========================================="
current_time=$(date +%s)
joint_state_time=$(ros2 topic echo /joint_states --once | grep "sec:" | head -1 | awk '{print $2}')
time_diff=$((current_time - joint_state_time))
echo "系统时间: $current_time"
echo "消息时间: $joint_state_time"
echo "时间差: $time_diff 秒"

if [ $time_diff -lt 2 ]; then
    echo "✅ 时间戳正常"
else
    echo "❌ 时间戳异常！"
fi
