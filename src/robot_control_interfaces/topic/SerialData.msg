# This is for fruit_robot

# nuc ->>>


#navigate
float32 linear_x    # 机器人前进/后退线速度 (m/s), 字节位置 3~6
float32 angular_z   # 机器人自转角速度 (rad/s), 字节位置 7~10

#arm
uint16 servo1       # 舵机1 (云台) 目标角度 (0-3600 代表 0-360.0 度), 字节位置 11~12
uint16 servo2       # 舵机5 目标角度, 字节位置 13~14
uint16 servo3       # 舵机2 目标角度, 字节位置 15~16
uint16 servo4       # 舵机4 目标角度, 字节位置 17~18
uint16 servo5       # 舵机3 (夹爪) 状态值: 1 代表闭合, 0 代表张开, 字节位置 19~20


# nuc <<<-

