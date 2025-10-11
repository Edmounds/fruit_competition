import struct

PC_FRAME_HEADER = 0xAA
PC_FRAME_FOOTER = 0x55


# 模拟 C 语言中的校验和计算函数
def PC_CalculateChecksum(data):
    """
    计算校验和，对所有字节求和并取低8位
    """
    checksum = sum(data) & 0xFF
    return checksum


def Simulate_data():
    linear_x = 10.0
    angular_z = 0.0
    servo1 = 500
    servo2 = 500
    servo3 = 800
    servo4 = 200
    servo5 = 500
    servo6 = 500
    # 1. 打包数据
    packed_data = struct.pack('<B B B f f H H H H H H B B',
                              PC_FRAME_HEADER,  # 帧头
                              0x03,             # 功能码 (发送数据)
                              20,               # 数据长度 (20字节数据 + 2字节校验和)
                              linear_x,        # 线速度
                              angular_z,       # 角速度
                              servo1,          # 舵机1
                              servo2,          # 舵机2
                              servo3,          # 舵机3
                              servo4,          # 舵机4
                              servo5,          # 舵机5
                              servo6,          # 舵机6
                              0,               # 校验和占位符
                              PC_FRAME_FOOTER  # 帧尾
                              )

    # 计算校验和
    checksum = PC_CalculateChecksum(packed_data[1:-2])
    # 替换校验和占位符
    packed_data = bytearray(packed_data)
    packed_data[-2] = checksum
    return packed_data
# 2. 调用函数打包数据


def main():
    # # packed_data = "aa02149a9999becdcccc3e08078c0a8403080701007f55"
    # packed_data = "aa03140000000000000000f501fb01fd01bd01f5010000bb55"
    # packed_data = bytearray.fromhex(packed_data)
    # print(packed_data)
    # # 3. 打印结果
    # print(f"打包好的数据: {packed_data.hex()}")
    # print(f"数据长度: {len(packed_data)} 字节")

    # # 4. 验证校验和
    # print(f"十六进制表示: {packed_data.hex()}")
    # print(f"数据长度字段值: {packed_data[2]} (十进制), 0x{packed_data[2]:02x} (十六进制)")

    # data_to_verify = packed_data[1:-2]
    # # 打印出数据段的内容
    # print(f"用于计算校验和的数据段 (十六进制): {data_to_verify.hex()}")
    # # 计算并打印校验和
    # calculated_checksum = PC_CalculateChecksum(data_to_verify)
    # print(f"计算出的校验和 (十进制): {calculated_checksum}")
    # print(f"计算出的校验和 (十六进制): {hex(calculated_checksum)}")

    # # 5. 验证帧头和帧尾
    # print(f"帧头: {hex(packed_data[0])}")
    # print(f"帧尾: {hex(packed_data[-1])}")
    packed_data = Simulate_data()
    print(f"打包好的数据: {packed_data.hex()}")
    
    
    
    
if __name__ == "__main__":
    main()