# Header for timestamp
std_msgs/Header header

# Detection result
bool is_ripe         # 是否成熟
int32 fruit_type     # 水果类型 (1=辣椒, 2=南瓜, ..., 0=未检测到)
float32 offset_x     # X偏移量
float32 offset_y     # Y偏移量