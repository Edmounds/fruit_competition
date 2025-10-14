include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER, -- 地图构建器配置
  trajectory_builder = TRAJECTORY_BUILDER, -- 轨迹构建器配置
  map_frame = "map", -- 地图坐标系名称
  tracking_frame = "imu_1", 
  published_frame = "base_link", -- 发布的坐标系（通常为机器人底盘）
  odom_frame = "odom", -- 里程计坐标系名称
  provide_odom_frame = true, -- 是否由Cartographer发布odom坐标系
  publish_frame_projected_to_2d = false, -- 是否将发布的frame投影到2D平面
  use_pose_extrapolator = true, -- 是否使用位姿外推器
  use_odometry = false, -- 是否使用里程计数据
  use_nav_sat = false, -- 是否使用GPS数据
  use_landmarks = false, -- 是否使用地标数据
  num_laser_scans = 1, -- 激光雷达数量
  num_multi_echo_laser_scans = 0, -- 多回波激光雷达数量
  num_subdivisions_per_laser_scan = 1, -- 每帧激光分割的子帧数
  num_point_clouds = 0, -- 点云传感器数量
  lookup_transform_timeout_sec = 0.2, -- 坐标变换查找超时时间（秒）
  submap_publish_period_sec = 0.3, -- 子地图发布周期（秒）
  pose_publish_period_sec = 5e-3, -- 位姿发布周期（秒）
  trajectory_publish_period_sec = 30e-3, -- 轨迹发布周期（秒）
  rangefinder_sampling_ratio = 1., -- 激光雷达数据采样率
  odometry_sampling_ratio = 1., -- 里程计数据采样率
  fixed_frame_pose_sampling_ratio = 1., -- 固定坐标系位姿采样率
  imu_sampling_ratio = 1., -- IMU数据采样率
  landmarks_sampling_ratio = 1., -- 地标数据采样率
  publish_tracked_pose = true -- 是否发布跟踪的位姿
}

MAP_BUILDER.num_background_threads = 6
MAP_BUILDER.use_trajectory_builder_2d = true -- 使用2D轨迹构建器
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- 每次累积多少帧激光数据进行处理
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 是否使用IMU数据
TRAJECTORY_BUILDER_2D.min_range = 0.13  -- 激光最小有效距离（米），需根据实际LiDAR调整
TRAJECTORY_BUILDER_2D.max_range = 50.0 -- 激光最大有效距离（米），需根据实际LiDAR调整
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0 -- 缺失数据射线长度（米）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 是否使用在线相关性扫描匹配

-- 运动滤波器最大角度变化（弧度），1.0改成0.1，提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)


-- 实时相关性扫描匹配器参数
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2 -- 线性搜索窗口（米）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10. -- 平移代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1 -- 旋转代价权重

POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- Huber损失函数尺度
POSE_GRAPH.optimize_every_n_nodes = 6 -- 每累计多少节点进行一次全局优化
-- Fast correlative scan matcher的最低分数，高于此分数才进行优化，0.55改成0.65
POSE_GRAPH.constraint_builder.min_score = 0.65

-- 全局定位最小分数，低于此分数则认为目前全局定位不准确
-- 降低分数可加快重定位，但需注意可能引入错误
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 设置为0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options