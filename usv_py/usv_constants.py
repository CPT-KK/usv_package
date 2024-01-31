from numpy import zeros, rad2deg, deg2rad, pi, abs, mean, sin, cos, tan, arctan, arctan2, std, sqrt, isnan

# ROS 定频
ROS_RATE = 10

# 常量
RPM_START = -160

L_HALF = 1.75

USP_GOINT_OUT = 1.0
SECS_GOING_OUT = 15

USP_SUAV_PURSUE = 2.75                  # 搜索无人机引导时 USV 的轴向速度
ANGLE_EST_POD_GAP = deg2rad(30)
USP_POD_PURSUE = 2.75                    # 吊舱引导时 USV 的轴向速度
DIST_ALLOW_POD = 400.0                  # 吊舱引导时允许的吊舱距离

USP_LIDAR_PURSUE_UB = 2.75               # 激光雷达引导时 USV 的轴向速度上界
USP_LIDAR_PURSUE_LB = 1.7               # 激光雷达引导时 USV 的轴向速度下界
DIST_LIDAR_PURSUE_UB = 100.0            # 激光雷达引导时取到 USP_LIDAR_PURSUE_UB 的 USV-TV 距离
DIST_LIDAR_PURSUE_LB = 75.0             # 激光雷达引导时取到 USP_LIDAR_PURSUE_LB 的 USV-TV 距离
DIST_PURSUE_TO_APPROACH = 70.0          # 由 PURSUE 切换到 DOCK_NEARBY 的 USV-TV 距离

USP_OBS_PURSUE = 2.75                   # 避障时 USV 的轴向速度
ANGLE_AVOID_OBS = deg2rad(35.0)         # 避障时 USV 的航向附加量

USP_DOCK_NEARBY = 1.8                   # DOCK_NEARBY 时 USV 的轴向速度
DIST_TONEXT_DOCK_NEARBY = 12.0          # DOCK_NEARBY 时切换追踪点为轨迹下一点的距离

USP_DOCK_MEASURE = 1.5                  # DOCK_MEASURE 时 USV 的轴向速度
DIST_TONEXT_DOCK_MEASURE = 12.0          # DOCK_MEASURE 时切换追踪点为轨迹下一点的距离
ANGLE_DOCK_MEASURE_JUMP = deg2rad(20.0) # DOCK_MEASURE 时认为激光雷达估计目标船朝向可能跳变的角度判据
HEALTHY_Z_TOL = 1.2                     # DOCK_MEASURE 时健康的高度阈值

USP_DOCK_APPROACH_UB = 1.5              # DOCK_APPROACH 时 USV 的轴向速度上界
USP_DOCK_APPROACH_LB = 0.8              # DOCK_APPROACH 时 USV 的轴向速度下界
DIST_TONEXT_DOCK_APPROACH = 12.0         # DOCK_APPROACH 时切换追踪点为轨迹下一点的距离

SECS_WAIT_DOCK_STEADY = 5.0      # DOCK_STEADY 时认为 USV 已经稳定前所需的秒数
SECS_TIMEOUT_DOCK_STEADY = 180.0
ANGLE_DOCK_STEADY_TOL = deg2rad(5)      # DOCK_STEADY 时认为 USV 已经稳定的角度判据
DIST_DOCK_STEADY_TOL = 1.25             # DOCK_STEADY 时认为 USV 已经稳定的位置判据
VEL_DOCK_STEADY_TOL = 0.3              # DOCK_STEADY 时认为 USV 已经稳定的速度判据

SECS_TIMEOUT_ATTACH = 30.0
SECS_WAIT_ATTACH = 3.0
DIST_ATTACH_TOL = 0.5
RPM_ATTACH = 450.0
RPM_ATTACH_UB = 500.0
RPM_ATTACH_LB = 120.0
ANGLE_LEFT_ATTACH = deg2rad(105)
ANGLE_RIGHT_ATTACH = deg2rad(96)

SECS_WAIT_FINAL = 10.0
SECS_TIMEOUT_WAIT_FINAL = 30.0
DIST_WAIT_FINAL = 8.0
VEL_WAIT_FINAL = 0.35

RPM_FINAL = 80.0
ANGLE_LEFT_FINAL = deg2rad(105)
ANGLE_RIGHT_FINAL = deg2rad(96)