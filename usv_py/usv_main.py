#!/usr/bin/env python3

import rospy
import threading
import atexit
import signal
from rich.console import Console
from numpy import zeros, rad2deg, deg2rad, pi, abs, mean, sin, cos, tan, arctan, arctan2, std, sqrt, isnan

from usv_pose import Pose
from usv_path_planner import PathPlanner
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import removeOutliers, wrapToPi, linearClip, rotationZ
from usv_record import genTable, USVData
from usv_test import test

# 无人船状态定义
    # STARTUP
    # STANDBY
    # PURSUE
    # PURSUE_OBS
    # DOCK_NEARBY
    # DOCK_MEASURE
    # DOCK_APPROACH
    # DOCK_ADJUST
    # DOCK_FINAL
    # ATTACH
    # STOP
# 以下是测试状态
    # TEST_LINE
    # TEST_CIRCLE
    # TEST_BOTH

TEST_MODE = "TEST_LINE"

# ROS 定频
ROS_RATE = 10

# 常量
USP_SUAV_PURSUE = 3.25                  # 搜索无人机引导时 USV 的轴向速度
USP_POD_PURSUE = 3.0                    # 吊舱引导时 USV 的轴向速度

USP_LIDAR_PURSUE_UB = 3.0               # 激光雷达引导时 USV 的轴向速度上界
USP_LIDAR_PURSUE_LB = 1.7               # 激光雷达引导时 USV 的轴向速度下界
DIST_LIDAR_PURSUE_UB = 100.0            # 激光雷达引导时取到 USP_LIDAR_PURSUE_UB 的 USV-TV 距离
DIST_LIDAR_PURSUE_LB = 75.0             # 激光雷达引导时取到 USP_LIDAR_PURSUE_LB 的 USV-TV 距离
DIST_PURSUE_TO_APPROACH = 70.0          # 由 PURSUE 切换到 DOCK_NEARBY 的 USV-TV 距离

USP_OBS_PURSUE = 3.25                   # 避障时 USV 的轴向速度
ANGLE_AVOID_OBS = deg2rad(35.0)         # 避障时 USV 的航向附加量

USP_DOCK_NEARBY = 1.6                   # DOCK_NEARBY 时 USV 的轴向速度
DIST_TONEXT_DOCK_NEARBY = 12.0          # DOCK_NEARBY 时切换追踪点为轨迹下一点的距离

USP_DOCK_MEASURE = 1.5                  # DOCK_MEASURE 时 USV 的轴向速度
DIST_TONEXT_DOCK_MEASURE = 8.0          # DOCK_MEASURE 时切换追踪点为轨迹下一点的距离
ANGLE_DOCK_MEASURE_JUMP = deg2rad(20.0) # DOCK_MEASURE 时认为激光雷达估计目标船朝向可能跳变的角度判据

USP_DOCK_APPROACH_UB = 1.5              # DOCK_APPROACH 时 USV 的轴向速度上界
USP_DOCK_APPROACH_LB = 0.8              # DOCK_APPROACH 时 USV 的轴向速度下界
DIST_TONEXT_DOCK_APPROACH = 6.0         # DOCK_APPROACH 时切换追踪点为轨迹下一点的距离

SECS_WAIT_DOCK_ADJUST_STEADY = 5.0      # DOCK_ADJUST 时认为 USV 已经稳定前所需的秒数
ANGLE_DOCK_STEADY_TOL = deg2rad(2)      # DOCK_ADJUST 时认为 USV 已经稳定的角度判据
DIST_DOCK_STEADY_TOL = 1.0              # DOCK_ADJUST 时认为 USV 已经稳定的位置判据
VEL_DOCK_STEADY_TOL = 0.25              # DOCK_ADJUST 时认为 USV 已经稳定的速度判据

SECS_WAIT_ARM_SEARCH = 10.0             # WAIT_ARM 时等待机械臂搜索大物体的秒数

DIST_TOLARGEOBJ_SIDE = 4.0              # TOLARGEOBJ 时 USV 前往的大物体侧面点与大物体的距离
SECS_WAIT_TOLARGEOBJ_STEADY = 5.0       # TOLARGEOBJ 时认为 USV 已经稳定前所需的秒数
DIST_TOLARGEOBJ_TOL = 1.5               # TOLARGEOBJ 时认为 USV 已经前往到大物体侧面点的位置判据

DIST_TOVESSEL_SIDE = 4.0
SECS_WAIT_TOVESSEL_STEADY = 5.0
DIST_TOVESSEL_TOL = 1.5

SECS_WAIT_ATTACH_STEADY = 5.0
VEL_ATTACH_TOL = 0.15

RPM_ATTACH_UB = 400.0
RPM_ATTACH_LB = 150.0
DIST_ATTACH_UB = 10.0
DIST_ATTACH_LB = 5.0

RPM_FINAL = 280.0

@atexit.register 
def clean():
    print(">>>>>>> USV program has exited.")

def interuptFunc(signum, frame):
    console = Console()
    console.print("\n[red]>>>>>>> Ctrl + C pressed! Exiting...")
    exit()

def main(args=None):
    # 控制台输出初始化
    console = Console()
    latestMsg = "Waiting USV self-check to complete..."
    console.print("[green]>>>>>>> Console initialized.")

    # 注册 Ctrl + C
    signal.signal(signal.SIGINT, interuptFunc)
    signal.signal(signal.SIGTERM, interuptFunc)
    console.print("[green]>>>>>>> Interrupt function initialized.")

    # 添加主节点
    rospy.init_node('usv_main_node', anonymous=True)
    rosRate = rospy.Rate(ROS_RATE)
    console.print("[green]>>>>>>> ROS node initialized.")

    # 添加功能类
    # usvCAN = USVCAN()
    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control()
    usvData = USVData()
    console.print("[green]>>>>>>> Function classes initialized.")

    # 添加服务节点
    # console.print("[green]>>>>>>> Waiting MAVROS services...")
    # rospy.wait_for_service("/mavros/cmd/arming")
    # mavrosArmClient = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    # armCmd = CommandBoolRequest(value=True)
    # disarmCmd = CommandBoolRequest(value=False)

    # rospy.wait_for_service("/mavros/set_mode")
    # mavrosSetModeClient = rospy.ServiceProxy("mavros/set_mode", SetMode)
    # holdSetMode = SetModeRequest(custom_mode='AUTO.LOITER')
    # console.print("[green]>>>>>>> Connect to MAVROS services.")

    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    console.print("[green]>>>>>>> ROS spin started.")
    
    # 初始化标志位
    isObsAvoidEnable = True
    isDockNearbyPlan = False
    isDockMeasurePlan = False
    isDockApproachPlan = False
    isDockAdjustPlan = False
    isDockWaitArmPlan = False
    isDockToPlan = False
    isDockAttachPlan = False
    isTestPlan = False
    isTestEnable = False

    # 无人船状态
    usvState = "STARTUP"

    # 无人船当前正在使用的路径
    currPath = zeros((2000, 2))

    # 记录程序启动时间
    t0 = rospy.Time.now().to_sec()

    # 设置计时器
    timer1 = rospy.Time.now().to_sec()

    # 保存目标船朝向角的数组``
    tvHeadings = zeros((1, 5000))
    tvHeadingIdx = 0
    tvLength = float("nan")
    tvWidth = float("nan")

    # Set point 量
    uSP = float("nan")
    vSP = float("nan")
    psiSP = float("nan")
    rSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")
    axbSP = float("nan")
    aybSP = float("nan")
    etaSP = float("nan")

    # 试一下  
    while not rospy.is_shutdown():
        try:
            if usvState == "STARTUP":
                # 单独为激光雷达设置启动检查
                pubTopicList = sum(rospy.get_published_topics(), [])
                usvPose.isLidarValid = ('/filter/target' in pubTopicList)
                    
                if (usvPose.isImuValid) & (usvPose.isDvlValid) & (usvPose.isPodValid) & (usvPose.isLidarValid) & (not isnan(usvControl.angleLeftEst)) & (not isnan(usvControl.angleRightEst)) & (not isnan(usvControl.rpmLeftEst) & (not isnan(usvControl.rpmRightEst))):
                    latestMsg = "Waiting sUAV to send heading..."
                    usvState = "STANDBY"

            elif usvState == "STANDBY":
                if (isTestEnable):
                    usvState = "DOCK_TOLARGEOBJ"
                    continue
                    
                if (usvComm.isSearchFindTV):
                    latestMsg = "Receive heading %.2f deg from sUAV." % rad2deg(usvComm.tvAngleEst)
                    usvState = "PURSUE"

            elif usvState == "PURSUE":   
                # 如果距离小于给定值，则进入 Approach 段
                if (usvPose.isLidarFindTV) & (usvPose.tvDist < DIST_PURSUE_TO_APPROACH):
                    usvState = "DOCK_NEARBY"
                    latestMsg = "Approaching to the measure circle..."
                    continue        
                 
                if (usvPose.isLidarFindTV):
                    # 如果激光雷达找到目标船，则使用激光雷达的信息
                    latestMsg = "Lidar finds target vessel at %.2f deg in %.2f m!" % (rad2deg(usvPose.tvAngleLidar), usvPose.tvDist)
                    uSP = linearClip(DIST_LIDAR_PURSUE_LB, USP_LIDAR_PURSUE_LB, DIST_LIDAR_PURSUE_UB, USP_LIDAR_PURSUE_UB, usvPose.tvDist)
                    psiSP = usvPose.tvAngleLidar

                elif (usvPose.isPodFindTV):
                    # 如果激光雷达没有找到目标船，则使用吊舱的信息
                    latestMsg = "Pod finds target vessel at %.2f deg!" % rad2deg(usvPose.tvAnglePod)
                    uSP = USP_POD_PURSUE
                    psiSP = usvPose.tvAnglePod

                elif (usvPose.isLidarFindObs) & (isObsAvoidEnable):
                    # 遇到障碍物，避障   
                    usvState = "PURSUE_OBS"
                    continue 

                else:
                    # 如果没有找到目标船，则继续跟随追踪路径
                    latestMsg = "Receive heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)
                    uSP = USP_SUAV_PURSUE
                    psiSP = usvComm.tvAngleEst          
                
                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)
                
            elif usvState == "PURSUE_OBS":             
                # 判断是否还需要避障
                if (usvPose.isLidarFindObs):       
                    # 计算避障所需航向角
                    psiSP = usvPose.obsAngleLidar - ANGLE_AVOID_OBS   
                    uSP = USP_OBS_PURSUE
                    [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                    latestMsg = "Obstacle detected at %.2f deg!" % usvPose.obsAngleLidar
                else:
                    latestMsg = "Back to follow heading %.2f deg from sUAV." % rad2deg(usvComm.tvAngleEst)     
                    usvState = "PURSUE"
                    continue
        
            elif usvState == "DOCK_NEARBY":
                if (isDockNearbyPlan == False):
                    currPath = usvPathPlanner.planDockNearby(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockNearbyPlan = True       
                
                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = USP_DOCK_NEARBY
                [psiSP, xSP, ySP] = usvGuidance.guidance(DIST_TONEXT_DOCK_NEARBY, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_MEASURE"
            
            elif usvState == "DOCK_MEASURE":             
                # 使用激光雷达读取的位置信息，规划测量路径
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockMeasurePlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = USP_DOCK_MEASURE
                [psiSP, xSP, ySP] = usvGuidance.guidance(DIST_TONEXT_DOCK_MEASURE, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 读取目标船的测量信息，若满足要求，则读取并保存目标船朝向角（ENU下）
                thisHeading = usvPose.tvHeading
                tvHeadings[0, tvHeadingIdx] = thisHeading
                tvHeadingIdx = tvHeadingIdx + 1
                latestMsg = "Estimating target vessel heading: %.2f deg." % rad2deg(thisHeading)

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    # 去除后面未使用的索引
                    tvHeadings = tvHeadings[0, 0:tvHeadingIdx-1]

                    # 如果标准差大于 ANGLE_DOCK_MEASURE_JUMP，认为大概率是-90°与90°跳变的情况，因此对 tvHeadings 中负角度加一个 pi
                    if (std(tvHeadings) > ANGLE_DOCK_MEASURE_JUMP):
                        tvHeadings[tvHeadings < 0] = tvHeadings[tvHeadings < 0] + pi

                    # 去除离群点
                    tvHeadings = removeOutliers(tvHeadings)
                    
                    # 计算平均值，并将结果角度映射到-90°~90°
                    tvHeadingMean = arctan(tan(mean(tvHeadings)))
                    
                    latestMsg = "Estimating finished with average heading %.2f deg. Begin final approach..." % rad2deg(tvHeadingMean)
                    usvState = "DOCK_APPROACH"

            elif usvState == "DOCK_APPROACH":
                # 使用激光雷达读取的位置信息，规划变轨路径
                if (isDockApproachPlan == False):
                    currPath = usvPathPlanner.planDockApproach(usvPose.xLidar, usvPose.yLidar, 0, 0, tvHeadingMean)
                    usvGuidance.setPath(currPath) 
                    isDockApproachPlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = linearClip(0, USP_DOCK_APPROACH_UB, usvGuidance.endIdx, USP_DOCK_APPROACH_LB, usvGuidance.currentIdx)
                [psiSP, xSP, ySP] = usvGuidance.guidance(DIST_TONEXT_DOCK_APPROACH, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_ADJUST"
                    
                    # 重要：清除 LOS yErrPID 和差分控制器 PID 的积分项
                    usvGuidance.yErrPID.clearIntResult()

            elif usvState == "DOCK_ADJUST":
                if (isDockAdjustPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                    # 使用上一段路径的最后一个点作为自稳点
                    # 使用上一段路径最后两个点的切线方向作为 USV 航向
                    xSP = currPath[-1, 0]
                    ySP = currPath[-1, 1]
                    psiSP = arctan2(ySP - currPath[-2, 1], xSP - currPath[-2, 0])
                    isDockAdjustPlan = True

                latestMsg = "Approach finished. Stablizing USV @ [%.2f, %.2f], %.2f deg... [%.2f / %.2f]s" % (xSP, ySP, rad2deg(psiSP), rospy.Time.now().to_sec() - timer1, 5.0)

                # 保持静止
                [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 等待船接近静止并保持 5.0s，进入 DOCK_WAITARM
                if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_DOCK_ADJUST_STEADY):   
                    usvState = "DOCK_WAITARM"
                elif (abs(usvPose.psi - psiSP) < ANGLE_DOCK_STEADY_TOL) & (abs(usvPose.xLidar - xSP) < DIST_DOCK_STEADY_TOL) & (abs(usvPose.yLidar - ySP) < DIST_DOCK_STEADY_TOL) & (abs(usvPose.uDVL) < VEL_DOCK_STEADY_TOL) & (abs(usvPose.vDVL) < VEL_DOCK_STEADY_TOL):
                    pass
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

            elif usvState == "DOCK_WAITARM":
                if (isDockWaitArmPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                    xSP = currPath[-1, 0]
                    ySP = currPath[-1, 1]
                    psiSP = arctan2(ySP - currPath[-2, 1], xSP - currPath[-2, 0])
                    
                    # 记录这个时候的目标船长宽
                    tvLength = usvPose.tvLength
                    tvWidth = usvPose.tvWidth

                    isDockWaitArmPlan = True

                latestMsg = "USV has been stablized. Waiting the arm searching... [%.2f / %.2f]s" % (rospy.Time.now().to_sec() - timer1, SECS_WAIT_ARM_SEARCH)

                # 保持静止
                [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 等待机械臂搜索大物体
                if (usvComm.isArmFindBigObj):   
                    # 找到大物体，向大物体泊近
                    usvState = "DOCK_TOLARGEOBJ"
                elif (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_ARM_SEARCH): 
                    # 等待机械臂超时，向目标船泊近
                    usvState = "DOCK_TOVESSEL"
                else:
                    pass

            elif usvState == "DOCK_TOLARGEOBJ":
                if (isDockToPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                    # 设置目标点为无人船对齐大物体那个点
                    [xSP, ySP] = rotationZ(usvComm.largeObjX, usvComm.largeObjY, -usvPose.psi)
                    psiSP = arctan2(ySP - currPath[-2, 1], xSP - currPath[-2, 0])
                    xSP = xSP + usvPose.xLidar + (DIST_TOLARGEOBJ_SIDE + 0.6 * tvWidth) * cos(psiSP - pi / 2)
                    ySP = ySP + usvPose.yLidar + (DIST_TOLARGEOBJ_SIDE + 0.6 * tvWidth) * sin(psiSP - pi / 2)
                    
                    isDockToPlan = True

                    latestMsg = "Receive the large object information from the robotic arm. USV is aligning with the large object @ [%.2f, %.2f]..." % (xSP, ySP)
                
                # 向大物体对齐
                [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)
                
                # 如果与大物体的轴向误差（？）小于给定距离并且持续 X 秒，则认为已经和大物体对齐
                if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_TOLARGEOBJ_STEADY): 
                    usvState = "DOCK_ATTACH"
                elif (sqrt((usvPose.x - xSP) ** 2 + (usvPose.y - ySP) ** 2) < DIST_TOLARGEOBJ_TOL):
                    pass
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()
                    
            elif usvState == "DOCK_TOVESSEL": 
                if (isDockToPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                    # 设置目标点为目标船的中心
                    psiSP = arctan2(ySP - currPath[-2, 1], xSP - currPath[-2, 0])
                    xSP = 0 + (DIST_TOVESSEL_SIDE + 0.6 * tvWidth) * cos(psiSP - pi / 2)
                    ySP = 0 + (DIST_TOVESSEL_SIDE + 0.6 * tvWidth) * sin(psiSP - pi / 2)
                    
                    isDockToPlan = True

                    latestMsg = "Timeout waiting the robotic arm to search. USV is aligning with the center of the target vessel..."

                # 向目标船中心对齐                 
                [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 如果与目标船的距离小于给定距离并且持续 X 秒，则认为已经和目标船中心对齐
                if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_TOVESSEL_STEADY): 
                    usvState = "DOCK_ATTACH"
                elif (sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2) < DIST_TOVESSEL_TOL):
                    pass
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

            elif usvState == "DOCK_ATTACH":
                if (isDockAttachPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                latestMsg = "Close enough. Try to attach. Need to stablize for [%.2f / %.2f]s" % (rospy.Time.now().to_sec() - timer1, SECS_WAIT_ATTACH_STEADY)
                
                # 横向移动                   
                thisThrust = linearClip(DIST_ATTACH_LB, RPM_ATTACH_LB, DIST_ATTACH_UB, RPM_ATTACH_UB, usvPose.tvDist)
                usvControl.thrustSet(thisThrust, thisThrust, deg2rad(90), deg2rad(90))       
                usvControl.thrustPub()

                # 如果 USV 速度小于给定值，则认为已经固连
                if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_ATTACH_STEADY): 
                    usvState = "DOCK_FINAL"
                elif (sqrt(usvPose.uDVL** 2 + usvPose.vDVL** 2) < VEL_ATTACH_TOL):
                    pass
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()
            
            elif usvState == "DOCK_FINAL":
                # DOCK_FINAL 是一个死循环
                    
                latestMsg = "Attached completed. Take-off signal for tUAV has been sent."
                usvComm.sendTakeOffFlag()
                usvComm.sendTVPosFromLidar(-usvPose.xLidar, -usvPose.yLidar)

                # 保持一定的推力
                usvControl.thrustSet(RPM_FINAL, RPM_FINAL, deg2rad(90), deg2rad(90)) 
            
            elif usvState == "TEST":              
                if (isTestPlan == False):
                    # Move USV straight left for X m
                    xSP = usvPose.x - 0.0 * cos(usvPose.psi - 0)
                    ySP = usvPose.y - 0.0 * sin(usvPose.psi - 0)
                    psiSP = wrapToPi(usvPose.psi + deg2rad(0))
                    isTestPlan = True

                [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.x, usvPose.y, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            else:
                # 程序不应该执行到这里
                console.print("\n[red] >>>>>>> USV state: %s invalid. Check code." % (usvState))
                break
        
            # 打印当前状态
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
            console.print(theTable)

            # 写入当前状态到文件
            usvData.saveData(usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP)

            # 发送无人船的东西 
            usvComm.sendUSVState(usvState)

            rosRate.sleep()
            
        except Exception as e:
            console.print("\n")
            console.print_exception(show_locals=True)
            console.print("[red]>>>>>>> Unexpected exception caught. Check code.")
            break
    
    # 程序不应该执行到这里
    console.print("[red]>>>>>>> USV program jumped out from the main loop.")
    return

if __name__ == '__main__':
    main()
