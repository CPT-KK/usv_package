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

# ROS 定频
ROS_RATE = 10

# 常量
RPM_START = -160

L_HALF = 1.75

USP_GOINT_OUT = 1.5
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
DIST_TONEXT_DOCK_MEASURE = 10.0          # DOCK_MEASURE 时切换追踪点为轨迹下一点的距离
ANGLE_DOCK_MEASURE_JUMP = deg2rad(20.0) # DOCK_MEASURE 时认为激光雷达估计目标船朝向可能跳变的角度判据

USP_DOCK_APPROACH_UB = 1.5              # DOCK_APPROACH 时 USV 的轴向速度上界
USP_DOCK_APPROACH_LB = 1.1              # DOCK_APPROACH 时 USV 的轴向速度下界
DIST_TONEXT_DOCK_APPROACH = 10.0         # DOCK_APPROACH 时切换追踪点为轨迹下一点的距离

SECS_WAIT_DOCK_STEADY = 5.0      # DOCK_STEADY 时认为 USV 已经稳定前所需的秒数
SECS_TIMEOUT_DOCK_STEADY = 30.0
ANGLE_DOCK_STEADY_TOL = deg2rad(5)      # DOCK_STEADY 时认为 USV 已经稳定的角度判据
DIST_DOCK_STEADY_TOL = 2             # DOCK_STEADY 时认为 USV 已经稳定的位置判据
VEL_DOCK_STEADY_TOL = 0.4              # DOCK_STEADY 时认为 USV 已经稳定的速度判据

HEALTHY_Z_TOL = 1.2                     # 
SECS_WAIT_HEIGHT_SEARCH = 10.0          # WAIT_ARM 时等待机械臂搜索大物体的秒数

SECS_TIMEOUT_ATTACH = 10.0
SECS_WAIT_ATTACH = 3.0
DIST_ATTACH_TOL = 0.75
RPM_ATTACH = 200.0
RPM_ATTACH_UB = 300.0
RPM_ATTACH_LB = 120.0
ANGLE_LEFT_ATTACH = deg2rad(90.5)
ANGLE_RIGHT_ATTACH = deg2rad(95)

RPM_FINAL = 0.0
ANGLE_LEFT_FINAL = deg2rad(0)
ANGLE_RIGHT_FINAL = deg2rad(0)

@atexit.register 
def clean():
    print(">>>>>>> USV program has exited.")

def interuptFunc(signum, frame):
    console = Console()
    console.print("\n[red]>>>>>>> Ctrl + C pressed! Exiting...")
    exit()

def updateTVHeading(existHeading, newHeading):
    newHeading2 = wrapToPi(newHeading + pi)
    angleGap1 = abs(newHeading - existHeading)
    angleGap2 = abs(newHeading2 - existHeading)

    if (angleGap1 <= angleGap2):
        return wrapToPi(newHeading)
    else:
        return wrapToPi(newHeading2)

def main():
    # 控制台输出初始化
    console = Console()
    latestMsg = "Waiting USV self-check to complete..."
    console.print("[green]>>>>>>> Console initialized.")

    # 注册 Ctrl + C
    signal.signal(signal.SIGINT, interuptFunc)
    signal.signal(signal.SIGTERM, interuptFunc)
    console.print("[green]>>>>>>> Interrupt function initialized.")

    # 添加主节点
    rospy.init_node('usv_test_dock_node', anonymous=True)
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

    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    console.print("[green]>>>>>>> ROS spin started.")
    
    # 初始化标志位
    isObsAvoidEnable = True

    isGoindOutPlan = False
    isDockNearbyPlan = False
    isDockMeasurePlan = False
    isDockApproachPlan = False
    isDockAdjustPlan = False
    isDockWaitArmPlan = False
    isDockToObjAreaPlan = False
    isDockToVesselPlan = False
    isDockAttachPlan = False
    isTestPlan = False

    isTestEnable = False

    # 无人船状态
    usvState = "DOCK_STEADY"

    # 无人船当前正在使用的路径
    currPath = zeros((2000, 2))

    # 记录程序启动时间
    t0 = rospy.Time.now().to_sec()

    # 设置计时器
    timer0 = rospy.Time.now().to_sec()
    timer1 = rospy.Time.now().to_sec()

    # 保存目标船朝向角的数组
    tvInfo = zeros((3, 5000))
    tvInfoIdx = 0
    tvHighestXYZs = zeros((3, 5000))
    tvHighestInfoIdx = 0

    # 大物体
    largeObjX = float("nan")
    largeObjY = float("nan")

    # Set point 量
    uSP = float("nan")
    vSP = float("nan")
    yawSP = float("nan")
    rSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")
    axbSP = float("nan")
    aybSP = float("nan")
    etaSP = float("nan")

    while (True):
        # 单独为激光雷达设置启动检查
        pubTopicList = sum(rospy.get_published_topics(), [])
        usvPose.isLidarValid = ('/filter/target' in pubTopicList)

        # 打印当前状态
        dt = rospy.Time.now().to_sec() - t0
        theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
        console.print(theTable)

        if (usvPose.isImuValid) & (usvPose.isDvlValid) & (usvPose.isPodValid) & (not isnan(usvControl.angleLeftEst)) & (not isnan(usvControl.angleRightEst)) & (not isnan(usvControl.rpmLeftEst) & (not isnan(usvControl.rpmRightEst))):
            break
    
    usvPose.isLidarFindTVPrevious = True

    while (not usvPose.isLidarFindTV):
        # 打印当前状态
        dt = rospy.Time.now().to_sec() - t0
        theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
        console.print(theTable)

        rosRate.sleep()

    semiFinalX = usvPose.xLidar
    semiFinalY = usvPose.yLidar
    finalyaw = usvPose.yaw
    tvLengthMean = 12.5
    tvWidthMean = 4.5

    deckCenterX = 0
    deckCenterY = 0
    deckyaw = 0
    while (not rospy.is_shutdown()):
        # 打印当前状态
        dt = rospy.Time.now().to_sec() - t0
        theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
        console.print(theTable)

        # 写入当前状态到文件
        usvData.saveData(usvState, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP)

        # 发送无人船的状态
        usvComm.sendUSVState(usvState)

        # 发送小物体搬运所需
        usvComm.sendTVPosFromLidar(deckCenterX, deckCenterY, deckyaw)
        
        if usvState == "DOCK_STEADY":
            if (isDockAdjustPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                # 使用上一段路径的最后一个点作为自稳点
                # 使用上一段路径最后两个点的切线方向作为 USV 航向
                
                isDockAdjustPlan = True

            latestMsg = f"Try to stablize at [{xSP:.2f}, {ySP:.2f}]m, {rad2deg(yawSP):.2f} deg. Err/Tol: [{sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2):.2f}/{DIST_DOCK_STEADY_TOL:.2f}]m."
            
            # "Dur/Tol/Timeout: [{rospy.Time.now().to_sec() - timer1:.2f}/{SECS_WAIT_DOCK_STEADY:.2f}/{rospy.Time.now().to_sec() - timer0:.2f}]s."

            # 更新航向值
            finalyaw = updateTVHeading(finalyaw, usvPose.tvHeading)

            # 保持静止
            xSP = 0 + (0.5 * tvWidthMean + L_HALF) * cos(finalyaw - pi / 2)
            ySP = 0 + (0.5 * tvWidthMean + L_HALF) * sin(finalyaw - pi / 2)
            yawSP = finalyaw
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, yawSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.yaw, usvPose.r)
            
            # 一旦船靠近到阈值以下范围，进入固连
            if (sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2) <= DIST_DOCK_STEADY_TOL):   
                usvState = "DOCK_ATTACH"
                continue

            # 超时
            if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_DOCK_STEADY):
                usvState = "DOCK_ATTACH"
                continue
        
        elif usvState == "DOCK_ATTACH":
            if (isDockAttachPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                finalyaw = updateTVHeading(finalyaw, usvPose.tvHeading)
                [tvXBody, _] = rotationZ(usvPose.tvX, usvPose.tvY, finalyaw)

                if (tvXBody >= 0):
                    isThrustPositive = True
                else:
                    isThrustPositive = False

                isDockAttachPlan = True

            # 计算目标船的在无人船船体系下坐标
            finalyaw = updateTVHeading(finalyaw, usvPose.tvHeading)
            [tvXBody, tvYBody] = rotationZ(usvPose.tvX, usvPose.tvY, usvPose.yaw)
            lateralDist = abs(tvYBody)

            # 计算推力方向并根据推力正负限制在±90之间，第一次是正方向推力（0-90度），以后都为正，反之亦然
            thrustAngle = arctan(tvYBody / tvXBody)
            if (thrustAngle < 0) & (isThrustPositive):
                thrustAngle = pi / 2
            elif (thrustAngle > 0) & (not isThrustPositive):
                thrustAngle = -pi / 2

            # 计算推力大小
            thrustRPM = linearClip(5, 10, RPM_ATTACH_LB, RPM_ATTACH_UB, usvPose.tvDist) 

            latestMsg = f"Attaching to the target vessel. Tol: [{lateralDist:.2f}/{DIST_ATTACH_TOL + 0.5 * tvWidthMean + L_HALF:.2f}]"

            if (isThrustPositive):
                usvControl.thrustSet(thrustRPM, thrustRPM, thrustAngle, thrustAngle)
            else:
                usvControl.thrustSet(-thrustRPM, -thrustRPM, thrustAngle, thrustAngle)
            usvControl.thrustPub()

            # if (usvControl.angleLeftEst <= deg2rad(89)) | (usvControl.angleRightEst <= deg2rad(89)):
            #     usvControl.thrustSet(0, 0, ANGLE_LEFT_ATTACH, ANGLE_RIGHT_ATTACH)
            # else:
            #     usvControl.thrustSet(RPM_ATTACH, RPM_ATTACH, ANGLE_LEFT_ATTACH, ANGLE_RIGHT_ATTACH)
            # usvControl.thrustPub()

            # 固连成功判据：距离，或者超时
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_ATTACH):   
                usvState = "MEASURE_HIGHEST"
                continue
            elif (tvYBody <= DIST_ATTACH_TOL + 0.5 * tvWidthMean + L_HALF):
                pass
            else:
                # 如果不满足静止条件，需要重置 t1 计时器
                timer1 = rospy.Time.now().to_sec()

            # 超时
            if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_ATTACH):
                usvState = "MEASURE_HIGHEST"
                continue

        elif usvState == "MEASURE_HIGHEST":
            if (isDockWaitArmPlan == False):
                # 将当前时间写入 t1 计时器
                timer1 = rospy.Time.now().to_sec()
                isDockWaitArmPlan = True

            latestMsg = f"Attach finished. Measuring the highest point... [{rospy.Time.now().to_sec() - timer1:.2f} / {SECS_WAIT_HEIGHT_SEARCH:.2f}]s."

            # 记录最高点信息
            tvHighestXYZs[0, tvHighestInfoIdx] = usvPose.tvHighestX
            tvHighestXYZs[1, tvHighestInfoIdx] = usvPose.tvHighestY
            tvHighestXYZs[2, tvHighestInfoIdx] = usvPose.tvHighestZ
            tvHighestInfoIdx = tvHighestInfoIdx + 1

            # 等待测量完成
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_HEIGHT_SEARCH):
                tvHighestXYZs = tvHighestXYZs[:, 0:tvHighestInfoIdx-1]
                tvHighestXs = removeOutliers(tvHighestXYZs[0, :], 0.1, 20)
                tvHighestYs = removeOutliers(tvHighestXYZs[1, :], 0.1, 20)
                tvHighestZs = removeOutliers(tvHighestXYZs[2, :], 0.1, 20)
                tvHighestXMean = mean(tvHighestXs)
                tvHighestYMean = mean(tvHighestYs)
                tvHighestZMean = mean(tvHighestZs)
                if (tvHighestZMean >= HEALTHY_Z_TOL):   
                    # 最高点测量健康，向最高点映射到船中轴线上的点泊近
                    # 注意：这里的 rotationZ 是要对点（向量）进行旋转，即求取点在旋转后的坐标（同一坐标系下），
                    # 而不是同一个点在不同坐标系下的表示，故取负号
                    [tvHighestXMean2, _] = rotationZ(tvHighestXMean, tvHighestYMean, finalyaw)
                    if (tvHighestXMean2 >= 0):
                        finalX = (-0.5 * tvLengthMean + tvHighestXMean2) / 2
                        finalY = 0.0
                    else:
                        finalX = (0.5 * tvLengthMean + tvHighestXMean2) / 2
                        finalY = 0.0
                    [finalX, finalY] = rotationZ(finalX, finalY, -finalyaw)
                else: 
                    # 最高点测量不健康，设置为目标船中心
                    finalX = 0.0
                    finalY = 0.0

                usvState = "DOCK_FINAL"
                continue
            else:
                pass

        elif usvState == "DOCK_FINAL":
            # DOCK_FINAL 是一个死循环
                
            latestMsg = f"Attached completed. Take-off signal for tUAV has been sent. Real-time deck point [{deckCenterX:.2f}, {deckCenterY:.2f}]m {rad2deg(deckyaw):.2f}deg"
            
            usvComm.sendTakeOffFlag()

            # 计算给小物体搬运的坐标点
            [deckCenterX, deckCenterY] = rotationZ(-usvPose.xLidar + finalX, -usvPose.yLidar + finalY, usvPose.yaw)
            deckyaw = finalyaw - usvPose.yaw
            
            # 保持一定的推力
            usvControl.thrustSet(RPM_FINAL, RPM_FINAL, ANGLE_LEFT_FINAL, ANGLE_RIGHT_FINAL)
            usvControl.thrustPub()

        rosRate.sleep()

if __name__ == '__main__':
    main()