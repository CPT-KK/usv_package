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
USP_GOINT_OUT = 1.5
SECS_GOING_OUT = 8

USP_SUAV_PURSUE = 2.75                  # 搜索无人机引导时 USV 的轴向速度
ANGLE_EST_POD_GAP = deg2rad(15)
USP_POD_PURSUE = 2.75                    # 吊舱引导时 USV 的轴向速度
DIST_ALLOW_POD = 500.0                  # 吊舱引导时允许的吊舱距离

USP_LIDAR_PURSUE_UB = 2.75               # 激光雷达引导时 USV 的轴向速度上界
USP_LIDAR_PURSUE_LB = 1.7               # 激光雷达引导时 USV 的轴向速度下界
DIST_LIDAR_PURSUE_UB = 100.0            # 激光雷达引导时取到 USP_LIDAR_PURSUE_UB 的 USV-TV 距离
DIST_LIDAR_PURSUE_LB = 75.0             # 激光雷达引导时取到 USP_LIDAR_PURSUE_LB 的 USV-TV 距离
DIST_PURSUE_TO_APPROACH = 70.0          # 由 PURSUE 切换到 DOCK_NEARBY 的 USV-TV 距离

USP_OBS_PURSUE = 2.75                   # 避障时 USV 的轴向速度
ANGLE_AVOID_OBS = deg2rad(35.0)         # 避障时 USV 的航向附加量

USP_DOCK_NEARBY = 2                   # DOCK_NEARBY 时 USV 的轴向速度
DIST_TONEXT_DOCK_NEARBY = 12.0          # DOCK_NEARBY 时切换追踪点为轨迹下一点的距离

USP_DOCK_MEASURE = 1.8                  # DOCK_MEASURE 时 USV 的轴向速度
DIST_TONEXT_DOCK_MEASURE = 10.0          # DOCK_MEASURE 时切换追踪点为轨迹下一点的距离
ANGLE_DOCK_MEASURE_JUMP = deg2rad(20.0) # DOCK_MEASURE 时认为激光雷达估计目标船朝向可能跳变的角度判据

USP_DOCK_APPROACH_UB = 1.8              # DOCK_APPROACH 时 USV 的轴向速度上界
USP_DOCK_APPROACH_LB = 1.6              # DOCK_APPROACH 时 USV 的轴向速度下界
DIST_TONEXT_DOCK_APPROACH = 10.0         # DOCK_APPROACH 时切换追踪点为轨迹下一点的距离

SECS_WAIT_DOCK_ADJUST_STEADY = 5.0      # DOCK_ADJUST 时认为 USV 已经稳定前所需的秒数
SECS_TIMEOUT_DOCK_ADJUST_STEADY = 30.0
ANGLE_DOCK_STEADY_TOL = deg2rad(2)      # DOCK_ADJUST 时认为 USV 已经稳定的角度判据
DIST_DOCK_STEADY_TOL = 2.5             # DOCK_ADJUST 时认为 USV 已经稳定的位置判据
VEL_DOCK_STEADY_TOL = 0.4              # DOCK_ADJUST 时认为 USV 已经稳定的速度判据

HEALTHY_Z_TOL = 1.5                     # 
SECS_WAIT_HEIGHT_SEARCH = 10.0          # WAIT_ARM 时等待机械臂搜索大物体的秒数

DIST_TOOBJAREA_SIDE = 3.5              # TOLARGEOBJ 时 USV 前往的大物体侧面点与船边的距离
SECS_WAIT_TOOBJAREA_STEADY = 5.0       # TOLARGEOBJ 时认为 USV 已经稳定前所需的秒数
SECS_TIMEOUT_TOOBJAREA_STEADY = 30.0
DIST_TOLARGEOBJ_TOL = 2               # TOLARGEOBJ 时认为 USV 已经前往到大物体侧面点的位置判据

DIST_TOVESSELCEN_SIDE = 3.5                # TOVESSEL 时 USV 前往的目标船侧面点与船边的距离
SECS_WAIT_TOVESSCEN_STEADY = 5.0         # TOVESSEL 时认为 USV 已经稳定前所需的秒数
SECS_TIMEOUT_TOVESSCEN_STEADY = 30.0
DIST_TOVESSEL_TOL = 2                # TOVESSEL 时认为 USV 已经前往到目标船侧面点的位置判据

SECS_WAIT_ATTACH_STEADY = 5.0
SECS_TIMEOUT_ATTACH_STEADY = 30.0
VEL_ATTACH_TOL = 0.08
DIST_ATTACH_TOL = 1.5

RPM_ATTACH_UB = 400.0       
RPM_ATTACH_LB = 150.0
DIST_ATTACH_UB = 10.0
DIST_ATTACH_LB = 5.0

RPM_ATTACH_FAILSAFE = 360.0

RPM_FINAL = 280.0

@atexit.register 
def clean():
    print(">>>>>>> USV program has exited.")

def interuptFunc(signum, frame):
    console = Console()
    console.print("\n[red]>>>>>>> Ctrl + C pressed! Exiting...")
    exit()


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
    usvState = "DOCK_ADJUST"

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
    psiSP = float("nan")
    rSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")
    axbSP = float("nan")
    aybSP = float("nan")
    etaSP = float("nan")

    usvPose.isLidarFindTVPrevious = True

    while (not usvPose.isLidarFindTV):
        rosRate.sleep()

    semiFinalX = usvPose.xLidar
    semiFinalY = usvPose.yLidar
    finalPsi = usvPose.psi
    tvLengthMean = 12
    tvWidthMean = 4.5

    while (not rospy.is_shutdown()):
        # 打印当前状态
        dt = rospy.Time.now().to_sec() - t0
        theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, psiSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
        console.print(theTable)

        if usvState == "DOCK_ADJUST":
            if (isDockAdjustPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                # 使用上一段路径的最后一个点作为自稳点
                # 使用上一段路径最后两个点的切线方向作为 USV 航向
                xSP = semiFinalX
                ySP = semiFinalY
                psiSP = finalPsi
                isDockAdjustPlan = True

            latestMsg = f"Approach finished. Stablizing USV @ [{xSP:.2f}, {ySP:.2f}], {rad2deg(psiSP):.2f} deg... [{rospy.Time.now().to_sec() - timer1:.2f} / {SECS_WAIT_DOCK_ADJUST_STEADY:.2f}]s"

            # 保持静止
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            # 等待船接近静止并保持 5.0s，进入 MEASURE_HIGHEST
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_DOCK_ADJUST_STEADY):   
                usvState = "MEASURE_HIGHEST"
                continue
            elif (abs(usvPose.psi - psiSP) < ANGLE_DOCK_STEADY_TOL) & (abs(usvPose.xLidar - xSP) < DIST_DOCK_STEADY_TOL) & (abs(usvPose.yLidar - ySP) < DIST_DOCK_STEADY_TOL) & (abs(usvPose.uDVL) < VEL_DOCK_STEADY_TOL) & (abs(usvPose.vDVL) < VEL_DOCK_STEADY_TOL):
                pass
            else:
                # 如果不满足静止条件，需要重置 t1 计时器
                timer1 = rospy.Time.now().to_sec()

            # 超时
            if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_DOCK_ADJUST_STEADY):
                usvState = "MEASURE_HIGHEST"
                continue

        elif usvState == "MEASURE_HIGHEST":
            if (isDockWaitArmPlan == False):
                # 将当前时间写入 t1 计时器
                timer1 = rospy.Time.now().to_sec()

                xSP = semiFinalX
                ySP = semiFinalY
                psiSP = finalPsi

                isDockWaitArmPlan = True

            latestMsg = f"USV has been stablized. Measuring the highest point... [{rospy.Time.now().to_sec() - timer1:.2f} / {SECS_WAIT_HEIGHT_SEARCH:.2f}]s."

            tvHighestXYZs[0, tvHighestInfoIdx] = usvPose.tvHighestX
            tvHighestXYZs[1, tvHighestInfoIdx] = usvPose.tvHighestY
            tvHighestXYZs[2, tvHighestInfoIdx] = usvPose.tvHighestZ
            tvHighestInfoIdx = tvHighestInfoIdx + 1
            
            # 保持静止
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

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
                    [tvHighestXMean2, _] = rotationZ(tvHighestXMean, tvHighestYMean, finalPsi)
                    if (tvHighestXMean2 >= 0):
                        finalX = (-0.5 * tvLengthMean + tvHighestXMean2) / 2
                        finalY = 0.0
                    else:
                        finalX = (0.5 * tvLengthMean + tvHighestXMean2) / 2
                        finalY = 0.0
                    [finalX, finalY] = rotationZ(finalX, finalY, -finalPsi)
                    usvState = "DOCK_TOOBJAREA"
                    continue
                else: 
                    # 最高点测量不健康，向目标船中心泊近
                    finalX = 0.0
                    finalY = 0.0
                    usvState = "DOCK_TOVESSCEN"
                    continue
            else:
                pass

        elif usvState == "DOCK_TOOBJAREA":
            if (isDockToObjAreaPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                # 设置目标点为无人船对齐目标区域侧面那个点
                xSP = finalX + (DIST_TOOBJAREA_SIDE + 0.5 * tvWidthMean) * cos(finalPsi - pi / 2)
                ySP = finalY + (DIST_TOOBJAREA_SIDE + 0.5 * tvWidthMean) * sin(finalPsi - pi / 2)
                psiSP = finalPsi
                
                isDockToObjAreaPlan = True

                latestMsg = f"USV is aligning with the estimated object area center [{xSP:.2f}, {ySP:.2f}] [{rospy.Time.now().to_sec() - timer1:.2f} / {SECS_WAIT_HEIGHT_SEARCH:.2f}]..."
            
            # 向目标区域对齐
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)
            
            # 如果与目标区域的轴向误差（？）小于给定距离并且持续 X 秒，则认为已经和目标区域对齐
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_TOOBJAREA_STEADY): 
                usvState = "DOCK_ATTACH"
                continue
            elif (sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2) < DIST_TOLARGEOBJ_TOL):
                pass
            else:
                # 如果不满足静止条件，需要重置 t1 计时器
                timer1 = rospy.Time.now().to_sec()

            # 超时
            if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_TOOBJAREA_STEADY):
                usvState = "DOCK_ATTACH"
                continue

        elif usvState == "DOCK_TOVESSCEN": 
            if (isDockToVesselPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                # 设置目标点为目标船的中心 
                xSP = finalX + (DIST_TOVESSELCEN_SIDE + 0.5 * tvWidthMean) * cos(finalPsi - pi / 2)
                ySP = finalY + (DIST_TOVESSELCEN_SIDE + 0.5 * tvWidthMean) * sin(finalPsi - pi / 2)
                psiSP = finalPsi

                isDockToVesselPlan = True

                latestMsg = f"Failed to estimate the object area center. USV is aligning with the center of the target vessel [{rospy.Time.now().to_sec() - timer1:.2f} / {SECS_WAIT_HEIGHT_SEARCH:.2f}]..."

            # 向目标船中心对齐                 
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            # 如果与目标船的距离小于给定距离并且持续 X 秒，则认为已经和目标船中心对齐
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_TOVESSCEN_STEADY): 
                usvState = "DOCK_ATTACH"
                continue
            elif (sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2) < DIST_TOVESSEL_TOL):
                pass
            else:
                # 如果不满足静止条件，需要重置 t1 计时器
                timer1 = rospy.Time.now().to_sec()

            # 超时
            if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_TOVESSCEN_STEADY):
                usvState = "DOCK_ATTACH"
                continue

        elif usvState == "DOCK_ATTACH":
            if (isDockAttachPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                # 设置目标点为目标船/大物体的中心
                xSP = finalX + (0.4 * tvWidthMean) * cos(finalPsi - pi / 2)
                ySP = finalY + (0.4 * tvWidthMean) * sin(finalPsi - pi / 2)
                psiSP = finalPsi

                isDockAttachPlan = True

            latestMsg = f"Close enough. Try to attach. Need to stablize for [{rospy.Time.now().to_sec() - timer1:.2f} / {SECS_WAIT_ATTACH_STEADY:.2f}]s"
            
            # 横向移动向大物体/目标船     
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            # 如果 USV 侧向速度小于 VEL_ATTACH_TOL，或者和给定点距离小于DIST_ATTACH_TOL，并且持续 SECS_WAIT_ATTACH_STEADY 秒，则认为已经固连
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_ATTACH_STEADY): 
                usvState = "DOCK_FINAL"
                continue
            elif (sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2) < DIST_ATTACH_TOL) | (abs(usvPose.vDVL) < VEL_ATTACH_TOL):
                pass
            else:
                # 如果不满足静止条件，需要重置 t1 计时器
                timer1 = rospy.Time.now().to_sec()

            # 超时
            if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_ATTACH_STEADY):
                isDockAttachPlan = False
                usvState = "DOCK_ATTACH_FAILSAFE"
                continue

        elif usvState == "DOCK_ATTACH_FAILSAFE":
            if (isDockAttachPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()

                isDockAttachPlan = True

            tvAngleLidarBody = usvPose.tvAngleLidar - usvPose.psi
            if (usvControl.angleLeftEst <= deg2rad(89)) | (usvControl.angleRightEst <= deg2rad(89)):
                usvControl.thrustSet(0, 0, tvAngleLidarBody + deg2rad(2), tvAngleLidarBody + deg2rad(4))
            else:
                usvControl.thrustSet(RPM_ATTACH_FAILSAFE, RPM_ATTACH_FAILSAFE, tvAngleLidarBody + deg2rad(2), tvAngleLidarBody + deg2rad(4))

            if (sqrt((usvPose.xLidar - xSP) ** 2 + (usvPose.yLidar - ySP) ** 2) < DIST_ATTACH_TOL): 
                usvState = "DOCK_FINAL"
                continue

        elif usvState == "DOCK_FINAL":
            # DOCK_FINAL 是一个死循环
                
            latestMsg = "Attached completed. Take-off signal for tUAV has been sent."
            usvComm.sendTakeOffFlag()
            [deckCenterX, deckCenterY] = rotationZ(-usvPose.xLidar + finalX, -usvPose.yLidar + finalY, usvPose.psi)

            usvComm.sendTVPosFromLidar(deckCenterX, deckCenterY, finalPsi - usvPose.psi)

            # 保持一定的推力
            usvControl.thrustSet(RPM_FINAL, RPM_FINAL, deg2rad(92), deg2rad(94))     

if __name__ == '__main__':
    main()