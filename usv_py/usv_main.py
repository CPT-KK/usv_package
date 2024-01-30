#!/usr/bin/env python3
import time
import rospy
import threading
import atexit
import signal
from rich.console import Console
from numpy import zeros, rad2deg, deg2rad, pi, abs, mean, sin, cos, tan, arctan, arctan2, std, sqrt, isnan
from numpy.linalg import norm

from usv_pose import Pose
from usv_path_planner import PathPlanner
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import removeOutliers, wrapToPi, linearClip, rotationZ, sign, updateTVHeading, calcHighest

from usv_record import genTable, USVData
from usv_constants import *

# 控制台输出
console = Console(record=True)

# 当前时间
currTimeStamp = time.strftime('%Y%m%d_%H%M%S', time.localtime())

@atexit.register 
def clean():
    fileNameStr = "usv_output_" + currTimeStamp + ".html"
    console.print(">>>>>>> USV program has exited.")
    console.save_html(fileNameStr)

def interuptFunc(signum, frame):
    console.print("\n[red]>>>>>>> Ctrl + C pressed! Exiting...")
    exit()

def main(args=None):
    # 控制台输出初始化
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
    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance()
    usvControl = Control()
    usvData = USVData(currTimeStamp)
    console.print("[green]>>>>>>> Function classes initialized.")

    # 开一个线程用于处理 rospy.spin()
    # 确保 daemon=True，这样主进程结束后，这个线程也会被结束
    # 不然，线程会一直运行，即便主进程结束
    spinThread = threading.Thread(target=rospy.spin, daemon=True)
    spinThread.start()
    console.print("[green]>>>>>>> ROS spin started.")
    
    # 初始化标志位
    isObsAvoidEnable = True
    isInitalBackStableEnable = True

    isGoingOutPlan = False
    isDockNearbyPlan = False
    isDockMeasurePlan = False
    isDockApproachPlan = False
    isDockSteadyPlan = False
    isDockAttachPlan = False
    isDockWaitFinalPlan = False
    isTestPlan = False

    isTestEnable = False

    # 无人船状态
    usvState = "SELF_CHECK"

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

    # 甲板中心的坐标和无人船到甲板的方向角（无人船船体坐标系下）
    deckCenterX = 0
    deckCenterY = 0
    deckyaw = 0

    isReleaseAttachStruct = 0

    # 试一下  
    while not rospy.is_shutdown():
        try:
            # 打印当前状态
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP)
            
            console.clear()
            console.print(theTable)

            # 写入当前状态到文件
            usvData.saveData(usvState, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP)

            # 发送无人船的状态
            usvComm.sendUSVState(usvState)

            # 发送小物体搬运所需
            usvComm.sendTVPosFromLidar(deckCenterX, deckCenterY, deckyaw)

            # 发送控制固连结构是否释放信号
            usvComm.releaseAttachStruct(isReleaseAttachStruct)

            if usvState == "SELF_CHECK":
                # 单独为激光雷达设置启动检查
                pubTopicList = sum(rospy.get_published_topics(), [])
                usvPose.isLidarValid = ('/filter/target' in pubTopicList)

                if (usvPose.isImuValid) & (usvPose.isDvlValid) & (usvPose.isPodValid) & (usvPose.isLidarValid) & \
                    (not isnan(usvControl.angleLeftEst)) & (not isnan(usvControl.angleRightEst)) & \
                    (not isnan(usvControl.rpmLeftEst) & (not isnan(usvControl.rpmRightEst))):
                    if (isTestEnable):
                        usvState = "PURSUE_POD"
                        continue
                    latestMsg = "Self check complete. Start checking comms..."
                    usvState = "COMM_TEST" ####### ALERT #######
                    continue
                
                if (isInitalBackStableEnable):
                    usvControl.thrustSet(RPM_START, RPM_START, 0, 0)
                    usvControl.thrustPub()

            elif usvState == "COMM_TEST":
                if (usvComm.suavState == "COMM_TEST" or usvComm.suavState == "READY" or usvComm.suavState == "COUNTDOWN" or usvComm.suavState == "SEARCH" or usvComm.suavState == "GUIDE") & (usvComm.tuav1State == "COMM_TEST" or usvComm.tuav1State == "READY" or usvComm.tuav1State == "WAIT"):
                    latestMsg = "Waiting sUAV countdown..."
                    usvState = "READY"

                if (isInitalBackStableEnable):
                    usvControl.thrustSet(RPM_START, RPM_START, 0, 0)
                    usvControl.thrustPub()

            elif usvState == "READY":
                if (usvComm.suavState == "COUNTDOWN" or usvComm.suavState == "SEARCH" or usvComm.suavState == "GUIDE"):
                    latestMsg = "Waiting sUAV to provide headings..."
                    usvState = "STANDBY"

                if (isInitalBackStableEnable):
                    usvControl.thrustSet(RPM_START, RPM_START, 0, 0)
                    usvControl.thrustPub()
            
            elif usvState == "STANDBY":                   
                if (usvComm.suavState == "DOCK") | (usvComm.suavState == "GUIDE"):
                    usvState = "GOING_OUT"
                    latestMsg = "USV is going out from the jetty..."
                    continue
                
                if (isInitalBackStableEnable):
                    usvControl.thrustSet(RPM_START, RPM_START, 0, 0)
                    usvControl.thrustPub()
                
            elif usvState == "GOING_OUT":
                if (isGoingOutPlan == False):
                    timer1 = rospy.Time.now().to_sec()
                    yawSP = usvPose.yaw
                    isGoingOutPlan = True

                if (rospy.Time.now().to_sec() - timer1 <= SECS_GOING_OUT):
                    uSP = USP_GOINT_OUT
                else:
                    uSP = 0
                
                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

                if (usvComm.suavState == "GUIDE"):
                    usvState = "PURSUE_SUAV"
                    continue

            elif usvState == "PURSUE_SUAV":
                if (norm([usvPose.tvEstPosX, usvPose.tvEstPosY]) >= DIST_ALLOW_POD):
                    usvPose.podReset()

                # 如果吊舱识别，则进入到吊舱导引
                if (usvPose.isPodFindTV) & (abs(usvPose.tvAnglePod - usvPose.tvAngleEst) <= ANGLE_EST_POD_GAP):####### ALERT #######
                    usvState = "PURSUE_POD"
                    continue
                else:
                    usvPose.podReset()
                    
                # 如果激光雷达识别，则进入到 LIDAR 导引
                if (usvPose.isLidarFindTV):
                    usvState = "PURSUE_LIDAR"
                    continue    

                # 如果遇到障碍物，避障 
                if (usvPose.isLidarFindObs) & (isObsAvoidEnable):                  
                    usvState = "PURSUE_OBS"
                    continue 
                
                # 接收 SUAV 的航向信息
                latestMsg = f"Following heading {rad2deg(usvPose.tvAngleEst):.2f} deg from sUAV."
                uSP = USP_SUAV_PURSUE
                yawSP = usvPose.tvAngleEst
                
                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

            elif usvState == "PURSUE_POD":
                # 如果激光雷达识别，则进入到 LIDAR 导引
                if (usvPose.isLidarFindTV):
                    usvState = "PURSUE_LIDAR"
                    continue       
                
                # 如果吊舱没有识别，则退回到 SUAV 导引
                if (not usvPose.isPodFindTV):
                    yawSP = usvPose.yaw
                    usvState = "PURSUE_SUAV"
                    continue

                # 使用吊舱的角度控制无人船
                latestMsg = f"Following heading {rad2deg(usvPose.tvAnglePod):.2f} deg from pod."    
                uSP = USP_POD_PURSUE
                yawSP = usvPose.tvAnglePod
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)
                
            elif usvState == "PURSUE_POD_LOST":
                if (usvPose.isPodFindTV):
                    usvState = "PURSUE_POD"
                    continue

                # 让无人船以吊舱最后发现目标船的角度停下来
                latestMsg = f"Pod lost the target. Stopping and wait."
                uSP = 0
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

            elif usvState == "PURSUE_LIDAR":
                # 如果激光雷达识别，且距离小于给定值，则进入 Dock 段
                if (usvPose.isLidarFindTV) & (usvPose.tvDist < DIST_PURSUE_TO_APPROACH):
                    usvState = "DOCK_NEARBY"
                    continue    

                # 如果激光雷达没有识别，则退回到吊舱导引
                if (not usvPose.isLidarFindTV):
                    usvState = "PURSUE_POD"
                    continue
                
                # 激光雷达找到目标船，则使用激光雷达的信息
                latestMsg = f"Following heading {rad2deg(usvPose.tvAngleLidar):.2f} deg from Lidar. Distance {usvPose.tvDist:.2f}/{DIST_PURSUE_TO_APPROACH}m to dock."
                uSP = linearClip(DIST_LIDAR_PURSUE_LB, USP_LIDAR_PURSUE_LB, DIST_LIDAR_PURSUE_UB, USP_LIDAR_PURSUE_UB, usvPose.tvDist)
                yawSP = usvPose.tvAngleLidar

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)
                
            elif usvState == "PURSUE_OBS":
                # 判断是否还需要避障，如果不需要，则回到 PURSUE_SUAV
                if (not usvPose.isLidarFindObs):  
                    usvState = "PURSUE_SUAV"
                    continue

                # 计算避障所需航向角
                uSP = USP_OBS_PURSUE
                yawSP = usvPose.obsAngleLidar - ANGLE_AVOID_OBS   
                latestMsg = f"Obstacle detected. Follwing heading {rad2deg(yawSP):.2f} deg to avoid it."

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)
        
            elif usvState == "DOCK_NEARBY":
                if (isDockNearbyPlan == False):
                    currPath = usvPathPlanner.planDockNearby(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockNearbyPlan = True     

                latestMsg = f"Approaching to the measure circle. Path [{usvGuidance.currentIdx} >> {usvGuidance.endIdx}]."
                                
                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = USP_DOCK_NEARBY
                [yawSP, xSP, ySP] = usvGuidance.guidance(DIST_TONEXT_DOCK_NEARBY, usvPose.xLidar, usvPose.yLidar, usvPose.yaw, usvPose.betaDVL)

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_MEASURE"
                    continue
            
            elif usvState == "DOCK_MEASURE":             
                # 使用激光雷达读取的位置信息，规划测量路径
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockMeasurePlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = USP_DOCK_MEASURE
                [yawSP, xSP, ySP] = usvGuidance.guidance(DIST_TONEXT_DOCK_MEASURE, usvPose.xLidar, usvPose.yLidar, usvPose.yaw, usvPose.betaDVL)

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

                # 读取目标船的朝向、长宽估计信息，若满足要求，则保存
                tvInfo[0, tvInfoIdx] = usvPose.tvHeading
                tvInfo[1, tvInfoIdx] = usvPose.tvLength
                tvInfo[2, tvInfoIdx] = usvPose.tvWidth
                tvInfoIdx = tvInfoIdx + 1

                latestMsg = f"Estimating target vessel heading: {rad2deg(usvPose.tvHeading):.2f} deg. L: {usvPose.tvLength:.2f} m. W: {usvPose.tvWidth:.2f} m. Path [{usvGuidance.currentIdx} >> {usvGuidance.endIdx}]."

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    # 去除后面未使用的索引
                    tvInfo = tvInfo[:, 0:tvInfoIdx-1]
                    tvHeadings = tvInfo[0, :]
                    tvLengths = tvInfo[1, :]
                    tvWidths = tvInfo[2, :]

                    # 如果标准差大于 ANGLE_DOCK_MEASURE_JUMP，认为大概率是-90°与90°跳变的情况，因此对 tvHeadings 中负角度加一个 pi
                    if (std(tvHeadings) > ANGLE_DOCK_MEASURE_JUMP):
                        tvHeadings[tvHeadings < 0] = tvHeadings[tvHeadings < 0] + pi

                    # 去除离群点
                    tvHeadings = removeOutliers(tvHeadings, 0.087266, 20)
                    tvLengths = removeOutliers(tvLengths, 0.2, 20)
                    tvWidths = removeOutliers(tvWidths, 0.2, 20)

                    # 计算平均值，并将结果角度映射到-90°~90°
                    tvHeadingMean = arctan(tan(mean(tvHeadings)))
                    tvLengthMean = mean(tvLengths)
                    tvWidthMean = mean(tvWidths)                

                    usvState = "DOCK_APPROACH"
                    continue

            elif usvState == "DOCK_APPROACH":
                # 使用激光雷达读取的位置信息，规划变轨路径
                if (isDockApproachPlan == False):
                    currPath = usvPathPlanner.planDockApproach(usvPose.xLidar, usvPose.yLidar, 0, 0, tvHeadingMean)
                    usvGuidance.setPath(currPath)

                    # 保存 yawf
                    yawf = arctan2(currPath[-1, 1] - currPath[-2, 1], currPath[-1, 0] - currPath[-2, 0])
                    
                    isDockApproachPlan = True

                latestMsg = f"Estimating finished with average heading {rad2deg(tvHeadingMean):.2f} deg. L: {tvLengthMean:.2f} m. W: {tvWidthMean:.2f} m. Path [{usvGuidance.currentIdx} >> {usvGuidance.endIdx}]."

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = linearClip(0, USP_DOCK_APPROACH_UB, usvGuidance.endIdx, USP_DOCK_APPROACH_LB, usvGuidance.currentIdx)
                [yawSP, xSP, ySP] = usvGuidance.guidance(DIST_TONEXT_DOCK_APPROACH, usvPose.xLidar, usvPose.yLidar, usvPose.yaw, usvPose.betaDVL)

                # 控制无人船
                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_STEADY"
                    
                    # 重要：清除 LOS yErrPID 的积分项
                    usvGuidance.yErrPID.clearIntResult()

                    continue

            elif usvState == "DOCK_STEADY":
                if (isDockSteadyPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer0 = rospy.Time.now().to_sec()
                    isDockSteadyPlan = True
                
                # 更新航向值
                yawf = updateTVHeading(usvPose.yaw, usvPose.tvHeading)

                # 保持静止
                xSP = 0 + (0.5 * tvWidthMean + L_HALF) * cos(yawf - pi / 2)
                ySP = 0 + (0.5 * tvWidthMean + L_HALF) * sin(yawf - pi / 2)
                yawSP = yawf
                [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, yawSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.yaw, usvPose.r)

                latestMsg = f"Try to stablize at [{xSP:.2f}, {ySP:.2f}]m, {rad2deg(yawSP):.2f}deg. Pos tol: [{norm([usvPose.xLidar - xSP, usvPose.yLidar - ySP]):.2f}/{DIST_DOCK_STEADY_TOL}]m. Vel tol: [{norm([usvPose.uDVL, usvPose.vDVL]):.2f}/{VEL_DOCK_STEADY_TOL}]m/s."

                # 测量最高点
                # 读取目标船的最高点信息
                tvHighestXYZs[0, tvHighestInfoIdx] = usvPose.tvHighestX
                tvHighestXYZs[1, tvHighestInfoIdx] = usvPose.tvHighestY
                tvHighestXYZs[2, tvHighestInfoIdx] = usvPose.tvHighestZ
                tvHighestInfoIdx = tvHighestInfoIdx + 1
                
                # 一旦船靠近到阈值以下范围或超时，进入固连
                if ((norm([usvPose.xLidar - xSP, usvPose.yLidar - ySP]) <= DIST_DOCK_STEADY_TOL) & (norm([usvPose.uDVL, usvPose.vDVL]) <= VEL_DOCK_STEADY_TOL)) | (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_DOCK_STEADY):   
                    tvHighestXYZs = tvHighestXYZs[:, 0:tvHighestInfoIdx-1]
                    tvHighestXs = removeOutliers(tvHighestXYZs[0, :], 0.1, 20)
                    tvHighestYs = removeOutliers(tvHighestXYZs[1, :], 0.1, 20)
                    tvHighestZs = removeOutliers(tvHighestXYZs[2, :], 0.1, 20)
                    tvHighestXMean = mean(tvHighestXs)
                    tvHighestYMean = mean(tvHighestYs)
                    tvHighestZMean = mean(tvHighestZs)

                    usvState = "DOCK_ATTACH"                  
                    continue
            
            elif usvState == "DOCK_ATTACH":
                if (isDockAttachPlan == False):
                    # 将当前时间写入 t1 计时器
                    timer0 = rospy.Time.now().to_sec()
                    timer1 = rospy.Time.now().to_sec()

                    isDockAttachPlan = True

                # 通过目标船在无人船船体系下的坐标计算两船侧向的距离
                yawf = updateTVHeading(usvPose.yaw, usvPose.tvHeading)
                yawSP = yawf
                lateralDist = abs(usvPose.tvYBody)

                if (usvControl.angleLeftEst < pi / 2) | (usvControl.angleRightEst < pi / 2):
                    usvControl.thrustSet(0, 0, ANGLE_LEFT_ATTACH, ANGLE_RIGHT_ATTACH)
                else:
                    usvControl.thrustSet(RPM_ATTACH, RPM_ATTACH, ANGLE_LEFT_ATTACH, ANGLE_RIGHT_ATTACH)
                usvControl.thrustPub()
                
                latestMsg = f"Attaching to the target vessel. Pos tol: [{lateralDist:.2f}/{DIST_ATTACH_TOL + 0.5 * tvWidthMean + L_HALF:.2f}]m. Time tol: [{rospy.Time.now().to_sec() - timer1:.2f}/{SECS_WAIT_ATTACH}]s"

                # 固连成功判据：距离，或者超时
                if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_ATTACH):   
                    usvState = "DOCK_WAIT_FINAL"
                    continue
                elif (lateralDist <= DIST_ATTACH_TOL + 0.5 * tvWidthMean + L_HALF):
                    pass
                elif (usvControl.angleLeftEst < pi / 2) | (usvControl.angleRightEst < pi / 2):
                    # 如果推力方向未到位置，则重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                # 超时
                if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_ATTACH):
                    usvState = "DOCK_WAIT_FINAL"
                    continue

            elif usvState == "DOCK_WAIT_FINAL":
                # 先判断无人船是否稳定
                if (isDockWaitFinalPlan == "False"):
                    timer0 = rospy.Time.now().to_sec()
                    timer1 = rospy.Time.now().to_sec()
                    isDockWaitFinalPlan = True

                # 计算给小物体搬运的坐标点
                yawf = updateTVHeading(usvPose.yaw, usvPose.tvHeading)
                yawSP = yawf
                [xf, yf] = calcHighest(tvHighestXMean, tvHighestYMean, tvHighestZMean, tvLengthMean, yawf)
                [deckCenterX, deckCenterY] = rotationZ(-usvPose.xLidar + xf, -usvPose.yLidar + yf, usvPose.yaw)
                deckyaw = yawf - usvPose.yaw

                latestMsg = f"Attached completed. Waiting USV stablized to send TAKEOFF signal [{norm([usvPose.uDVL, usvPose.vDVL]):.2f}/{VEL_WAIT_FINAL:.2f}]m/s & [{rospy.Time.now().to_sec() - timer1:.2f}/{SECS_WAIT_FINAL:.2f}]s. Real-time deck point at [{deckCenterX:.2f}, {deckCenterY:.2f}]m @ {rad2deg(deckyaw):.2f}deg."

                # usvControl.thrustSet(120, 120, deg2rad(105), deg2rad(96))
                # usvControl.thrustPub()
                
                # 如果稳定，则发送起飞状态
                if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_FINAL):   
                    usvState = "DOCK_FINAL"
                    continue
                elif (norm([usvPose.uDVL, usvPose.vDVL]) <= VEL_WAIT_FINAL):
                    pass
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

                if (rospy.Time.now().to_sec() - timer0 > SECS_TIMEOUT_WAIT_FINAL):
                    usvState = "DOCK_FINAL"
                    continue

            elif usvState == "DOCK_FINAL":      
                # 计算给小物体搬运的坐标点
                yawf = updateTVHeading(usvPose.yaw, usvPose.tvHeading)
                yawSP = yawf
                [xf, yf] = calcHighest(tvHighestXMean, tvHighestYMean, tvHighestZMean, tvLengthMean, yawf)
                [deckCenterX, deckCenterY] = rotationZ(-usvPose.xLidar + xf, -usvPose.yLidar + yf, usvPose.yaw)
                deckyaw = yawf - usvPose.yaw

                usvControl.thrustSet(RPM_FINAL, RPM_FINAL, ANGLE_LEFT_FINAL, ANGLE_RIGHT_FINAL)
                usvControl.thrustPub()

                latestMsg = f"TAKEOFF signal sent!!! Real-time deck point at [{deckCenterX:.2f}, {deckCenterY:.2f}]m @ {rad2deg(deckyaw):.2f}deg."              

            elif usvState == "TEST":
                uSP = 2.25            
                if (isTestPlan == False):              
                    yawSP = wrapToPi(usvPose.yaw + deg2rad(0))
                    isTestPlan = True

                [uSP, rSP, axbSP, etaSP] = usvControl.moveUSV(uSP, yawSP, usvPose.uDVL, usvPose.axb, usvPose.yaw, usvPose.r)

            else:
                # 程序不应该执行到这里
                console.print(f"\n[red] >>>>>>> USV state: {usvState} invalid. Check code.")
                break
            
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
