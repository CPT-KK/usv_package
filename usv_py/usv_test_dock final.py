#!/usr/bin/env python3

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
from usv_math import removeOutliers, wrapToPi, linearClip, rotationZ
from usv_record import genTable, USVData
from usv_constants import *

@atexit.register 
def clean():
    print(">>>>>>> USV program has exited.")

def interuptFunc(signum, frame):
    console = Console()
    console.print("\n[red]>>>>>>> Ctrl + C pressed! Exiting...")
    exit()

def updateTVHeading(existHeading, newHeading):
    newHeading2 = wrapToPi(newHeading + pi)
    angleGap1 = abs(wrapToPi(newHeading - existHeading))
    angleGap2 = abs(wrapToPi(newHeading2 - existHeading))

    if (angleGap1 <= angleGap2):
        return wrapToPi(newHeading)
    else:
        return wrapToPi(newHeading2)

def calcHighest(tvHighestXMean, tvHighestYMean, tvHighestZMean, tvLengthMean, yawf):
    if (tvHighestZMean >= HEALTHY_Z_TOL):   
        # 最高点测量健康，向最高点映射到船中轴线上的点泊近
        # 注意：这里的 rotationZ 是要对点（向量）进行旋转，即求取点在旋转后的坐标（同一坐标系下），
        # 而不是同一个点在不同坐标系下的表示，故取负号
        [tvHighestXMean2, _] = rotationZ(tvHighestXMean, tvHighestYMean, yawf)
        if (tvHighestXMean2 >= 0):
            xf = (-0.5 * tvLengthMean + tvHighestXMean2) / 2
            yf = 0.0
        else:
            xf = (0.5 * tvLengthMean + tvHighestXMean2) / 2
            yf = 0.0
        [xf, yf] = rotationZ(xf, yf, -yawf)
    else: 
        # 最高点测量不健康，设置为目标船中心
        xf = 0.0
        yf = 0.0

    return [xf, yf]

def main():
    # 控制台输出初始化
    console = Console(record=True)
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
    isInitalBackStableEnable = False

    isGoindOutPlan = False
    isDockNearbyPlan = False
    isDockMeasurePlan = False
    isDockApproachPlan = False
    isDockSteadyPlan = False
    isDockAttachPlan = False
    isDockWaitFinalPlan = False
    isDockSteadyFSPlan = False

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

    usvPose.isLidarFindTVPrevious = True

    while (True):
        # 单独为激光雷达设置启动检查
        pubTopicList = sum(rospy.get_published_topics(), [])
        usvPose.isLidarValid = ('/filter/target' in pubTopicList)

        # 打印当前状态
        dt = rospy.Time.now().to_sec() - t0
        theTable = genTable(usvState, latestMsg, usvPose, usvControl, usvComm, dt, uSP, vSP, yawSP, rSP, xSP, ySP, axbSP, aybSP, etaSP) 
        console.print(theTable)

        if (usvPose.isImuValid) & (usvPose.isDvlValid) & (not isnan(usvControl.angleLeftEst)) & (not isnan(usvControl.angleRightEst)) & (not isnan(usvControl.rpmLeftEst) & (not isnan(usvControl.rpmRightEst))):
            break
    
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
    tvHighestXMean = 0
    tvHighestYMean = 0
    tvHighestZMean = 0

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
            if (isDockSteadyPlan == False):
                # 将当前时间写入 t1 计时器
                timer0 = rospy.Time.now().to_sec()
                timer1 = rospy.Time.now().to_sec()
                isDockSteadyPlan = True
            
            # 更新航向值
            yawf = updateTVHeading(usvPose.yaw, usvPose.tvHeading)

            # 保持静止
            xSP = 0 + (0.5 * tvWidthMean + L_HALF) * cos(yawf - pi / 2)
            ySP = 0 + (0.5 * tvWidthMean + L_HALF) * sin(yawf - pi / 2)
            yawSP = yawf
            [uSP, vSP, rSP, axbSP, aybSP, etaSP] = usvControl.moveUSVVec(xSP, ySP, yawSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.yaw, usvPose.r)

            latestMsg = f"Try to stablize at [{xSP:.2f}, {ySP:.2f}]m, {rad2deg(yawSP):.2f}deg. Pos tol: [{norm([usvPose.xLidar - xSP, usvPose.yLidar - ySP]):.2f}/{DIST_DOCK_STEADY_TOL:.2f}]m. Vel tol: [{norm([usvPose.uDVL, usvPose.vDVL]):.2f}/{0.3}]m/s."
            
            # 一旦船靠近到阈值以下范围，进入固连
            if (norm([usvPose.xLidar - xSP, usvPose.yLidar - ySP]) <= DIST_DOCK_STEADY_TOL) & (norm([usvPose.uDVL, usvPose.vDVL]) <= 0.3):   
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

                isDockAttachPlan = True

            # 通过目标船在无人船船体系下的坐标计算两船侧向的距离
            yawf = updateTVHeading(usvPose.yaw, usvPose.tvHeading)
            lateralDist = abs(usvPose.tvYBody)

            if (usvControl.angleLeftEst < deg2rad(90)) | (usvControl.angleRightEst < deg2rad(90)):
                usvControl.thrustSet(0, 0, deg2rad(105), deg2rad(96))
            else:
                usvControl.thrustSet(400, 400, deg2rad(105), deg2rad(96))
            usvControl.thrustPub()
            
            latestMsg = f"Attaching to the target vessel. Pos tol: [{lateralDist:.2f}/{DIST_ATTACH_TOL + 0.5 * tvWidthMean + L_HALF:.2f}]m. Time tol: [{rospy.Time.now().to_sec() - timer1:.2f}/{SECS_WAIT_ATTACH}]s"

            # 固连成功判据：距离，或者超时
            if (rospy.Time.now().to_sec() - timer1 > SECS_WAIT_ATTACH):   
                usvState = "DOCK_WAIT_FINAL"
                continue
            elif (lateralDist <= DIST_ATTACH_TOL + 0.5 * tvWidthMean + L_HALF):
                pass
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
            [xf, yf] = calcHighest(tvHighestXMean, tvHighestYMean, tvHighestZMean, tvLengthMean, yawf)
            [deckCenterX, deckCenterY] = rotationZ(-usvPose.xLidar + xf, -usvPose.yLidar + yf, usvPose.yaw)
            deckyaw = yawf - usvPose.yaw

            usvControl.thrustSet(80, 80, deg2rad(105), deg2rad(96))
            usvControl.thrustPub()

            latestMsg = f"TAKEOFF signal sent!!! Real-time deck point at [{deckCenterX:.2f}, {deckCenterY:.2f}]m @ {rad2deg(deckyaw):.2f}deg."              
            
        rosRate.sleep()

        
if __name__ == '__main__':
    main()