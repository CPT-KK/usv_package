#!/usr/bin/env python3

import rospy
import threading
import atexit
import traceback
import signal

from numpy import zeros, rad2deg, median, deg2rad, sin, cos, pi, abs, min, argmin, mean, tan, arctan, arctan2, std

from usv_can import USVCAN
from usv_pose import Pose
from usv_path_planner import PathPlanner, planCirclePath
from usv_guidance import Guidance
from usv_control import Control
from usv_communication import Communication
from usv_math import removeOutliers
from usv_record import genTable, USVData
from usv_test import test
from rich.console import Console
from rich.table import Column, Table

from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

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
    usvCAN = USVCAN()
    usvPose = Pose()
    usvComm = Communication()
    usvPathPlanner = PathPlanner()
    usvGuidance = Guidance(ROS_RATE)
    usvControl = Control(ROS_RATE)
    usvData = USVData(ROS_RATE)
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
    isTestLinePlan = False
    isTestCirclePlan = False
    isTestEnable = False

    # 无人船状态
    usvState = "STANDBY"

    # 无人船当前正在使用的路径
    currPath = zeros((2000, 2))

    # 记录程序启动时间
    t0 = rospy.Time.now().to_sec()

    # 设置计时器
    timer1 = rospy.Time.now().to_sec()

    # 保存目标船朝向角的数组
    tvHeadings = zeros((1, 5000))
    tvHeadingIdx = 0

    # Set point 量
    uSP = float("nan")
    vSP = float("nan")
    psiSP = float("nan")
    xSP = float("nan")
    ySP = float("nan")

    # 试一下  
    while not rospy.is_shutdown():
        try:
            if usvState == "STARTUP":
                # 单独为激光雷达设置启动检查
                pubTopicList = sum(rospy.get_published_topics(), [])
                usvPose.isLidarValid = ('/filter/target' in pubTopicList)
                    
                if (usvPose.isImuValid) & (usvPose.isDvlValid) & (usvPose.isPodValid) & (usvPose.isLidarValid):
                    latestMsg = "Waiting sUAV to send heading..."
                    usvState = "STANDBY"

            elif usvState == "STANDBY":
                if (isTestEnable):
                    testRet = test(TEST_MODE, usvPathPlanner, usvGuidance, usvControl, usvPose)
                    if (testRet == 1):
                        break
                    else:
                        continue
                    
                if (usvComm.isSearchFindTV):
                    latestMsg = "Receive heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)
                    # mavrosArmClient.call(armCmd)
                    usvState = "PURSUE"

            elif usvState == "PURSUE":   
                # 如果距离小于给定值，则进入 Approach 段
                if (usvPose.isLidarFindTV) & (usvPose.tvDist < 70.0):
                    usvState = "DOCK_NEARBY"
                    latestMsg = "Approaching to the measure circle..."
                    continue        
                 
                if (usvPose.isLidarFindTV):
                    # 如果激光雷达找到目标船，则使用激光雷达的信息
                    latestMsg = "Lidar finds target vessel at %.2f deg in %.2f m!" % (rad2deg(usvPose.tvAngleLidar), usvPose.tvDist)
                    if (usvPose.tvDist < 75.0):
                        uSP = 1.7
                    elif (usvPose.tvDist > 100.0):
                        uSP = 3.0
                    else:
                        uSP = 1.7 + (3.0 - 1.7) * (usvPose.tvDist - 75.0) / (100.0 - 75.0)

                    psiSP = usvPose.tvAngleLidar

                elif (usvPose.isPodFindTV):
                    # 如果激光雷达没有找到目标船，则使用吊舱的信息
                    latestMsg = "Pod finds target vessel at %.2f deg!" % rad2deg(usvPose.tvAnglePod)
                    uSP = 3.25
                    psiSP = usvPose.tvAnglePod

                elif (usvPose.isLidarFindObs) & (isObsAvoidEnable):
                    # 遇到障碍物，避障   
                    usvState = "PURSUE_OBS"
                    continue 

                else:
                    # 如果没有找到目标船，则继续跟随追踪路径
                    latestMsg = "Receive heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)
                    uSP = 3.25
                    psiSP = usvComm.tvAngleEst          
                
                # 控制无人船
                uSP = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)
                
            elif usvState == "PURSUE_OBS":             
                # 判断是否还需要避障
                if (usvPose.isLidarFindObs):       
                    # 计算避障所需航向角
                    if (usvPose.obsAngleLidar >= 0):
                        psiSP = usvPose.obsAngleLidar - deg2rad(35.0)
                    else:
                        psiSP = usvPose.obsAngleLidar - deg2rad(35.0)
                    
                    uSP = 3.25
                    uSP = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                    latestMsg = "Obstacle detected at %.2f deg!" % usvPose.obsAngleLidar
                else:
                    latestMsg = "Back to follow heading %d deg from sUAV." % rad2deg(usvComm.tvAngleEst)     
                    usvState = "PURSUE"
                    continue
            
            elif usvState == "DOCK_NEARBY":
                if (isDockNearbyPlan == False):
                    currPath = usvPathPlanner.planDockNearby(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockNearbyPlan = True       
                
                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = 1.6
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(uSP, 12.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                uSP = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_MEASURE"
            
            elif usvState == "DOCK_MEASURE":             
                # 使用激光雷达读取的位置信息，规划测量路径
                if (isDockMeasurePlan == False):
                    currPath = usvPathPlanner.planDockMeasure(usvPose.xLidar, usvPose.yLidar, 0, 0)
                    usvGuidance.setPath(currPath)
                    isDockMeasurePlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = 1.5
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(uSP, 8.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                uSP = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                # 读取目标船的测量信息，若满足要求，则读取并保存目标船朝向角（ENU下）
                thisHeading = usvPose.tvHeading
                tvHeadings[0, tvHeadingIdx] = thisHeading
                tvHeadingIdx = tvHeadingIdx + 1
                latestMsg = "Estimating target vessel heading: %.2f deg." % rad2deg(thisHeading)

                # 如果测量段结束了，打印出测量段测量结果，进入变轨段
                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    # 去除后面未使用的索引
                    tvHeadings = tvHeadings[0, 0:tvHeadingIdx-1]

                    # 如果标准差大于 20°(0.34906585 rad)，认为大概率是-90°与90°跳变的情况，因此对 tvHeadings 中负角度加一个 pi
                    if (std(tvHeadings) > 0.34906585):
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
                    currPath = usvPathPlanner.planDockApproach2(usvPose.xLidar, usvPose.yLidar, 0, 0, tvHeadingMean)
                    usvGuidance.setPath(currPath) 
                    isDockApproachPlan = True

                # 读取激光雷达信息（这个时候应该能保证读到目标船吧？），生成控制指令
                uSP = 1.5 - 0.7 * (usvGuidance.currentIdx / usvGuidance.endIdx)
                [uSP, psiSP, xSP, ySP] = usvGuidance.guidance(uSP, 6.0, usvPose.xLidar, usvPose.yLidar, usvPose.psi, usvPose.betaDVL)

                # 控制无人船
                uSP = usvControl.moveUSV(uSP, psiSP, usvPose.uDVL, usvPose.axb, usvPose.psi, usvPose.r)

                if (usvGuidance.currentIdx >= usvGuidance.endIdx):
                    usvState = "DOCK_ADJUST"
                    
                    # 重要：清除 LOS yErrPID 和差分控制器 PID 的积分项
                    usvGuidance.yErrPID.clearIntResult()
                    usvControl.uPID.clearIntResult()

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

                    latestMsg = "Approach finished. Stablizing USV pose at [%.2f, %.2f] @ %.2f deg for %.2f / %.2f secs..." % (xSP, ySP, rad2deg(psiSP), rospy.Time.now().to_sec() - timer1, 5.0)

                # 保持静止
                [uSP, vSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

                # 等待船接近静止并保持 5.0s，进入 FINAL
                if (rospy.Time.now().to_sec() - timer1 > 5.0):   
                    usvState = "DOCK_FINAL"
                elif (abs(usvPose.psi - psiSP) < deg2rad(2.0)) & (abs(usvPose.xLidar - xSP) < 1.0) & (abs(usvPose.yLidar - ySP) < 1.0) & (abs(usvPose.uDVL) < 0.25) & (abs(usvPose.vDVL) < 0.25):
                    pass
                else:
                    # 如果不满足静止条件，需要重置 t1 计时器
                    timer1 = rospy.Time.now().to_sec()

            elif usvState == "DOCK_FINAL":
                # DOCK_FINAL 是一个死循环
                # if (usvPose.state.armed):
                    # mavrosArmClient.call(disarmCmd)
                    
                latestMsg = "USV has been stabilized. Start to send tUAV take-off flag..."
                usvComm.sendTakeOffFlag()
                usvComm.sendTVPosFromLidar(-usvPose.xLidar, -usvPose.yLidar)

                # 继续保持静止
                [uSP, vSP] = usvControl.moveUSVVec(xSP, ySP, psiSP, usvPose.xLidar, usvPose.yLidar, usvPose.uDVL, usvPose.vDVL, usvPose.axb, usvPose.ayb, usvPose.psi, usvPose.r)

            else:
                # 程序不应该执行到这里
                console.print("\n[red] >>>>>>> USV state: %s invalid. Check code." % (usvState))
                break
        
            # 打印当前状态
            dt = rospy.Time.now().to_sec() - t0
            theTable = genTable(usvState, latestMsg, usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP) 
            console.print(theTable)

            # 写入当前状态到文件
            usvData.saveData(usvCAN, usvPose, usvComm, dt, uSP, vSP, psiSP, xSP, ySP)

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
